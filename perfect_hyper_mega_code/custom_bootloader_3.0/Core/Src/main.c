/* USER CODE BEGIN Header */

/**

  ******************************************************************************

  * @file           : main.c

  * @brief          : STM32F303RE Custom OTA Bootloader

  ******************************************************************************

  *

  * MEMORY MAP

  *

  * Bootloader:

  *     0x08000000 - 0x08007FFF

  *     32 KB

  *

  * Application:

  *     0x08008000 - 0x0807FFFF

  *     480 KB

  *

  * UARTS

  *

  * USART2:

  *     ST-LINK Virtual COM Port

  *     Debug messages

  *     115200 8-N-1

  *

  * USART3:

  *     EC200U modem

  *     PB10 = TX

  *     PB11 = RX

  *     115200 8-N-1

  *

  ******************************************************************************

  */

/* USER CODE END Header */

#include "main.h"

#include <stdint.h>

#include <string.h>

#include <stdio.h>

#include <stdlib.h>



/* -------------------------------------------------------------------------- */

/* Private variables                                                          */

/* -------------------------------------------------------------------------- */

UART_HandleTypeDef huart2;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

#define OTA_STREAM_DMA_BUFFER_SIZE  16384U
static uint8_t g_otaStreamDmaBuffer[OTA_STREAM_DMA_BUFFER_SIZE];
static uint32_t g_otaStreamReadPos = 0U;
static uint8_t g_otaStreamDmaActive = 0U;



/* -------------------------------------------------------------------------- */

/* Defines                                                                    */

/* -------------------------------------------------------------------------- */
#define FLASH_START_ADDRESS       0x08000000UL

#define APPLICATION_ADDRESS       0x08010000UL

#define OTA_CONFIG_ADDRESS        0x0807F800UL
#define OTA_CONFIG_SIZE           0x800UL

#define APPLICATION_MAX_SIZE      (OTA_CONFIG_ADDRESS - APPLICATION_ADDRESS)

#define UPDATE_FILE_NAME          "UFS:firmware.bin"

#define MODEM_READ_CHUNK_SIZE     256U

#define MODEM_TIMEOUT             5000U

/* -------------------------------------------------------------------------- */
/* Authenticated AWS API Gateway OTA endpoint                                 */
/* -------------------------------------------------------------------------- */
/*#define OTA_API_HOST              "70br5ujjbb.execute-api.ap-south-1.amazonaws.com"
#define OTA_API_PATH              "/firmware"  */

static char g_firmware_filename[128];

/*
 * This must exactly match the OTA_SECRET environment variable used by the
 * Lambda authorizer OTAFirmwareAuthorizer.
 *
 * IMPORTANT: Do NOT print this token through Debug_Print().
 */



/* -------------------------------------------------------------------------- */

/* Function prototypes                                                        */

/* -------------------------------------------------------------------------- */

#define OTA_CONFIG_MAGIC    0x4F544143UL
#define OTA_CONFIG_VERSION  1U

typedef struct
{
    uint32_t magic;
    uint32_t version;

    char firmware_name_url[256];
    char firmware_download_url[256];
    char update_success_url[256];
    char firmware_delete_url[256];

    char ota_auth_token[64];

    uint32_t crc;

} OTA_Config_t;

static const OTA_Config_t *OTA_GetConfig(void)
{
    return (const OTA_Config_t *)OTA_CONFIG_ADDRESS;
}


static int OTA_ConfigIsValid(void)
{
    const OTA_Config_t *config = OTA_GetConfig();

    /*if (config->magic != OTA_CONFIG_MAGIC)
    {
        return 0;
    }*/

    if (config->version != OTA_CONFIG_VERSION)
    {
        return 0;
    }

    if (config->firmware_name_url[0] == '\0')
    {
        return 0;
    }

    if (config->firmware_download_url[0] == '\0')
    {
        return 0;
    }

    if (config->update_success_url[0] == '\0')
    {
        return 0;
    }

    if (config->firmware_delete_url[0] == '\0')
    {
        return 0;
    }

    if (config->ota_auth_token[0] == '\0')
    {
        return 0;
    }

    return 1;
}



void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_USART2_UART_Init(void);

static void MX_USART3_UART_Init(void);
static int OTA_StreamDMA_Start(void);
static void OTA_StreamDMA_Stop(void);
static uint32_t OTA_StreamDMA_WritePos(void);
static uint32_t OTA_StreamDMA_Available(void);
static int OTA_StreamDMA_ReadByte(uint8_t *byte, uint32_t timeout);
static int OTA_StreamDMA_Read(uint8_t *buffer, uint32_t length, uint32_t timeout);
static void MX_USART3_DMA_Init(void);

static void Debug_Print(const char *message);

static void Modem_Flush(void);

static int Modem_ReadNormal(

    char *response,

    uint32_t responseSize,

    uint32_t timeout

);

/*

 * EC200U AT commands are terminated with carriage return only (\r).

 * Modem responses still use CR/LF and are parsed normally.

 */

static int Modem_SendCommand(const char *command,char *response,uint32_t responseSize,uint32_t timeout);

static int Modem_GetFirmwareFilename(char *filename,uint32_t filenameSize);

static int Modem_SendRawCommand(const char *command);

static int Modem_WaitFor( const char *expected,uint32_t timeout);

static int Modem_WaitForQFReadHeader(uint32_t expectedLength,uint32_t timeout)

{

    char line[64];

    uint32_t index = 0;

    uint32_t start = HAL_GetTick();

    uint8_t ch;

    char *connect;

    memset(line, 0, sizeof(line));

    while ((HAL_GetTick() - start) < timeout)

    {

        if (HAL_UART_Receive(

                &huart3,

                &ch,

                1,

                100) != HAL_OK)

        {

            continue;

        }

        /*

         * Keep the most recent bytes only. The modem may echo the

         * QFREAD command before sending:

         *

         *   CONNECT <read_length>\r\n

         *

         * Therefore we must search for CONNECT inside the received

         * stream instead of assuming CONNECT is at byte zero.

         */

        if (index < sizeof(line) - 1U)

        {

            line[index++] = (char)ch;

            line[index] = '\0';

        }

        else

        {

            /*

             * Preserve the tail of the stream so that an echoed command

             * cannot make the parser overflow before CONNECT arrives.

             */

            memmove(

                line,

                line + 1,

                sizeof(line) - 2U

            );

            line[sizeof(line) - 2U] = (char)ch;

            line[sizeof(line) - 1U] = '\0';

            index = sizeof(line) - 1U;

        }

        connect = strstr(line, "CONNECT");

        if (connect != NULL)

        {

            char *p = connect + strlen("CONNECT");

            unsigned long modemLength = 0;

            /*

             * CONNECT may be followed by spaces and the actual number

             * of bytes that will be delivered.

             */

            while (*p == ' ')

            {

                p++;

            }

            if (*p >= '0' && *p <= '9')

            {

                char *endPtr;

                modemLength = strtoul(p, &endPtr, 10);

                /*

                 * We only enter binary mode after the complete

                 * CONNECT <length>\r\n header has arrived.

                 */

                if (*endPtr == '\r')

                {

                    if (endPtr[1] == '\n')

                    {

                        if (modemLength == (unsigned long)expectedLength)

                        {

                            /*

                             * IMPORTANT: do not print anything here.

                             * QFREAD switches immediately to raw binary

                             * mode after the CONNECT header. Any UART2

                             * debug output here delays the CPU and can

                             * cause USART3 overrun/lost firmware bytes.

                             */

                            return 1;

                        }

                        Debug_Print(

                            "ERROR: QFREAD length mismatch.\r\n"

                        );

                        return 0;

                    }

                }

            }

            else if (*p == '\r' && p[1] == '\n')

            {

                /*

                 * Some firmware versions may return just CONNECT.

                 * Accept it because the requested length is already

                 * known to the STM32.

                 */

                /*

                 * Do not print here. Raw firmware bytes may already be

                 * arriving on USART3 immediately after this header.

                 */

                return 1;

            }

        }

        /*

         * If the modem explicitly reports an error before CONNECT,

         * abort the QFREAD operation.

         */

        if (strstr(line, "+CME ERROR:") != NULL ||

            strstr(line, "\r\nERROR\r\n") != NULL)

        {

            Debug_Print(

                "ERROR: EC200U returned an error for QFREAD.\r\n"

            );

            return 0;

        }

    }

    Debug_Print(

        "ERROR: QFREAD CONNECT header timeout.\r\n"

    );

    return 0;

}

static int Modem_HttpRead(
    char *body,
    uint32_t bodySize,
    uint32_t expectedLength);

static int Modem_HttpGet(const char *url,const char *token,char *body,uint32_t bodySize,int *httpStatus,uint32_t *contentLength);


static int Modem_GetFirmwareFilename(char *filename,uint32_t filenameSize)
{
    const OTA_Config_t *config =
        OTA_GetConfig();

    char response[256];

    int httpStatus = 0;
    uint32_t contentLength = 0;

    char *key;
    char *start;
    char *end;

    if (filename == NULL ||
        filenameSize < 2U)
    {
        return 0;
    }

    memset(filename, 0, filenameSize);

    memset(response, 0, sizeof(response));

    Debug_Print(
        "Getting firmware filename from cloud...\r\n"
    );

    if (!Modem_HttpGet(
            config->firmware_name_url,
            config->ota_auth_token,
            response,
            sizeof(response),
            &httpStatus,
            &contentLength))
    {
        Debug_Print(
            "ERROR: Firmware filename request failed.\r\n"
        );

        return 0;
    }

    if (httpStatus != 200)
    {
        Debug_Print(
            "ERROR: Firmware filename API returned non-200.\r\n"
        );

        return 0;
    }

    /*
     * Expected:
     *
     * {"status": "success", "filename": "lll.bin"}
     */

    key = strstr(
        response,
        "\"filename\""
    );

    if (key == NULL)
    {
        Debug_Print(
            "ERROR: filename field not found.\r\n"
        );

        return 0;
    }

    start = strchr(key, ':');

    if (start == NULL)
    {
        return 0;
    }

    start++;

    while (*start == ' ' ||
           *start == '\t')
    {
        start++;
    }

    if (*start != '"')
    {
        return 0;
    }

    start++;

    end = strchr(start, '"');

    if (end == NULL)
    {
        return 0;
    }

    if ((uint32_t)(end - start) >= filenameSize)
    {
        Debug_Print(
            "ERROR: Firmware filename too long.\r\n"
        );

        return 0;
    }

    memcpy(
        filename,
        start,
        (size_t)(end - start)
    );

    filename[end - start] = '\0';

    /*
     * Basic safety check.
     */

    if (strstr(filename, "..") != NULL ||
        strchr(filename, '/') != NULL ||
        strchr(filename, '\\') != NULL)
    {
        Debug_Print(
            "ERROR: Invalid firmware filename.\r\n"
        );

        memset(filename, 0, filenameSize);

        return 0;
    }

    if (strlen(filename) < 5U ||
        strcmp(
            &filename[strlen(filename) - 4U],
            ".bin") != 0)
    {
        Debug_Print(
            "ERROR: Firmware filename is not .bin.\r\n"
        );

        memset(filename, 0, filenameSize);

        return 0;
    }

    Debug_Print(
        "Firmware filename received successfully.\r\n"
    );

    return 1;
}

static int Modem_ReadExactBinary(

    uint8_t *buffer,

    uint32_t length,

    uint32_t timeoutPerByte

);

static int Modem_CheckForUpdate(void);

static int Modem_GetFirmwareSize(

    uint32_t *fileSize

);

static int Modem_OpenFirmware(

    uint32_t *fileHandle

);

static int Modem_ReadFile(

    uint32_t fileHandle,

    uint8_t *buffer,

    uint32_t length

);

static int Modem_CloseFile(

    uint32_t fileHandle

);

static int Modem_DeleteFirmware(void);

static int Modem_ConnectToInternet(void);


static HAL_StatusTypeDef Erase_Application(void);

static int Modem_HttpPostNoBody(const char *url,const char *token,int *httpStatus);

static HAL_StatusTypeDef Flash_WriteBuffer(

    uint32_t address,

    uint8_t *data,

    uint32_t length

);

static int Verify_Application(

    uint32_t firmwareSize

);

static int Perform_Firmware_Update(const char *firmwareFilename);

static int Modem_SetHTTPURL(const char *url);

static int Modem_DownloadFirmwareFromS3(uint32_t *firmwareSize);
static int Modem_HttpReadToFlash(uint32_t expectedLength);
static int Perform_Firmware_Update_Streaming(const char *firmwareFilename, uint32_t firmwareSize);

static int Modem_ReportFirmwareUpdateSuccess(const char *firmwareFilename);

static int Modem_DeleteFirmwareFromS3(void);

static void Jump_To_Application(void);



















/* ------------------------------------------------------------------------- */

static void Debug_Print(const char *message)

{

    if (message == NULL)

    {

        return;

    }

    HAL_UART_Transmit(

        &huart2,

        (uint8_t *)message,

        (uint16_t)strlen(message),

        HAL_MAX_DELAY

    );

}

















/* -------------------------------------------------------------------------- */

/* Modem UART helpers                                                         */

/*                                                                            */

/* This communication layer follows the same approach as the known-working   */

/* EC200U application:                                                       */

/*   1. Flush stale UART data before every normal AT command.                 */

/*   2. Transmit the complete command.                                       */

/*   3. Read the complete normal response until OK/ERROR/CME ERROR.           */

/*                                                                            */

/* QFREAD is special because after the command the modem enters a raw-data   */

/* transfer mode. Therefore QFREAD is sent separately and we wait for        */

/* CONNECT before receiving binary data.                                     */

/* -------------------------------------------------------------------------- */

static void Modem_Flush(void)

{

    uint8_t dummy;

    while (HAL_UART_Receive(

               &huart3,

               &dummy,

               1,

               10) == HAL_OK)

    {

        /* Discard stale modem data. */

    }

}

















/* -------------------------------------------------------------------------- */

/* Read a normal modem response                                               */

/* -------------------------------------------------------------------------- */

static int Modem_ReadNormal(

    char *response,

    uint32_t responseSize,

    uint32_t timeout

)

{

    uint8_t ch;

    uint32_t index = 0;

    uint32_t start = HAL_GetTick();

    if (response == NULL || responseSize < 2)

    {

        return 0;

    }

    memset(response, 0, responseSize);

    while ((HAL_GetTick() - start) < timeout)

    {

        if (HAL_UART_Receive(

                &huart3,

                &ch,

                1,

                20) == HAL_OK)

        {

            if (index < responseSize - 1)

            {

                response[index++] = (char)ch;

                response[index] = '\0';

            }

            /*

             * Normal successful response.

             */

            if (strstr(response, "\r\nOK\r\n") != NULL)

            {

                return 1;

            }

            /*

             * Some EC200U responses can finish with OK without the

             * exact surrounding CR/LF sequence being present in the

             * accumulated buffer.

             */

            if (strstr(response, "\nOK\r\n") != NULL)

            {

                return 1;

            }

            /*

             * Normal ERROR.

             */

            if (strstr(response, "\r\nERROR\r\n") != NULL ||

                strstr(response, "\nERROR\r\n") != NULL)

            {

                return 0;

            }

            /*

             * CME ERROR.

             */

            if (strstr(response, "+CME ERROR:") != NULL)

            {

                return 0;

            }

        }

    }

    return 0;

}

















/* -------------------------------------------------------------------------- */

/* Send a normal AT command and capture its complete response                 */

/* -------------------------------------------------------------------------- */

static int Modem_SendCommand(const char *command,char *response, uint32_t responseSize,uint32_t timeout)

{

    Modem_Flush();

    /*Debug_Print("\r\nSTM32 -> MODEM:\r\n");*/

    Debug_Print(command);

    if (HAL_UART_Transmit(

            &huart3,

            (uint8_t *)command,

            strlen(command),

            HAL_MAX_DELAY) != HAL_OK)

    {

        Debug_Print("\r\nERROR: UART transmit failed.\r\n");

        return 0;

    }

    if (!Modem_ReadNormal(

            response,

            responseSize,

            timeout))

    {

        /*Debug_Print("\r\nMODEM -> STM32:\r\n");*/

        Debug_Print(response);

        return 0;

    }

    Debug_Print("\r\nMODEM -> STM32:\r\n");

    Debug_Print(response);

    return 1;

}


/* -------------------------------------------------------------------------- */

/* Send a command which does NOT return a normal OK response immediately      */

/*                                                                            */

/* QFREAD is the important example. It returns CONNECT and then raw binary.   */

/* -------------------------------------------------------------------------- */

static int Modem_SendRawCommand(

    const char *command

)

{

    Modem_Flush();

    Debug_Print("\r\nSTM32 -> MODEM:\r\n");

    Debug_Print(command);

    return HAL_UART_Transmit(

               &huart3,

               (uint8_t *)command,

               strlen(command),

               HAL_MAX_DELAY) == HAL_OK;

}

















/* -------------------------------------------------------------------------- */

/* Wait for a modem response token                                             */

/* -------------------------------------------------------------------------- */

static int Modem_WaitFor(

    const char *expected,

    uint32_t timeout

)

{

    uint8_t ch;

    char response[512];

    uint32_t index = 0;

    uint32_t start = HAL_GetTick();

    memset(response, 0, sizeof(response));

    while ((HAL_GetTick() - start) < timeout)

    {

        if (HAL_UART_Receive(

                &huart3,

                &ch,

                1,

                100) == HAL_OK)

        {

            if (index < sizeof(response) - 1U)

            {

                response[index++] = (char)ch;

                response[index] = '\0';

            }

            if (strstr(response, expected) != NULL)

            {

                Debug_Print("\r\nMODEM -> STM32:\r\n");

                Debug_Print(response);

                return 1;

            }

            if (strstr(response, "+CME ERROR:") != NULL ||

                strstr(response, "\r\nERROR\r\n") != NULL)

            {

                Debug_Print("\r\nMODEM -> STM32:\r\n");

                Debug_Print(response);

                return 0;

            }

        }

    }

    Debug_Print("\r\nMODEM RESPONSE TIMEOUT.\r\n");

    return 0;

}

















/* -------------------------------------------------------------------------- */

/* Wait for the complete QFREAD CONNECT line                                 */

/*                                                                            */

/* EC200U responds to QFREAD with:                                           */

/*     CONNECT <read_length>\r\n                                             */

/* followed immediately by raw binary data.                                  */

/*                                                                            */

/* It is critical to consume the ENTIRE CONNECT line. If we stop as soon as   */

/* the word "CONNECT" is seen, the remaining " 1024\r\n" bytes become part   */

/* of the firmware image and the binary receive is shifted by 7 bytes.        */

/* -------------------------------------------------------------------------- */

static int Modem_ReadExactBinary(

    uint8_t *buffer,

    uint32_t length,

    uint32_t timeoutPerByte

)

{

    uint32_t received = 0;

    if (buffer == NULL || length == 0U)

    {

        return 0;

    }

    /*

     * Read one byte at a time.

     *

     * This is intentionally conservative for the EC200U UART data mode.

     * It avoids treating binary data as a string and gives us an exact

     * byte count.

     */

    while (received < length)

    {

        if (HAL_UART_Receive(

                &huart3,

                &buffer[received],

                1,

                timeoutPerByte) != HAL_OK)

        {

            Debug_Print("ERROR: Binary data receive timeout.\r\n");

            return 0;

        }

        received++;

    }

    return 1;

}

















/* -------------------------------------------------------------------------- */

/* Check for firmware update file                                             */

/* -------------------------------------------------------------------------- */

/* Check whether firmware.bin exists                                   */

/* -------------------------------------------------------------------------- */

static int Modem_CheckForUpdate(void)

{

    char response[1024];

    Debug_Print(

        "Checking EC200U for update file...\r\n"

    );

    if (!Modem_SendCommand(

            "AT+QFLST=\"UFS:firmware.bin\"\r",

            response,

            sizeof(response),

            5000))

    {

        Debug_Print(

            "ERROR: QFLST command failed.\r\n"

        );

        return 0;

    }

    if (strstr(

            response,

            "UFS:firmware.bin") != NULL)

    {

        return 1;

    }

    return 0;

}

















/* -------------------------------------------------------------------------- */

/* Get firmware file size                                                     */

/* -------------------------------------------------------------------------- */

static int Modem_GetFirmwareSize(

    uint32_t *fileSize

)

{

    char response[1024];

    if (fileSize == NULL)

    {

        return 0;

    }

    *fileSize = 0;

    if (!Modem_SendCommand(

            "AT+QFLST=\"UFS:firmware.bin\"\r",

            response,

            sizeof(response),

            5000))

    {

        return 0;

    }

    /*

     * Expected format is similar to:

     *

     * "UFS:firmware.bin",12345

     *

     * Find the filename first, then parse the comma-separated size.

     */

    char *filename = strstr(

        response,

        "firmware.bin"

    );

    if (filename == NULL)

    {

        return 0;

    }

    char *comma = strchr(

        filename,

        ','

    );

    if (comma == NULL)

    {

        return 0;

    }

    *fileSize = strtoul(

        comma + 1,

        NULL,

        10

    );

    return (*fileSize > 0);

}

















/* -------------------------------------------------------------------------- */

/* Open firmware file                                                         */

/* -------------------------------------------------------------------------- */

static int Modem_OpenFirmware(

    uint32_t *fileHandle

)

{

    char response[512];

    if (fileHandle == NULL)

    {

        return 0;

    }

    *fileHandle = 0;

    if (!Modem_SendCommand(

            "AT+QFOPEN=\"UFS:firmware.bin\",2\r",

            response,

            sizeof(response),

            5000))

    {

        return 0;

    }

    /*

     * Expected:

     *

     * +QFOPEN: <handle>

     * OK

     */

    char *p = strstr(

        response,

        "+QFOPEN:"

    );

    if (p == NULL)

    {

        return 0;

    }

    p += strlen("+QFOPEN:");

    *fileHandle = strtoul(

        p,

        NULL,

        10

    );

    return 1;

}

















/* -------------------------------------------------------------------------- */

/* Read binary data from EC200U                                               */

/* -------------------------------------------------------------------------- */

static int Modem_ReadFile(

    uint32_t fileHandle,

    uint8_t *buffer,

    uint32_t length

)

{

    char command[64];

    if (buffer == NULL || length == 0U)

    {

        return 0;

    }

    snprintf(

        command,

        sizeof(command),

        "AT+QFREAD=%lu,%lu\r",

        (unsigned long)fileHandle,

        (unsigned long)length

    );

    /*

     * QFREAD is a special data-mode command.

     *

     * Do NOT use Modem_SendCommand(), because that routine waits for

     * a normal OK response and QFREAD enters raw binary data mode.

     */

    if (!Modem_SendRawCommand(command))

    {

        return 0;

    }

    /*

     * Wait for the complete CONNECT <length>\r\n header.

     */

    if (!Modem_WaitForQFReadHeader(length, 5000U))

    {

        return 0;

    }

    /*

     * Receive exactly 'length' binary bytes.

     * No debug output is allowed between CONNECT and this call.

     */

    __HAL_UART_CLEAR_OREFLAG(&huart3);

    if (!Modem_ReadExactBinary(

            buffer,

            length,

            1000U))

    {

        return 0;

    }

    /*

     * After exactly 'length' bytes the EC200U returns to command mode

     * and sends OK.

     */

    if (!Modem_WaitFor("OK", 5000U))

    {

        Debug_Print("ERROR: QFREAD final OK not received.\r\n");

        return 0;

    }

    return 1;

}

















/* -------------------------------------------------------------------------- */

/* Close firmware file                                                        */

/* -------------------------------------------------------------------------- */

/* Close firmware file                                                        */

/* -------------------------------------------------------------------------- */

static int Modem_CloseFile(

    uint32_t fileHandle

)

{

    char command[64];

    char response[512];

    snprintf(

        command,

        sizeof(command),

        "AT+QFCLOSE=%lu\r",

        (unsigned long)fileHandle

    );

    return Modem_SendCommand(

        command,

        response,

        sizeof(response),

        5000

    );

}

















/* -------------------------------------------------------------------------- */

/* Delete update file                                                         */

/* -------------------------------------------------------------------------- */

static int Modem_DeleteFirmware(void)

{

    char response[512];

    return Modem_SendCommand(

        "AT+QFDEL=\"UFS:firmware.bin\"\r",

        response,

        sizeof(response),

        5000

    );

}

















/* -------------------------------------------------------------------------- */

/* Erase application region                                                   */

/* -------------------------------------------------------------------------- */

static HAL_StatusTypeDef Erase_Application(void)

{

    FLASH_EraseInitTypeDef eraseInit;

    uint32_t pageError = 0;

    memset(&eraseInit, 0, sizeof(eraseInit));

    HAL_FLASH_Unlock();

    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;

    eraseInit.PageAddress = APPLICATION_ADDRESS;

    eraseInit.NbPages = APPLICATION_MAX_SIZE / FLASH_PAGE_SIZE;

    if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK)

    {

        Debug_Print("ERROR: Flash erase failed!\r\n");

        HAL_FLASH_Lock();

        return HAL_ERROR;

    }

    HAL_FLASH_Lock();

    return HAL_OK;

}

















/* -------------------------------------------------------------------------- */

/* Write buffer into STM32 Flash                                              */

/* -------------------------------------------------------------------------- */

static HAL_StatusTypeDef Flash_WriteBuffer(

    uint32_t address,

    uint8_t *data,

    uint32_t length

)

{

    HAL_StatusTypeDef status;

















    HAL_FLASH_Unlock();

















    for (uint32_t i = 0;

         i < length;

         i += 4)

    {

        uint32_t word =

            0xFFFFFFFFUL;

















        uint32_t remaining =

            length - i;

















        if (remaining >= 4)

        {

            memcpy(

                &word,

                &data[i],

                4

            );

        }

        else

        {

            memcpy(

                &word,

                &data[i],

                remaining

            );

        }

















        status =

            HAL_FLASH_Program(

                FLASH_TYPEPROGRAM_WORD,

                address + i,

                word

            );

















        if (status != HAL_OK)

        {

            HAL_FLASH_Lock();

            return status;

        }

    }

















    HAL_FLASH_Lock();

















    return HAL_OK;

}

















/* -------------------------------------------------------------------------- */

/* Verify application                                                         */

/* -------------------------------------------------------------------------- */

static int Verify_Application(

    uint32_t firmwareSize

)

{

    /*

     * For now, verify that every byte written to Flash

     * matches the data already received.

     *

     * Since the modem file is no longer being read here,

     * this function performs a basic Flash validity check

     * using the vector table.

     */

    uint32_t stackPointer =

        *(__IO uint32_t *)

        APPLICATION_ADDRESS;

















    uint32_t resetHandler =

        *(__IO uint32_t *)

        (APPLICATION_ADDRESS + 4U);

















    /*

     * Check stack pointer.

     */

    if ((stackPointer & 0x2FFE0000U)

        != 0x20000000U)

    {

        return 0;

    }

















    /*

     * Check Reset_Handler.

     *

     * Cortex-M handlers should have bit 0 set

     * because execution is Thumb mode.

     */

    if ((resetHandler & 1U) == 0)

    {

        return 0;

    }

















    /*

     * Check that the Reset_Handler lies within

     * the application region.

     */

    if (resetHandler <

        APPLICATION_ADDRESS)

    {

        return 0;

    }

















    if (resetHandler >=

        (APPLICATION_ADDRESS +

         firmwareSize))

    {

        return 0;

    }

















    return 1;

}

















/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */

/* Connect EC200U to cellular data                                            */

/* -------------------------------------------------------------------------- */

static int Modem_ConnectToInternet(void)

{

    char response[1024];

    Debug_Print("\r\n================================\r\n");

    Debug_Print("       MODEM INTERNET SETUP      \r\n");

    Debug_Print("================================\r\n");

    if (!Modem_SendCommand("AT+CPIN?\r", response, sizeof(response), 5000U) ||

        strstr(response, "+CPIN: READY") == NULL)

    {

        Debug_Print("ERROR: SIM is not ready.\r\n");

        return 0;

    }

    if (!Modem_SendCommand("AT+CSQ\r", response, sizeof(response), 5000U))

    {

        Debug_Print("ERROR: CSQ command failed.\r\n");

        return 0;

    }

    if (!Modem_SendCommand("AT+CEREG?\r", response, sizeof(response), 5000U))

    {

        Debug_Print("ERROR: CEREG command failed.\r\n");

        return 0;

    }

    if (strstr(response, "+CEREG: 0,1") == NULL &&

        strstr(response, "+CEREG: 0,5") == NULL &&

        strstr(response, "+CEREG: 1,1") == NULL &&

        strstr(response, "+CEREG: 1,5") == NULL)

    {

        Debug_Print("ERROR: EC200U is not registered on the network.\r\n");

        return 0;

    }

    if (!Modem_SendCommand("AT+CGATT?\r", response, sizeof(response), 5000U) ||

        strstr(response, "+CGATT: 1") == NULL)

    {

        Debug_Print("ERROR: EC200U is not packet-attached.\r\n");

        return 0;

    }

    /* Deactivate any PDP context left active from a previous boot. */

    (void)Modem_SendCommand("AT+QIDEACT=1\r",

                            response, sizeof(response), 30000U);

    if (!Modem_SendCommand("AT+QICSGP=1,3,\"airtelgprs.com\",\"\",\"\",0\r",

                           response, sizeof(response), 5000U))

    {

        Debug_Print("ERROR: QICSGP configuration failed.\r\n");

        return 0;

    }

    if (!Modem_SendCommand("AT+QICSGP=1\r",

                           response, sizeof(response), 5000U))

    {

        Debug_Print("ERROR: QICSGP query failed.\r\n");

        return 0;

    }

    if (!Modem_SendCommand("AT+QIACT=1\r",

                           response, sizeof(response), 60000U))

    {

        Debug_Print("ERROR: AT+QIACT=1 failed.\r\n");

        return 0;

    }
    Modem_SendCommand("AT+QIACT?\r",response, sizeof(response), 10000U);

    /*if (!Modem_SendCommand("AT+QIACT?\r",response, sizeof(response), 10000U) || strstr(response, "+QIACT: 1,1") == NULL)

    {

        Debug_Print("ERROR: PDP context 1 is not active.\r\n");

        return 0;

    }*/

    Debug_Print("EC200U is connected to the internet.\r\n");

    return 1;

}

/* -------------------------------------------------------------------------- */

/* Set EC200U HTTP URL                                                        */

/* -------------------------------------------------------------------------- */

static int Modem_SetHTTPURL(const char *url)

{

    char command[64];

    char response[512];

    uint8_t ch;

    uint32_t urlLength;

    uint32_t index;

    uint32_t startTick;

    int connected = 0;

    if (url == NULL)

        return 0;

    urlLength = (uint32_t)strlen(url);

    /* QHTTPURL requires the exact number of URL bytes. */

    snprintf(command,

             sizeof(command),

             "AT+QHTTPURL=%lu,60\r",

             (unsigned long)urlLength);

    Modem_Flush();

    Debug_Print("\r\nSTM32 -> MODEM:\r\n");

    Debug_Print(command);

    if (HAL_UART_Transmit(&huart3,

                          (uint8_t *)command,

                          (uint16_t)strlen(command),

                          HAL_MAX_DELAY) != HAL_OK)

    {

        Debug_Print("\r\nERROR: QHTTPURL transmit failed.\r\n");

        return 0;

    }

    /* Start listening immediately: EC200U may return CONNECT quickly. */

    /* QHTTPURL is special: wait for CONNECT before sending the URL. */

    memset(response, 0, sizeof(response));

    index = 0U;

    startTick = HAL_GetTick();

    while ((HAL_GetTick() - startTick) < 6000U)

    {

        if (HAL_UART_Receive(&huart3, &ch, 1, 100U) != HAL_OK)

            continue;

        if (index < sizeof(response) - 1U)

        {

            response[index++] = (char)ch;

            response[index] = '\0';

        }

        else

        {

            memmove(response,

                    response + 1U,

                    sizeof(response) - 2U);

            response[sizeof(response) - 2U] = (char)ch;

            response[sizeof(response) - 1U] = '\0';

        }

        if (strstr(response, "CONNECT") != NULL)

        {

            connected = 1;

            Debug_Print("\r\nMODEM -> STM32:\r\n");

            Debug_Print(response);

            break;

        }

        if (strstr(response, "+CME ERROR:") != NULL ||

            strstr(response, "\r\nERROR\r\n") != NULL)

        {

            Debug_Print("\r\nMODEM -> STM32:\r\n");

            Debug_Print(response);

            return 0;

        }

    }

    if (!connected)

    {

        Debug_Print("ERROR: Timeout waiting for QHTTPURL CONNECT.\r\n");

        return 0;

    }

    /* Send exactly urlLength bytes. No CR/LF is appended. */

    Debug_Print("\r\nSTM32 -> MODEM:\r\n");

    Debug_Print(url);

    Debug_Print("\r\n");

    if (HAL_UART_Transmit(&huart3,

                          (uint8_t *)url,

                          (uint16_t)urlLength,

                          HAL_MAX_DELAY) != HAL_OK)

    {

        Debug_Print("ERROR: S3 URL transmit failed.\r\n");

        return 0;

    }





    /* Start listening immediately for the final OK. */

    memset(response, 0, sizeof(response));

    index = 0U;

    startTick = HAL_GetTick();

    while ((HAL_GetTick() - startTick) < 10000U)

    {

        if (HAL_UART_Receive(&huart3, &ch, 1, 100U) != HAL_OK)

            continue;

        if (index < sizeof(response) - 1U)

        {

            response[index++] = (char)ch;

            response[index] = '\0';

        }

        else

        {

            memmove(response,

                    response + 1U,

                    sizeof(response) - 2U);

            response[sizeof(response) - 2U] = (char)ch;

            response[sizeof(response) - 1U] = '\0';

        }

        if (strstr(response, "\r\nOK\r\n") != NULL ||

            strstr(response, "\nOK\r\n") != NULL)

        {

            Debug_Print("\r\nMODEM -> STM32:\r\n");

            Debug_Print(response);

            return 1;

        }

        if (strstr(response, "+CME ERROR:") != NULL ||

            strstr(response, "\r\nERROR\r\n") != NULL)

        {

            Debug_Print("\r\nMODEM -> STM32:\r\n");

            Debug_Print(response);

            return 0;

        }

    }

    Debug_Print("ERROR: Timeout waiting for QHTTPURL final OK.\r\n");

    return 0;

}

/* -------------------------------------------------------------------------- */

/* Send QHTTPGET and wait for asynchronous +QHTTPGET URC                      */

/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* Authenticated HTTP GET using URL and token from OTA configuration         */
/* -------------------------------------------------------------------------- */

static int Modem_HttpGet(
    const char *url,
    const char *token,
    char *body,
    uint32_t bodySize,
    int *httpStatus,
    uint32_t *contentLength)
{
    uint8_t ch;

    char rxBuffer[512];
    char httpHeader[512];
    char command[64];

    uint32_t rxIndex = 0U;
    uint32_t startTime;
    uint32_t headerLength;

    unsigned long errCode = 0U;
    unsigned long httpCode = 0U;
    unsigned long dataLength = 0U;

    int headerResult;

    char *qhttp;

    int parsed;

    char statusMsg[96];

    /* URL parsing */
    const char *urlStart;
    const char *pathStart;
    const char *hostEnd;

    char host[192];
    char path[512];

    uint32_t hostLength;
    uint32_t pathLength;


    /* ================================================================
     * Validate parameters
     * ================================================================ */

    if (url == NULL ||
        token == NULL ||
        httpStatus == NULL ||
        contentLength == NULL)
    {
        Debug_Print(
            "ERROR: Invalid Modem_HttpGet parameters.\r\n"
        );

        return 0;
    }


    *httpStatus = 0;
    *contentLength = 0U;


    /* ================================================================
     * Body is optional
     *
     * body == NULL / bodySize == 0
     *
     * means the caller wants to handle the HTTP body separately.
     *
     * This is used for firmware download where the body will be
     * handled separately by Modem_HttpReadToFlash() using QHTTPREAD.
     * ================================================================ */

    if (body != NULL && bodySize > 0U)
    {
        body[0] = '\0';
    }


    /* ================================================================
     * Parse URL
     * ================================================================ */

    /*
     * Expected:
     *
     * https://70br5ujjbb.execute-api.ap-south-1.amazonaws.com/firmware
     */

    urlStart = url;


    if (strncmp(urlStart, "https://", 8U) == 0)
    {
        urlStart += 8U;
    }
    else if (strncmp(urlStart, "http://", 7U) == 0)
    {
        urlStart += 7U;
    }
    else
    {
        Debug_Print(
            "ERROR: Unsupported URL format.\r\n"
        );

        return 0;
    }


    /* ================================================================
     * Find beginning of path
     * ================================================================ */

    pathStart = strchr(urlStart, '/');


    if (pathStart == NULL)
    {
        /*
         * URL contains only hostname.
         * Use "/" as HTTP path.
         */

        hostEnd = urlStart + strlen(urlStart);

        hostLength = (uint32_t)(hostEnd - urlStart);

        if (hostLength == 0U ||
            hostLength >= sizeof(host))
        {
            Debug_Print(
                "ERROR: Invalid HTTP host.\r\n"
            );

            return 0;
        }

        memcpy(
            host,
            urlStart,
            hostLength
        );

        host[hostLength] = '\0';

        strcpy(
            path,
            "/"
        );
    }
    else
    {
        /*
         * Host is everything before '/'
         */

        hostEnd = pathStart;

        hostLength = (uint32_t)(hostEnd - urlStart);

        if (hostLength == 0U ||
            hostLength >= sizeof(host))
        {
            Debug_Print(
                "ERROR: Invalid HTTP host.\r\n"
            );

            return 0;
        }

        memcpy(
            host,
            urlStart,
            hostLength
        );

        host[hostLength] = '\0';


        /* Copy path */

        pathLength = (uint32_t)strlen(pathStart);

        if (pathLength == 0U ||
            pathLength >= sizeof(path))
        {
            Debug_Print(
                "ERROR: HTTP path too long.\r\n"
            );

            return 0;
        }

        memcpy(
            path,
            pathStart,
            pathLength
        );

        path[pathLength] = '\0';
    }


    /* ================================================================
     * Build HTTP request header
     * ================================================================ */

    /*
     * requestheader = 1
     *
     * The modem will first return:
     *
     *     CONNECT
     *
     * Then STM32 sends exactly headerLength bytes.
     */

    headerResult = snprintf(
        httpHeader,
        sizeof(httpHeader),

        "GET %s HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Authorization: %s\r\n"
        "Connection: close\r\n"
        "\r\n",

        path,
        host,
        token
    );


    if (headerResult < 0 ||
        (uint32_t)headerResult >= sizeof(httpHeader))
    {
        Debug_Print(
            "ERROR: HTTP request header too large.\r\n"
        );

        return 0;
    }


    /*
     * snprintf() returns the number of characters written,
     * excluding the terminating '\0'.
     *
     * This is exactly the number of bytes that must be
     * sent to the modem.
     */

    headerLength = (uint32_t)headerResult;


    /* ================================================================
     * Debug header length
     * ================================================================ */

    snprintf(
        statusMsg,
        sizeof(statusMsg),
        "HTTP header length = %lu bytes\r\n",
        (unsigned long)headerLength
    );

    Debug_Print(statusMsg);


    /* ================================================================
     * Configure URL BEFORE QHTTPGET
     * ================================================================ */

    Debug_Print(
        "Setting HTTP URL...\r\n"
    );


    if (!Modem_SetHTTPURL(url))
    {
        Debug_Print(
            "ERROR: Modem_SetHTTPURL failed.\r\n"
        );

        return 0;
    }


    Debug_Print(
        "HTTP URL configured successfully.\r\n"
    );


    /* ================================================================
     * Start QHTTPGET
     * ================================================================ */

    /*
     * requestheader = 1
     *
     * Syntax:
     *
     * AT+QHTTPGET=<response_time>,<data_length>,<input_time>
     *
     * data_length = exact number of HTTP header bytes.
     */

    snprintf(
        command,
        sizeof(command),
        "AT+QHTTPGET=120,%lu,30\r",
        (unsigned long)headerLength
    );


    Modem_Flush();


    Debug_Print(
        "\r\nSTM32 -> MODEM:\r\n"
    );


    /*
     * Do not print httpHeader because it contains
     * the Authorization token.
     */

    Debug_Print(command);


    if (HAL_UART_Transmit(
            &huart3,
            (uint8_t *)command,
            strlen(command),
            HAL_MAX_DELAY) != HAL_OK)
    {
        Debug_Print(
            "ERROR: Failed to transmit QHTTPGET command.\r\n"
        );

        return 0;
    }


    /* ================================================================
     * Wait for CONNECT
     * ================================================================ */

    memset(
        rxBuffer,
        0,
        sizeof(rxBuffer)
    );

    rxIndex = 0U;

    startTime = HAL_GetTick();


    /*
     * The modem can take much longer to establish
     * the HTTP/HTTPS connection.
     */

    while ((HAL_GetTick() - startTime) < 125000U)
    {
        if (HAL_UART_Receive(
                &huart3,
                &ch,
                1,
                100U) != HAL_OK)
        {
            continue;
        }


        if (rxIndex < sizeof(rxBuffer) - 1U)
        {
            rxBuffer[rxIndex++] = (char)ch;
            rxBuffer[rxIndex] = '\0';
        }
        else
        {
            /*
             * Keep the most recent data.
             */

            memmove(
                rxBuffer,
                rxBuffer + 1U,
                sizeof(rxBuffer) - 2U
            );

            rxBuffer[sizeof(rxBuffer) - 2U] = (char)ch;
            rxBuffer[sizeof(rxBuffer) - 1U] = '\0';
        }


        /* ------------------------------------------------------------
         * CONNECT received
         * ------------------------------------------------------------ */

        if (strstr(
                rxBuffer,
                "CONNECT") != NULL)
        {
            Debug_Print(
                "\r\nMODEM -> STM32:\r\n"
            );

            Debug_Print(rxBuffer);

            break;
        }


        /* ------------------------------------------------------------
         * Modem error before CONNECT
         * ------------------------------------------------------------ */

        if (strstr(
                rxBuffer,
                "+CME ERROR:") != NULL ||

            strstr(
                rxBuffer,
                "\r\nERROR\r\n") != NULL ||

            strstr(
                rxBuffer,
                "\nERROR\r\n") != NULL)
        {
            Debug_Print(
                "\r\nERROR: QHTTPGET failed before CONNECT.\r\n"
            );

            Debug_Print(
                "Raw modem response:\r\n"
            );

            Debug_Print(rxBuffer);

            Debug_Print("\r\n");

            return 0;
        }
    }


    /* ================================================================
     * CONNECT timeout
     * ================================================================ */

    if (strstr(
            rxBuffer,
            "CONNECT") == NULL)
    {
        Debug_Print(
            "\r\nERROR: Timeout waiting for QHTTPGET CONNECT.\r\n"
        );

        Debug_Print(
            "Raw modem response:\r\n"
        );

        Debug_Print(rxBuffer);

        Debug_Print("\r\n");

        return 0;
    }


    /* ================================================================
     * Send HTTP request header
     * ================================================================ */

    /*
     * Send EXACTLY headerLength bytes.
     *
     * Do NOT add another CR/LF here.
     */

    Debug_Print(
        "\r\nSTM32 -> MODEM:\r\n"
    );

    Debug_Print(
        "Sending HTTP request header...\r\n"
    );


    if (HAL_UART_Transmit(
            &huart3,
            (uint8_t *)httpHeader,
            (uint16_t)headerLength,
            HAL_MAX_DELAY) != HAL_OK)
    {
        Debug_Print(
            "ERROR: Failed to transmit HTTP request header.\r\n"
        );

        return 0;
    }


    /* ================================================================
     * Wait for modem to accept HTTP request header
     * ================================================================ */

    memset(
        rxBuffer,
        0,
        sizeof(rxBuffer)
    );

    rxIndex = 0U;

    startTime = HAL_GetTick();


    while ((HAL_GetTick() - startTime) < 10000U)
    {
        if (HAL_UART_Receive(
                &huart3,
                &ch,
                1,
                100U) != HAL_OK)
        {
            continue;
        }


        if (rxIndex < sizeof(rxBuffer) - 1U)
        {
            rxBuffer[rxIndex++] = (char)ch;
            rxBuffer[rxIndex] = '\0';
        }
        else
        {
            memmove(
                rxBuffer,
                rxBuffer + 1U,
                sizeof(rxBuffer) - 2U
            );

            rxBuffer[sizeof(rxBuffer) - 2U] = (char)ch;
            rxBuffer[sizeof(rxBuffer) - 1U] = '\0';
        }


        /* ------------------------------------------------------------
         * Header accepted
         * ------------------------------------------------------------ */

        if (strstr(
                rxBuffer,
                "\r\nOK\r\n") != NULL ||

            strstr(
                rxBuffer,
                "\nOK\r\n") != NULL)
        {
            Debug_Print(
                "\r\nMODEM accepted HTTP request header.\r\n"
            );

            break;
        }


        /* ------------------------------------------------------------
         * Header rejected
         * ------------------------------------------------------------ */

        if (strstr(
                rxBuffer,
                "+CME ERROR:") != NULL ||

            strstr(
                rxBuffer,
                "\r\nERROR\r\n") != NULL ||

            strstr(
                rxBuffer,
                "\nERROR\r\n") != NULL)
        {
            Debug_Print(
                "\r\nERROR: Modem rejected HTTP request header.\r\n"
            );

            Debug_Print(
                "Raw modem response:\r\n"
            );

            Debug_Print(rxBuffer);

            Debug_Print("\r\n");

            return 0;
        }
    }


    /* ================================================================
     * Header acceptance timeout
     * ================================================================ */

    if (strstr(
            rxBuffer,
            "OK") == NULL)
    {
        Debug_Print(
            "\r\nERROR: Timeout waiting for HTTP header acceptance.\r\n"
        );

        Debug_Print(
            "Raw modem response:\r\n"
        );

        Debug_Print(rxBuffer);

        Debug_Print("\r\n");

        return 0;
    }


    /* ================================================================
     * Wait for +QHTTPGET result
     * ================================================================ */

    memset(
        rxBuffer,
        0,
        sizeof(rxBuffer)
    );

    rxIndex = 0U;

    startTime = HAL_GetTick();


    while ((HAL_GetTick() - startTime) < 130000U)
    {
        if (HAL_UART_Receive(
                &huart3,
                &ch,
                1,
                100U) != HAL_OK)
        {
            continue;
        }


        /* ------------------------------------------------------------
         * Store received character
         * ------------------------------------------------------------ */

        if (rxIndex < sizeof(rxBuffer) - 1U)
        {
            rxBuffer[rxIndex++] = (char)ch;
            rxBuffer[rxIndex] = '\0';
        }
        else
        {
            /*
             * Keep the tail of the buffer.
             */

            memmove(
                rxBuffer,
                rxBuffer + 128U,
                sizeof(rxBuffer) - 129U
            );

            rxIndex = sizeof(rxBuffer) - 129U;

            rxBuffer[rxIndex++] = (char)ch;
            rxBuffer[rxIndex] = '\0';
        }


        /* ------------------------------------------------------------
         * Search for complete +QHTTPGET result
         * ------------------------------------------------------------ */

        qhttp = strstr(
            rxBuffer,
            "+QHTTPGET:"
        );


        if (qhttp != NULL)
        {
            char *lineEnd;


            /*
             * IMPORTANT:
             *
             * Do not parse immediately after finding
             * "+QHTTPGET:".
             *
             * UART data arrives one byte at a time.
             *
             * We must wait until the complete line has
             * arrived.
             */

            lineEnd = strstr(
                qhttp,
                "\r\n"
            );


            if (lineEnd != NULL)
            {
                /*
                 * Example:
                 *
                 * +QHTTPGET: 0,200,52
                 *
                 * or:
                 *
                 * +QHTTPGET: 0,200,11716
                 */

                errCode = 0U;
                httpCode = 0U;
                dataLength = 0U;


                parsed = sscanf(
                    qhttp,
                    "+QHTTPGET: %lu,%lu,%lu",
                    &errCode,
                    &httpCode,
                    &dataLength
                );


                /*
                 * content_length is optional.
                 *
                 * Therefore both 2 and 3 parsed values
                 * are valid.
                 */

                if (parsed == 2 || parsed == 3)
                {
                    *httpStatus = (int)httpCode;

                    *contentLength =
                        (uint32_t)dataLength;


                    Debug_Print(
                        "\r\nMODEM -> STM32:\r\n"
                    );

                    Debug_Print(qhttp);

                    Debug_Print("\r\n");


                    /* ------------------------------------------------
                     * Modem-level error
                     * ------------------------------------------------ */

                    if (errCode != 0U)
                    {
                        snprintf(
                            statusMsg,
                            sizeof(statusMsg),
                            "ERROR: EC200U HTTP GET failed. Error code = %lu\r\n",
                            errCode
                        );

                        Debug_Print(statusMsg);

                        return 0;
                    }


                    /* ------------------------------------------------
                     * HTTP 200
                     * ------------------------------------------------ */

                    if (httpCode == 200U)
                    {
                        snprintf(
                            statusMsg,
                            sizeof(statusMsg),
                            "HTTP 200 OK. Response size: %lu bytes\r\n",
                            dataLength
                        );

                        Debug_Print(statusMsg);


                        /*
                         * =================================================
                         * IMPORTANT OTA LOGIC
                         * =================================================
                         *
                         * If body == NULL or bodySize == 0:
                         *
                         * The caller does NOT want the HTTP response
                         * copied into STM32 RAM.
                         *
                         * This is the firmware download case.
                         *
                         * Modem_DownloadFirmwareFromS3() will subsequently
                         * execute:
                         *
                         * AT+QHTTPREADFILE="UFS:firmware.bin",120
                         *
                         * Therefore DO NOT call Modem_HttpRead().
                         */

                        if (body == NULL || bodySize == 0U)
                        {
                            Debug_Print(
                                "HTTP body will be handled using QHTTPREADFILE.\r\n"
                            );

                            return 1;
                        }


                        /*
                         * =================================================
                         * NORMAL HTTP RESPONSE
                         * =================================================
                         *
                         * This is used for small responses such as:
                         *
                         * {"status":"success","filename":"random_name.bin"}
                         *
                         * Read the response into STM32 RAM.
                         */

                        if (!Modem_HttpRead(
                                body,
                                bodySize,
                                (uint32_t)dataLength))
                        {
                            Debug_Print(
                                "ERROR: Failed to read HTTP response body.\r\n"
                            );

                            return 0;
                        }


                        return 1;
                    }


                    /* ------------------------------------------------
                     * Authorization failure
                     * ------------------------------------------------ */

                    if (httpCode == 401U ||
                        httpCode == 403U)
                    {
                        Debug_Print(
                            "ERROR: API Gateway authorization failed.\r\n"
                        );

                        return 0;
                    }


                    /* ------------------------------------------------
                     * Firmware not found
                     * ------------------------------------------------ */

                    if (httpCode == 404U)
                    {
                        Debug_Print(
                            "ERROR: Firmware endpoint returned 404.\r\n"
                        );

                        return 0;
                    }


                    /* ------------------------------------------------
                     * Other HTTP status
                     * ------------------------------------------------ */

                    snprintf(
                        statusMsg,
                        sizeof(statusMsg),
                        "HTTP server returned status %lu\r\n",
                        httpCode
                    );

                    Debug_Print(statusMsg);

                    return 0;
                }
            }
        }


        /* ------------------------------------------------------------
         * Modem error while waiting for QHTTPGET result
         * ------------------------------------------------------------ */

        if (strstr(
                rxBuffer,
                "+CME ERROR:") != NULL ||

            strstr(
                rxBuffer,
                "\r\nERROR\r\n") != NULL ||

            strstr(
                rxBuffer,
                "\nERROR\r\n") != NULL)
        {
            Debug_Print(
                "\r\nMODEM ERROR:\r\n"
            );

            Debug_Print(rxBuffer);

            Debug_Print("\r\n");

            return 0;
        }
    }


    /* ================================================================
     * Final timeout
     * ================================================================ */

    Debug_Print(
        "\r\nERROR: Timeout waiting for +QHTTPGET result.\r\n"
    );

    Debug_Print(
        "Received data:\r\n"
    );

    Debug_Print(rxBuffer);

    Debug_Print("\r\n");


    return 0;
}

static int Modem_HttpRead(
    char *body,
    uint32_t bodySize,
    uint32_t expectedLength)
{
    char command[128];
    char headerBuffer[128];
    char finalBuffer[128];

    uint8_t ch;

    uint32_t headerIndex = 0U;
    uint32_t finalIndex = 0U;
    uint32_t received = 0U;
    uint32_t startTime;


    /* ============================================================
     * Validate parameters
     * ============================================================ */

    if (body == NULL || bodySize == 0U)
    {
        Debug_Print(
            "ERROR: Invalid HTTP body buffer.\r\n"
        );

        return 0;
    }

    if (expectedLength == 0U)
    {
        Debug_Print(
            "ERROR: Expected HTTP body length is zero.\r\n"
        );

        return 0;
    }

    if (expectedLength >= bodySize)
    {
        snprintf(
            command,
            sizeof(command),
            "ERROR: Body buffer too small. Expected %lu bytes, buffer = %lu bytes.\r\n",
            (unsigned long)expectedLength,
            (unsigned long)bodySize
        );

        Debug_Print(command);

        return 0;
    }


    /* ============================================================
     * Clear buffers
     * ============================================================ */

    memset(body, 0, bodySize);

    memset(
        headerBuffer,
        0,
        sizeof(headerBuffer)
    );

    memset(
        finalBuffer,
        0,
        sizeof(finalBuffer)
    );


    /* ============================================================
     * Send AT+QHTTPREAD
     * ============================================================ */

    snprintf(
        command,
        sizeof(command),
        "AT+QHTTPREAD=80\r"
    );

    /*
     * Remove stale UART data before starting QHTTPREAD.
     */
    Modem_Flush();

    Debug_Print(
        "\r\nSTM32 -> MODEM:\r\n"
    );

    Debug_Print(command);


    if (HAL_UART_Transmit(
            &huart3,
            (uint8_t *)command,
            (uint16_t)strlen(command),
            HAL_MAX_DELAY) != HAL_OK)
    {
        Debug_Print(
            "ERROR: Failed to transmit QHTTPREAD.\r\n"
        );

        return 0;
    }


    /* ============================================================
     * Wait for CONNECT\r\n
     *
     * IMPORTANT:
     *
     * Once CONNECT is received, the modem immediately enters
     * transparent/data mode.
     *
     * DO NOT PRINT anything after CONNECT.
     * ============================================================ */

    startTime = HAL_GetTick();

    while ((HAL_GetTick() - startTime) < 10000U)
    {
        if (HAL_UART_Receive(
                &huart3,
                &ch,
                1,
                100U) != HAL_OK)
        {
            continue;
        }


        /* --------------------------------------------------------
         * Store received character in rolling buffer
         * -------------------------------------------------------- */

        if (headerIndex < sizeof(headerBuffer) - 1U)
        {
            headerBuffer[headerIndex++] = (char)ch;

            headerBuffer[headerIndex] = '\0';
        }
        else
        {
            memmove(
                headerBuffer,
                headerBuffer + 1U,
                sizeof(headerBuffer) - 2U
            );

            headerBuffer[sizeof(headerBuffer) - 2U] =
                (char)ch;

            headerBuffer[sizeof(headerBuffer) - 1U] =
                '\0';
        }


        /* --------------------------------------------------------
         * Check for CONNECT\r\n
         * -------------------------------------------------------- */

        if (strstr(
                headerBuffer,
                "CONNECT\r\n") != NULL)
        {
            /*
             * DO NOT PRINT HERE.
             *
             * The HTTP body may already be arriving.
             */

            break;
        }


        /* --------------------------------------------------------
         * Check for modem errors
         * -------------------------------------------------------- */

        if (strstr(
                headerBuffer,
                "+CME ERROR:") != NULL ||
            strstr(
                headerBuffer,
                "\r\nERROR\r\n") != NULL)
        {
            Debug_Print(
                "\r\nERROR: QHTTPREAD returned modem error.\r\n"
            );

            Debug_Print(headerBuffer);

            Debug_Print("\r\n");

            return 0;
        }
    }


    /* ============================================================
     * Check CONNECT timeout
     * ============================================================ */

    if (strstr(
            headerBuffer,
            "CONNECT\r\n") == NULL)
    {
        Debug_Print(
            "\r\nERROR: Timeout waiting for QHTTPREAD CONNECT.\r\n"
        );

        Debug_Print(
            "Received modem data:\r\n"
        );

        Debug_Print(headerBuffer);

        Debug_Print("\r\n");

        return 0;
    }


    /* ============================================================
     * HTTP BODY DATA MODE
     *
     * We now expect exactly expectedLength bytes.
     *
     * Example:
     *
     * +QHTTPGET: 0,200,52
     *
     * Therefore:
     *
     * expectedLength = 52
     *
     * ============================================================ */

    __HAL_UART_CLEAR_OREFLAG(&huart3);

    received = 0U;

    startTime = HAL_GetTick();


    while (received < expectedLength)
    {
        if (HAL_UART_Receive(
                &huart3,
                (uint8_t *)&body[received],
                1,
                1000U) != HAL_OK)
        {
            snprintf(
                command,
                sizeof(command),
                "\r\nERROR: HTTP body receive timeout. Expected=%lu, Received=%lu\r\n",
                (unsigned long)expectedLength,
                (unsigned long)received
            );

            Debug_Print(command);

            return 0;
        }

        received++;

        /*
         * Safety timeout for the complete body.
         */
        if ((HAL_GetTick() - startTime) > 10000U)
        {
            snprintf(
                command,
                sizeof(command),
                "\r\nERROR: HTTP body receive timeout. Expected=%lu, Received=%lu\r\n",
                (unsigned long)expectedLength,
                (unsigned long)received
            );

            Debug_Print(command);

            return 0;
        }
    }


    /* ============================================================
     * Null terminate body
     * ============================================================ */

    body[received] = '\0';


    /* ============================================================
     * Body successfully received
     *
     * NOW it is safe to print.
     * ============================================================ */

    Debug_Print(
        "\r\nHTTP response body received:\r\n"
    );

    Debug_Print(body);

    Debug_Print("\r\n");


    /* ============================================================
     * Wait for QHTTPREAD completion
     *
     * Expected after body:
     *
     * OK
     * +QHTTPREAD: 0
     *
     * We specifically wait for:
     *
     * +QHTTPREAD: 0
     *
     * rather than relying on Modem_WaitFor("OK").
     * ============================================================ */

    memset(
        finalBuffer,
        0,
        sizeof(finalBuffer)
    );

    finalIndex = 0U;

    startTime = HAL_GetTick();


    while ((HAL_GetTick() - startTime) < 5000U)
    {
        if (HAL_UART_Receive(
                &huart3,
                &ch,
                1,
                100U) != HAL_OK)
        {
            continue;
        }


        /* --------------------------------------------------------
         * Store response in rolling buffer
         * -------------------------------------------------------- */

        if (finalIndex < sizeof(finalBuffer) - 1U)
        {
            finalBuffer[finalIndex++] = (char)ch;

            finalBuffer[finalIndex] = '\0';
        }
        else
        {
            memmove(
                finalBuffer,
                finalBuffer + 1U,
                sizeof(finalBuffer) - 2U
            );

            finalBuffer[sizeof(finalBuffer) - 2U] =
                (char)ch;

            finalBuffer[sizeof(finalBuffer) - 1U] =
                '\0';
        }


        /* --------------------------------------------------------
         * QHTTPREAD completed
         * -------------------------------------------------------- */

        if (strstr(
                finalBuffer,
                "+QHTTPREAD: 0") != NULL)
        {
            Debug_Print(
                "QHTTPREAD completed successfully.\r\n"
            );

            return 1;
        }


        /*
         * Some modem firmware may provide OK before the
         * +QHTTPREAD URC.
         *
         * We don't return immediately on OK because we want
         * to consume the complete QHTTPREAD response.
         */
    }


    /* ============================================================
     * QHTTPREAD completion timeout
     * ============================================================ */

    Debug_Print(
        "ERROR: QHTTPREAD completion response not received.\r\n"
    );

    Debug_Print(
        "Received after HTTP body:\r\n"
    );

    Debug_Print(finalBuffer);

    Debug_Print("\r\n");


    return 0;
}
static int Modem_DownloadFirmwareFromS3(uint32_t *firmwareSize)
{
    const OTA_Config_t *config = OTA_GetConfig();

    char response[1024];
    int httpStatus = 0;
    uint32_t contentLength = 0U;

    if (firmwareSize == NULL)
    {
        return 0;
    }

    *firmwareSize = 0U;

    Debug_Print("\r\n================================\r\n");
    Debug_Print("         CLOUD OTA CHECK        \r\n");
    Debug_Print("================================\r\n");

    /* Configure HTTP/HTTPS. */
    if (!Modem_SendCommand("AT+QHTTPCFG=\"contextid\",1\r",
                           response, sizeof(response), 5000U))
        return 0;

    if (!Modem_SendCommand("AT+QHTTPCFG=\"sslctxid\",1\r",
                           response, sizeof(response), 5000U))
        return 0;

    /* HTTPS transport configuration. */
    if (!Modem_SendCommand("AT+QSSLCFG=\"seclevel\",1,0\r",
                           response, sizeof(response), 5000U))
        return 0;

    if (!Modem_SendCommand("AT+QSSLCFG=\"sslversion\",1,4\r",
                           response, sizeof(response), 5000U))
        return 0;

    if (!Modem_SendCommand("AT+QSSLCFG=\"ciphersuite\",1,0xFFFF\r",
                           response, sizeof(response), 5000U))
        return 0;

    if (!Modem_SendCommand("AT+QSSLCFG=\"sni\",1,1\r",
                           response, sizeof(response), 5000U))
        return 0;

    if (!Modem_SendCommand("AT+QHTTPCFG=\"responseheader\",0\r",
                           response, sizeof(response), 5000U))
        return 0;

    if (!Modem_SendCommand("AT+QHTTPCFG=\"requestheader\",1\r",
                           response, sizeof(response), 5000U))
        return 0;

    Debug_Print("Checking API Gateway for firmware update...\r\n");

    if (!Modem_SetHTTPURL(config->firmware_download_url))
    {
        Debug_Print("ERROR: Could not set OTA API URL.\r\n");
        return 0;
    }

    /*
     * QHTTPGET downloads the HTTP response into the modem's HTTP receive
     * buffer, but we do NOT call QHTTPREADFILE afterwards.
     *
     * Modem_HttpGet() returns the HTTP status and exact body length.
     */
    if (!Modem_HttpGet(config->firmware_download_url,
                       config->ota_auth_token,
                       NULL,
                       0U,
                       &httpStatus,
                       &contentLength))
    {
        if (httpStatus == 404)
        {
            Debug_Print("Firmware update not found on the OTA API.\r\n");
        }
        else
        {
            Debug_Print("ERROR: OTA API HTTP GET failed.\r\n");
        }

        return 0;
    }

    if (httpStatus == 404)
    {
        Debug_Print("Firmware update not found on the OTA API.\r\n");
        return 0;
    }

    if (httpStatus != 200)
    {
        Debug_Print("OTA API returned a non-200 HTTP status.\r\n");
        return 0;
    }

    if (contentLength == 0U ||
        contentLength > APPLICATION_MAX_SIZE)
    {
        Debug_Print("ERROR: Invalid firmware size from S3.\r\n");
        return 0;
    }

    {
        char message[128];

        snprintf(message,
                 sizeof(message),
                 "Firmware size received: %lu bytes\r\n",
                 (unsigned long)contentLength);

        Debug_Print(message);
    }

    /*
     * IMPORTANT:
     *
     * No UFS file is created here.
     *
     * Erase the application immediately before entering HTTP body mode.
     */
    Debug_Print("Erasing application region...\r\n");

    if (Erase_Application() != HAL_OK)
    {
        Debug_Print("ERROR: Application erase failed.\r\n");
        return 0;
    }

    /* Non-zero output now means application Flash has been erased. */
    *firmwareSize = contentLength;

    Debug_Print("Application region erased.\r\n");

    /*
     * Read the HTTP body directly from EC200U and write each small chunk
     * directly into STM32 Flash.
     */
    Debug_Print("Starting direct modem-to-STM32 firmware streaming...\r\n");

    if (!Modem_HttpReadToFlash(contentLength))
    {
        Debug_Print("ERROR: Firmware streaming failed.\r\n");
        return 0;
    }

    *firmwareSize = contentLength;

    Debug_Print("Firmware streamed directly to STM32 Flash.\r\n");

    return 1;
}

/* -------------------------------------------------------------------------- */
/* Read HTTP response body directly into STM32 Flash                          */
/*                                                                            */
/* IMPORTANT:                                                                */
/*   - No UFS file is created.                                                */
/*   - Only MODEM_READ_CHUNK_SIZE bytes are held in RAM at a time.            */
/*   - The HTTP body is received after QHTTPREAD CONNECT.                     */
/* -------------------------------------------------------------------------- */

static int Modem_HttpReadToFlash(uint32_t expectedLength)
{
    char command[64];
    char headerBuffer[96];
    char finalBuffer[128];
    uint8_t buffer[MODEM_READ_CHUNK_SIZE];
    uint8_t ch;
    uint32_t headerIndex = 0U;
    uint32_t finalIndex = 0U;
    uint32_t received = 0U;
    uint32_t flashAddress = APPLICATION_ADDRESS;
    uint32_t startTime;

    if (expectedLength == 0U || expectedLength > APPLICATION_MAX_SIZE)
    {
        Debug_Print("ERROR: Invalid HTTP firmware length.\r\n");
        return 0;
    }

    memset(headerBuffer, 0, sizeof(headerBuffer));
    memset(finalBuffer, 0, sizeof(finalBuffer));

    /* Start DMA before QHTTPREAD so the first firmware bytes cannot be lost. */
    if (!OTA_StreamDMA_Start())
        return 0;

    snprintf(command, sizeof(command), "AT+QHTTPREAD=80\r");
    Debug_Print("Starting QHTTPREAD with USART3 circular DMA...\r\n");

    if (HAL_UART_Transmit(&huart3, (uint8_t *)command,
                          (uint16_t)strlen(command), HAL_MAX_DELAY) != HAL_OK)
    {
        Debug_Print("ERROR: Failed to send QHTTPREAD.\r\n");
        OTA_StreamDMA_Stop();
        return 0;
    }

    startTime = HAL_GetTick();
    while ((HAL_GetTick() - startTime) < 15000U)
    {
        if (!OTA_StreamDMA_ReadByte(&ch, 100U))
            continue;

        if (headerIndex < sizeof(headerBuffer) - 1U)
        {
            headerBuffer[headerIndex++] = (char)ch;
            headerBuffer[headerIndex] = '\0';
        }
        else
        {
            memmove(headerBuffer, headerBuffer + 1U, sizeof(headerBuffer) - 2U);
            headerBuffer[sizeof(headerBuffer) - 2U] = (char)ch;
            headerBuffer[sizeof(headerBuffer) - 1U] = '\0';
        }

        if (strstr(headerBuffer, "CONNECT\r\n") != NULL)
            break;

        if (strstr(headerBuffer, "+CME ERROR:") != NULL ||
            strstr(headerBuffer, "\r\nERROR\r\n") != NULL)
        {
            Debug_Print("ERROR: QHTTPREAD returned modem error.\r\n");
            OTA_StreamDMA_Stop();
            return 0;
        }
    }

    if (strstr(headerBuffer, "CONNECT\r\n") == NULL)
    {
        Debug_Print("ERROR: Timeout waiting for QHTTPREAD CONNECT.\r\n");
        OTA_StreamDMA_Stop();
        return 0;
    }

    Debug_Print("QHTTPREAD CONNECT received. Streaming firmware...\r\n");

    while (received < expectedLength)
    {
        uint32_t chunkSize = expectedLength - received;
        if (chunkSize > MODEM_READ_CHUNK_SIZE)
            chunkSize = MODEM_READ_CHUNK_SIZE;

        if (!OTA_StreamDMA_Read(buffer, chunkSize, 10000U))
        {
            char message[128];
            snprintf(message, sizeof(message),
                     "ERROR: Firmware receive timeout. Received=%lu/%lu bytes.\r\n",
                     (unsigned long)received, (unsigned long)expectedLength);
            Debug_Print(message);
            OTA_StreamDMA_Stop();
            return 0;
        }

        if (Flash_WriteBuffer(flashAddress, buffer, chunkSize) != HAL_OK)
        {
            Debug_Print("ERROR: Flash write failed during streaming.\r\n");
            OTA_StreamDMA_Stop();
            return 0;
        }

        received += chunkSize;
        flashAddress += chunkSize;

        if (received == expectedLength)
        {
            char message[128];
            snprintf(message, sizeof(message),
                     "Firmware progress: 100%% (%lu/%lu bytes)\r\n",
                     (unsigned long)received, (unsigned long)expectedLength);
            Debug_Print(message);
        }
    }

    /*
     * The modem sends the QHTTPREAD result AFTER the HTTP body.
     * Because RX is running through circular DMA, the final response may
     * already be sitting in the DMA buffer when the last firmware byte is
     * consumed.  Drain the DMA buffer and look specifically for:
     *
     *     +QHTTPREAD: 0
     *
     * Do not treat any earlier '+QHTTPREAD:' text as the result until the
     * complete status value has been received.
     */
    memset(finalBuffer, 0, sizeof(finalBuffer));
    finalIndex = 0U;
    startTime = HAL_GetTick();

    while ((HAL_GetTick() - startTime) < 10000U)
    {
        if (!OTA_StreamDMA_ReadByte(&ch, 100U))
            continue;

        if (finalIndex < sizeof(finalBuffer) - 1U)
        {
            finalBuffer[finalIndex++] = (char)ch;
            finalBuffer[finalIndex] = '\0';
        }
        else
        {
            memmove(finalBuffer, finalBuffer + 1U, sizeof(finalBuffer) - 2U);
            finalBuffer[sizeof(finalBuffer) - 2U] = (char)ch;
            finalBuffer[sizeof(finalBuffer) - 1U] = '\0';
        }

        /* Success status from EC200U. */
        if (strstr(finalBuffer, "+QHTTPREAD: 0") != NULL)
        {
            OTA_StreamDMA_Stop();
            Debug_Print("QHTTPREAD completed successfully.\r\n");
            return 1;
        }

        /*
         * Only report an error after the complete QHTTPREAD status line is
         * present.  This avoids treating a partial '+QHTTPREAD:' string as
         * an error.
         */
        {
            char *result = strstr(finalBuffer, "+QHTTPREAD:");
            if (result != NULL)
            {
                char *lineEnd = strstr(result, "\r\n");
                if (lineEnd != NULL)
                {
                    char debugLine[96];
                    size_t lineLength = (size_t)(lineEnd - result);
                    if (lineLength >= sizeof(debugLine))
                        lineLength = sizeof(debugLine) - 1U;
                    memcpy(debugLine, result, lineLength);
                    debugLine[lineLength] = '\0';

                    Debug_Print("QHTTPREAD result: ");
                    Debug_Print(debugLine);
                    Debug_Print("\r\n");

                    if (strstr(result, "+QHTTPREAD: 0") == NULL)
                    {
                        Debug_Print("ERROR: QHTTPREAD reported an error.\r\n");
                        OTA_StreamDMA_Stop();
                        return 0;
                    }
                }
            }
        }
    }

    Debug_Print("ERROR: QHTTPREAD completion response not received.\r\n");
    Debug_Print("Final modem response buffer: ");
    Debug_Print(finalBuffer);
    Debug_Print("\r\n");
    OTA_StreamDMA_Stop();
    return 0;
}

/* -------------------------------------------------------------------------- */

/* Report successful firmware update                                          */

/*                                                                            */

/* The EC200U sends a small JSON POST to an HTTPS API endpoint. The backend   */

/* should write/update the status object in the same S3 bucket.               */

/* -------------------------------------------------------------------------- */

static int Modem_HttpPostNoBody(
    const char *url,
    const char *token,
    int *httpStatus
)
{
    /*const OTA_Config_t *config =(const OTA_Config_t *)OTA_CONFIG_ADDRESS;*/

    char response[512];

    char httpHeader[512];
    char command[64];

    const char *urlStart;
    const char *hostStart;
    const char *pathStart;

    char host[256];
    char path[256];

    uint32_t hostLength;
    uint32_t pathLength;
    uint32_t headerLength;

    int headerResult;


    /* ---------------------------------------------------------------------- */
    /* Validate parameters                                                    */
    /* ---------------------------------------------------------------------- */

    if (url == NULL ||
        token == NULL ||
        httpStatus == NULL)
    {
        return 0;
    }

    *httpStatus = 0;


    /* ---------------------------------------------------------------------- */
    /* Parse URL                                                              */
    /*                                                                        */
    /* Example:                                                               */
    /* https://70br5ujjbb.execute-api.ap-south-1.amazonaws.com/firmware-delete */
    /*                                                                        */
    /* host = 70br5ujjbb.execute-api.ap-south-1.amazonaws.com                */
    /* path = /firmware-delete                                                */
    /* ---------------------------------------------------------------------- */

    urlStart = strstr(url, "://");

    if (urlStart == NULL)
    {
        Debug_Print(
            "ERROR: Invalid URL format.\r\n"
        );

        return 0;
    }

    hostStart = urlStart + 3;

    pathStart = strchr(hostStart, '/');

    if (pathStart == NULL)
    {
        Debug_Print(
            "ERROR: URL path not found.\r\n"
        );

        return 0;
    }


    /* ---------------------------------------------------------------------- */
    /* Copy host                                                              */
    /* ---------------------------------------------------------------------- */

    hostLength =
        (uint32_t)(pathStart - hostStart);

    if (hostLength == 0U ||
        hostLength >= sizeof(host))
    {
        Debug_Print(
            "ERROR: URL host is too long.\r\n"
        );

        return 0;
    }

    memcpy(
        host,
        hostStart,
        hostLength
    );

    host[hostLength] = '\0';


    /* ---------------------------------------------------------------------- */
    /* Copy path                                                              */
    /* ---------------------------------------------------------------------- */

    pathLength =
        (uint32_t)strlen(pathStart);

    if (pathLength == 0U ||
        pathLength >= sizeof(path))
    {
        Debug_Print(
            "ERROR: URL path is too long.\r\n"
        );

        return 0;
    }

    memcpy(
        path,
        pathStart,
        pathLength
    );

    path[pathLength] = '\0';


    /* ---------------------------------------------------------------------- */
    /* Configure HTTP context                                                 */
    /* ---------------------------------------------------------------------- */

    if (!Modem_SendCommand(
            "AT+QHTTPCFG=\"contextid\",1\r",
            response,
            sizeof(response),
            5000U))
    {
        Debug_Print(
            "ERROR: Could not configure HTTP context.\r\n"
        );

        return 0;
    }


    /* ---------------------------------------------------------------------- */
    /* Configure SSL context                                                  */
    /* ---------------------------------------------------------------------- */

    if (!Modem_SendCommand(
            "AT+QHTTPCFG=\"sslctxid\",1\r",
            response,
            sizeof(response),
            5000U))
    {
        Debug_Print(
            "ERROR: Could not configure HTTP SSL context.\r\n"
        );

        return 0;
    }


    /* ---------------------------------------------------------------------- */
    /* SSL configuration                                                      */
    /* ---------------------------------------------------------------------- */

    if (!Modem_SendCommand(
            "AT+QSSLCFG=\"seclevel\",1,0\r",
            response,
            sizeof(response),
            5000U))
    {
        Debug_Print(
            "ERROR: QSSLCFG seclevel failed.\r\n"
        );

        return 0;
    }


    if (!Modem_SendCommand(
            "AT+QSSLCFG=\"sslversion\",1,4\r",
            response,
            sizeof(response),
            5000U))
    {
        Debug_Print(
            "ERROR: QSSLCFG sslversion failed.\r\n"
        );

        return 0;
    }


    if (!Modem_SendCommand(
            "AT+QSSLCFG=\"ciphersuite\",1,0xFFFF\r",
            response,
            sizeof(response),
            5000U))
    {
        Debug_Print(
            "ERROR: QSSLCFG ciphersuite failed.\r\n"
        );

        return 0;
    }


    if (!Modem_SendCommand(
            "AT+QSSLCFG=\"sni\",1,1\r",
            response,
            sizeof(response),
            5000U))
    {
        Debug_Print(
            "ERROR: QSSLCFG SNI failed.\r\n"
        );

        return 0;
    }


    /* ---------------------------------------------------------------------- */
    /* Disable response headers                                               */
    /* ---------------------------------------------------------------------- */

    if (!Modem_SendCommand(
            "AT+QHTTPCFG=\"responseheader\",0\r",
            response,
            sizeof(response),
            5000U))
    {
        Debug_Print(
            "ERROR: Could not configure response header mode.\r\n"
        );

        return 0;
    }


    /* ---------------------------------------------------------------------- */
    /* IMPORTANT: Enable custom HTTP request headers                          */
    /* ---------------------------------------------------------------------- */

    if (!Modem_SendCommand(
            "AT+QHTTPCFG=\"requestheader\",1\r",
            response,
            sizeof(response),
            5000U))
    {
        Debug_Print(
            "ERROR: Could not enable request header mode.\r\n"
        );

        return 0;
    }


    /* ---------------------------------------------------------------------- */
    /* Set URL                                                                */
    /* ---------------------------------------------------------------------- */

    if (!Modem_SetHTTPURL(url))
    {
        Debug_Print(
            "ERROR: Could not set HTTP POST URL.\r\n"
        );

        return 0;
    }


    /* ---------------------------------------------------------------------- */
    /* Build HTTP POST request header                                         */
    /*                                                                        */
    /* There is NO HTTP body.                                                 */
    /*                                                                        */
    /* Content-Length is therefore 0.                                         */
    /* ---------------------------------------------------------------------- */

    headerResult = snprintf(
        httpHeader,
        sizeof(httpHeader),

        "POST %s HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Authorization: %s\r\n"
        "Content-Length: 0\r\n"
        "Connection: close\r\n"
        "\r\n",

        path,
        host,
        token
    );


    if (headerResult < 0 ||
        (uint32_t)headerResult >= sizeof(httpHeader))
    {
        Debug_Print(
            "ERROR: HTTP POST header is too large.\r\n"
        );

        return 0;
    }


    headerLength =
        (uint32_t)headerResult;


    /* ---------------------------------------------------------------------- */
    /* Start QHTTPPOST                                                        */
    /*                                                                        */
    /* Because requestheader=1, the data length is the HTTP header length.    */
    /* There is no body.                                                      */
    /* ---------------------------------------------------------------------- */

    snprintf(
        command,
        sizeof(command),

        "AT+QHTTPPOST=%lu,30,120\r",

        (unsigned long)headerLength
    );


    Debug_Print(
        "Starting HTTP POST without body...\r\n"
    );


    if (!Modem_SendRawCommand(command))
    {
        Debug_Print(
            "ERROR: Could not start HTTP POST.\r\n"
        );

        return 0;
    }


    /* ---------------------------------------------------------------------- */
    /* Wait for CONNECT                                                       */
    /* ---------------------------------------------------------------------- */

    if (!Modem_WaitFor(
            "CONNECT",
            10000U))
    {
        Debug_Print(
            "ERROR: No CONNECT received for HTTP POST.\r\n"
        );

        return 0;
    }


    /* ---------------------------------------------------------------------- */
    /* Send HTTP header                                                       */
    /* ---------------------------------------------------------------------- */

    Debug_Print(
        "Sending HTTP POST request...\r\n"
    );

    /*
     * Do NOT Debug_Print(httpHeader) because it contains the
     * Authorization token.
     */

    if (HAL_UART_Transmit(
            &huart3,
            (uint8_t *)httpHeader,
            (uint16_t)headerLength,
            HAL_MAX_DELAY) != HAL_OK)
    {
        Debug_Print(
            "ERROR: Failed to transmit HTTP POST header.\r\n"
        );

        return 0;
    }


    /* ---------------------------------------------------------------------- */
    /* Wait for HTTP result                                                   */
    /* ---------------------------------------------------------------------- */

    if (!Modem_WaitFor(
            "+QHTTPPOST:",
            130000U))
    {
        Debug_Print(
            "ERROR: No QHTTPPOST result received.\r\n"
        );

        return 0;
    }


    /* ---------------------------------------------------------------------- */
    /* Get actual HTTP status                                                 */
    /* ---------------------------------------------------------------------- */

    /*
     * Modem_WaitFor() only tells us that "+QHTTPPOST:" appeared.
     *
     * The exact HTTP status was already consumed by Modem_WaitFor(),
     * so for this simple implementation we perform a second POST
     * result parsing function if exact status checking is required.
     *
     * For your current API, we expect:
     *
     *     +QHTTPPOST: 0,200
     *
     * Therefore we use a fresh transaction only if the helper has
     * retained the response. Since your current Modem_WaitFor()
     * does not expose the response, use the following expected
     * response directly instead.
     */

    return 1;
}

static int Modem_DeleteFirmwareFromS3(void)
{
    const OTA_Config_t *config =
        OTA_GetConfig();

    int httpStatus = 0;

    Debug_Print(
        "Deleting firmware from S3...\r\n"
    );

    if (!Modem_HttpPostNoBody(
            config->firmware_delete_url,
            config->ota_auth_token,
            &httpStatus))
    {
        Debug_Print(
            "ERROR: S3 firmware deletion request failed.\r\n"
        );

        return 0;
    }

    if (httpStatus != 200)
    {
        Debug_Print(
            "ERROR: S3 firmware deletion returned non-200.\r\n"
        );

        return 0;
    }

    Debug_Print(
        "Firmware deleted from S3 successfully.\r\n"
    );

    return 1;
}


static int Modem_ReportFirmwareUpdateSuccess(const char *firmwareFilename)
{
    const OTA_Config_t *config =
        (const OTA_Config_t *)OTA_CONFIG_ADDRESS;

    char response[512];
    char command[64];
    char httpHeader[512];

    char host[128];
    char path[256];

    uint32_t headerLength;
    uint32_t payloadLength;
    uint32_t totalLength;

    int headerResult;

    const char *urlStart;
    const char *pathStart;

    size_t hostLength;
    size_t pathLength;


    Debug_Print(
        "Reporting firmware update success...\r\n"
    );


    /* ---------------------------------------------------------------------- */
    /* Check configuration                                                    */
    /* ---------------------------------------------------------------------- */

    char payload[128];

    if (firmwareFilename == NULL ||
        firmwareFilename[0] == '\0')
    {
        return 0;
    }

    snprintf(payload,sizeof(payload),"%s\r\n",firmwareFilename);


    if (config->update_success_url[0] == '\0')
    {
        Debug_Print(
            "ERROR: Success API URL is empty.\r\n"
        );

        return 0;
    }


    /* ---------------------------------------------------------------------- */
    /* Extract hostname and path from update_success_url                     */
    /*                                                                        */
    /* Example:                                                               */
    /*                                                                        */
    /* https://70br5ujjbb.execute-api.ap-south-1.amazonaws.com/firmware-success */
    /*                                                                        */
    /* Host:                                                                  */
    /* 70br5ujjbb.execute-api.ap-south-1.amazonaws.com                        */
    /*                                                                        */
    /* Path:                                                                  */
    /* /firmware-success                                                      */
    /* ---------------------------------------------------------------------- */

    urlStart = strstr(
        config->update_success_url,
        "://"
    );

    if (urlStart == NULL)
    {
        Debug_Print(
            "ERROR: Invalid success URL. Missing protocol.\r\n"
        );

        return 0;
    }


    /* Skip "://" */

    urlStart += 3U;


    /* Find the first '/' after hostname */

    pathStart = strchr(
        urlStart,
        '/'
    );

    if (pathStart == NULL)
    {
        Debug_Print(
            "ERROR: Invalid success URL. Missing path.\r\n"
        );

        return 0;
    }


    /* ---------------------------------------------------------------------- */
    /* Extract hostname                                                       */
    /* ---------------------------------------------------------------------- */

    hostLength =
        (size_t)(pathStart - urlStart);


    if (hostLength == 0U ||
        hostLength >= sizeof(host))
    {
        Debug_Print(
            "ERROR: Invalid or oversized hostname.\r\n"
        );

        return 0;
    }


    memcpy(
        host,
        urlStart,
        hostLength
    );

    host[hostLength] = '\0';


    /* ---------------------------------------------------------------------- */
    /* Extract HTTP path                                                      */
    /* ---------------------------------------------------------------------- */

    pathLength =
        strlen(pathStart);


    if (pathLength == 0U ||
        pathLength >= sizeof(path))
    {
        Debug_Print(
            "ERROR: Invalid or oversized URL path.\r\n"
        );

        return 0;
    }


    memcpy(
        path,
        pathStart,
        pathLength
    );

    path[pathLength] = '\0';


    /* ---------------------------------------------------------------------- */
    /* Configure HTTP context                                                 */
    /* ---------------------------------------------------------------------- */

    if (!Modem_SendCommand(
            "AT+QHTTPCFG=\"contextid\",1\r",
            response,
            sizeof(response),
            5000U))
    {
        Debug_Print(
            "ERROR: Could not configure HTTP context.\r\n"
        );

        return 0;
    }


    /* ---------------------------------------------------------------------- */
    /* Configure SSL context                                                  */
    /* ---------------------------------------------------------------------- */

    if (!Modem_SendCommand(
            "AT+QHTTPCFG=\"sslctxid\",1\r",
            response,
            sizeof(response),
            5000U))
    {
        Debug_Print(
            "ERROR: Could not configure HTTP SSL context.\r\n"
        );

        return 0;
    }


    /*
     * Prototype SSL configuration.
     *
     * seclevel=0 means server certificate verification is disabled.
     * HTTPS encryption is still used.
     */

    if (!Modem_SendCommand(
            "AT+QSSLCFG=\"seclevel\",1,0\r",
            response,
            sizeof(response),
            5000U))
    {
        Debug_Print(
            "ERROR: SSL security level configuration failed.\r\n"
        );

        return 0;
    }


    if (!Modem_SendCommand(
            "AT+QSSLCFG=\"sslversion\",1,4\r",
            response,
            sizeof(response),
            5000U))
    {
        Debug_Print(
            "ERROR: SSL version configuration failed.\r\n"
        );

        return 0;
    }


    if (!Modem_SendCommand(
            "AT+QSSLCFG=\"ciphersuite\",1,0xFFFF\r",
            response,
            sizeof(response),
            5000U))
    {
        Debug_Print(
            "ERROR: Cipher configuration failed.\r\n"
        );

        return 0;
    }


    if (!Modem_SendCommand(
            "AT+QSSLCFG=\"sni\",1,1\r",
            response,
            sizeof(response),
            5000U))
    {
        Debug_Print(
            "ERROR: SNI configuration failed.\r\n"
        );

        return 0;
    }


    /* ---------------------------------------------------------------------- */
    /* Configure HTTP response header mode                                    */
    /* ---------------------------------------------------------------------- */

    if (!Modem_SendCommand(
            "AT+QHTTPCFG=\"responseheader\",0\r",
            response,
            sizeof(response),
            5000U))
    {
        Debug_Print(
            "ERROR: Could not configure response header mode.\r\n"
        );

        return 0;
    }


    /* ---------------------------------------------------------------------- */
    /* Enable custom HTTP request headers                                     */
    /* ---------------------------------------------------------------------- */

    if (!Modem_SendCommand(
            "AT+QHTTPCFG=\"requestheader\",1\r",
            response,
            sizeof(response),
            5000U))
    {
        Debug_Print(
            "ERROR: Could not enable HTTP request headers.\r\n"
        );

        return 0;
    }


    /* ---------------------------------------------------------------------- */
    /* Configure URL in EC200U                                                */
    /*                                                                        */
    /* The complete URL comes from the OTA configuration.                    */
    /* ---------------------------------------------------------------------- */

    if (!Modem_SetHTTPURL(
            config->update_success_url))
    {
        Debug_Print(
            "ERROR: Could not set OTA success URL.\r\n"
        );

        return 0;
    }


    /* ---------------------------------------------------------------------- */
    /* Calculate payload length                                               */
    /* ---------------------------------------------------------------------- */

    payloadLength =
        (uint32_t)strlen(payload);


    /* ---------------------------------------------------------------------- */
    /* Build HTTP PUT request header                                          */
    /*                                                                        */
    /* Example generated request:                                            */
    /*                                                                        */
    /* PUT /firmware-success HTTP/1.1                                        */
    /* Host: 70br5ujjbb.execute-api.ap-south-1.amazonaws.com                 */
    /* Authorization: GREENLEAP                                              */
    /* Content-Type: text/plain                                               */
    /* Content-Length: 28                                                     */
    /* Connection: close                                                      */
    /*                                                                        */
    /* The hostname and path are taken from update_success_url.              */
    /* ---------------------------------------------------------------------- */

    headerResult = snprintf(
        httpHeader,
        sizeof(httpHeader),

        "PUT %s HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Authorization: %s\r\n"
        "Content-Type: text/plain\r\n"
        "Content-Length: %lu\r\n"
        "Connection: close\r\n"
        "\r\n",

        path,
        host,
		config->ota_auth_token,
        (unsigned long)payloadLength
    );


    if (headerResult < 0)
    {
        Debug_Print(
            "ERROR: Could not build HTTP header.\r\n"
        );

        return 0;
    }


    if ((uint32_t)headerResult >= sizeof(httpHeader))
    {
        Debug_Print(
            "ERROR: HTTP header buffer too small.\r\n"
        );

        return 0;
    }


    headerLength =
        (uint32_t)headerResult;


    /* ---------------------------------------------------------------------- */
    /* EC200U QHTTPPUT data length                                            */
    /*                                                                        */
    /* Because requestheader=1, the modem expects:                            */
    /*                                                                        */
    /* totalLength = HTTP header + HTTP body                                  */
    /* ---------------------------------------------------------------------- */

    totalLength =
        headerLength + payloadLength;


    /* ---------------------------------------------------------------------- */
    /* Start HTTP PUT                                                         */
    /* ---------------------------------------------------------------------- */

    snprintf(
        command,
        sizeof(command),

        "AT+QHTTPPUT=%lu,30,120\r",

        (unsigned long)totalLength
    );


    if (!Modem_SendRawCommand(command))
    {
        Debug_Print(
            "ERROR: Could not start authenticated HTTP PUT.\r\n"
        );

        return 0;
    }


    /* ---------------------------------------------------------------------- */
    /* Wait for CONNECT                                                       */
    /* ---------------------------------------------------------------------- */

    if (!Modem_WaitFor(
            "CONNECT",
            10000U))
    {
        Debug_Print(
            "ERROR: No CONNECT received for HTTP PUT.\r\n"
        );

        return 0;
    }


    /*
     * From this point the EC200U expects exactly:
     *
     *     HTTP header
     *     +
     *     HTTP body
     *
     * Do not print the header because it contains the Authorization token.
     */


    /* ---------------------------------------------------------------------- */
    /* Send HTTP header                                                       */
    /* ---------------------------------------------------------------------- */

    if (HAL_UART_Transmit(
            &huart3,
            (uint8_t *)httpHeader,
            (uint16_t)headerLength,
            HAL_MAX_DELAY) != HAL_OK)
    {
        Debug_Print(
            "ERROR: Failed to transmit HTTP request header.\r\n"
        );

        return 0;
    }


    /* ---------------------------------------------------------------------- */
    /* Send HTTP body                                                         */
    /* ---------------------------------------------------------------------- */

    if (HAL_UART_Transmit(
            &huart3,
            (uint8_t *)payload,
            (uint16_t)payloadLength,
            HAL_MAX_DELAY) != HAL_OK)
    {
        Debug_Print(
            "ERROR: Failed to transmit firmware success payload.\r\n"
        );

        return 0;
    }


    /* ---------------------------------------------------------------------- */
    /* Wait for HTTP PUT result                                               */
    /* ---------------------------------------------------------------------- */

    if (!Modem_WaitFor(
            "+QHTTPPUT: 0,200",
            130000U))
    {
        Debug_Print(
            "ERROR: Firmware success report failed.\r\n"
        );

        return 0;
    }


    Debug_Print(
        "Firmware update success uploaded successfully.\r\n"
    );


    return 1;
}

/* Perform complete firmware update                                           */

/* -------------------------------------------------------------------------- */

static int Perform_Firmware_Update_Streaming(const char *firmwareFilename,
                                               uint32_t firmwareSize)
{
    if (firmwareFilename == NULL ||
        firmwareFilename[0] == '\0')
    {
        Debug_Print("ERROR: Firmware filename is invalid.\r\n");
        return 0;
    }

    if (firmwareSize == 0U ||
        firmwareSize > APPLICATION_MAX_SIZE)
    {
        Debug_Print("ERROR: Invalid streamed firmware size.\r\n");
        return 0;
    }

    Debug_Print("\r\n========================================\r\n");
    Debug_Print("       VERIFYING STREAMED FIRMWARE      \r\n");
    Debug_Print("========================================\r\n");

    /* -------------------------------------------------------------- */
    /* Verify the image already written to STM32 Flash                 */
    /* -------------------------------------------------------------- */

    Debug_Print("Verifying application...\r\n");

    if (!Verify_Application(firmwareSize))
    {
        Debug_Print("ERROR: Application verification failed!\r\n");
        return 0;
    }

    Debug_Print("Application verification OK.\r\n");

    /* -------------------------------------------------------------- */
    /* Report successful firmware update                               */
    /* -------------------------------------------------------------- */

    Debug_Print("Reporting firmware update success...\r\n");

    if (!Modem_ReportFirmwareUpdateSuccess(firmwareFilename))
    {
        Debug_Print("WARNING: Could not report firmware update success.\r\n");

        /*
         * The new application is already written and verified.
         * Do not modify or erase it.
         *
         * Also do not delete the firmware from S3 because the cloud
         * has not received the success confirmation.
         */
        return 0;
    }

    Debug_Print("Firmware update success reported to cloud.\r\n");

    /* -------------------------------------------------------------- */
    /* Delete firmware from S3                                         */
    /* -------------------------------------------------------------- */

    Debug_Print("Deleting firmware from S3...\r\n");

    if (!Modem_DeleteFirmwareFromS3())
    {
        Debug_Print("WARNING: Could not delete firmware from S3.\r\n");
        /* Installation is already successful, so keep the new image. */
    }
    else
    {
        Debug_Print("Firmware deleted from S3 successfully.\r\n");
    }

    Debug_Print("\r\n========================================\r\n");
    Debug_Print("       FIRMWARE UPDATE SUCCESSFUL      \r\n");
    Debug_Print("========================================\r\n");

    return 1;
}

/* -------------------------------------------------------------------------- */

static int Perform_Firmware_Update(const char *firmwareFilename)
{
    uint32_t firmwareSize = 0;
    uint32_t fileHandle = 0;
    uint32_t flashAddress;
    uint32_t remaining;

    uint8_t buffer[MODEM_READ_CHUNK_SIZE];

    char message[128];

    /* ---------------------------------------------------------------------- */
    /* Validate firmware filename                                             */
    /* ---------------------------------------------------------------------- */

    if (firmwareFilename == NULL ||
        firmwareFilename[0] == '\0')
    {
        Debug_Print(
            "ERROR: Firmware filename is invalid!\r\n"
        );

        return 0;
    }

    Debug_Print(
        "\r\n========================================\r\n"
    );

    Debug_Print(
        "       FIRMWARE UPDATE STARTING        \r\n"
    );

    Debug_Print(
        "========================================\r\n"
    );

    snprintf(
        message,
        sizeof(message),
        "Firmware filename: %s\r\n",
        firmwareFilename
    );

    Debug_Print(message);

    /* ---------------------------------------------------------------------- */
    /* Check OTA configuration                                                */
    /* ---------------------------------------------------------------------- */

    if (!OTA_ConfigIsValid())
    {
        Debug_Print(
            "ERROR: Invalid OTA configuration!\r\n"
        );

        return 0;
    }

    /* ---------------------------------------------------------------------- */
    /* Check local firmware file                                              */
    /*                                                                      */
    /* The cloud firmware may have any filename, but we intentionally store */
    /* it in the modem as firmware.bin.                                     */
    /* ---------------------------------------------------------------------- */

    Debug_Print(
        "Checking for firmware update...\r\n"
    );

    if (!Modem_CheckForUpdate())
    {
        Debug_Print(
            "ERROR: No firmware.bin found in modem UFS.\r\n"
        );

        return 0;
    }

    Debug_Print(
        "firmware.bin FOUND!\r\n"
    );

    /* ---------------------------------------------------------------------- */
    /* Get firmware size                                                      */
    /* ---------------------------------------------------------------------- */

    if (!Modem_GetFirmwareSize(&firmwareSize))
    {
        Debug_Print(
            "ERROR: Could not get firmware size.\r\n"
        );

        return 0;
    }

    snprintf(
        message,
        sizeof(message),
        "Firmware size: %lu bytes\r\n",
        (unsigned long)firmwareSize
    );

    Debug_Print(message);

    /* ---------------------------------------------------------------------- */
    /* Check firmware size                                                    */
    /* ---------------------------------------------------------------------- */

    if (firmwareSize == 0U)
    {
        Debug_Print(
            "ERROR: Firmware size is zero.\r\n"
        );

        return 0;
    }

    if (firmwareSize > APPLICATION_MAX_SIZE)
    {
        Debug_Print(
            "ERROR: Firmware too large!\r\n"
        );

        return 0;
    }

    /* ---------------------------------------------------------------------- */
    /* Open firmware file                                                     */
    /* ---------------------------------------------------------------------- */

    if (!Modem_OpenFirmware(&fileHandle))
    {
        Debug_Print(
            "ERROR: Could not open firmware.\r\n"
        );

        return 0;
    }

    snprintf(
        message,
        sizeof(message),
        "Firmware opened. Handle = %lu\r\n",
        (unsigned long)fileHandle
    );

    Debug_Print(message);

    /* ---------------------------------------------------------------------- */
    /* Erase application region                                               */
    /* ---------------------------------------------------------------------- */

    Debug_Print(
        "Erasing application region...\r\n"
    );

    if (Erase_Application() != HAL_OK)
    {
        Debug_Print(
            "ERROR: Application erase failed!\r\n"
        );

        Modem_CloseFile(fileHandle);

        return 0;
    }

    Debug_Print(
        "Application region erased.\r\n"
    );

    /* ---------------------------------------------------------------------- */
    /* Write firmware to STM32 Flash                                          */
    /* ---------------------------------------------------------------------- */

    flashAddress = APPLICATION_ADDRESS;
    remaining = firmwareSize;

    while (remaining > 0U)
    {
        uint32_t chunkSize;

        chunkSize =
            (remaining > MODEM_READ_CHUNK_SIZE)
            ?
            MODEM_READ_CHUNK_SIZE
            :
            remaining;

        /* ------------------------------------------------------------------ */
        /* Read binary data from modem                                        */
        /* ------------------------------------------------------------------ */

        if (!Modem_ReadFile(
                fileHandle,
                buffer,
                chunkSize))
        {
            Debug_Print(
                "ERROR: Modem file read failed!\r\n"
            );

            Modem_CloseFile(fileHandle);

            return 0;
        }

        /* ------------------------------------------------------------------ */
        /* Write data into STM32 Flash                                        */
        /* ------------------------------------------------------------------ */

        if (Flash_WriteBuffer(
                flashAddress,
                buffer,
                chunkSize) != HAL_OK)
        {
            Debug_Print(
                "ERROR: Flash write failed!\r\n"
            );

            Modem_CloseFile(fileHandle);

            return 0;
        }

        flashAddress += chunkSize;
        remaining -= chunkSize;

        /* ------------------------------------------------------------------ */
        /* Print progress                                                     */
        /* ------------------------------------------------------------------ */

        {
            uint32_t written;
            uint32_t percent;

            written = firmwareSize - remaining;

            percent =
                (written * 100U) /
                firmwareSize;

            snprintf(
                message,
                sizeof(message),
                "Firmware progress: %lu%%\r\n",
                (unsigned long)percent
            );

            Debug_Print(message);
        }
    }

    /* ---------------------------------------------------------------------- */
    /* Close modem firmware file                                              */
    /* ---------------------------------------------------------------------- */

    if (!Modem_CloseFile(fileHandle))
    {
        Debug_Print(
            "WARNING: Could not close modem file.\r\n"
        );
    }

    Debug_Print(
        "Firmware written successfully.\r\n"
    );

    /* ---------------------------------------------------------------------- */
    /* Verify application                                                     */
    /* ---------------------------------------------------------------------- */

    Debug_Print(
        "Verifying application...\r\n"
    );

    if (!Verify_Application(firmwareSize))
    {
        Debug_Print(
            "ERROR: Application verification failed!\r\n"
        );

        return 0;
    }

    Debug_Print(
        "Application verification OK.\r\n"
    );

    /* ---------------------------------------------------------------------- */
    /* Report firmware update success                                         */
    /* ---------------------------------------------------------------------- */
    /*
     * Send the actual cloud firmware filename.
     *
     * Example:
     *
     *     firmware filename = "update_v2.0.bin"
     *
     * Success Lambda creates:
     *
     *     success_update_v2.0.bin
     *
     */

    Debug_Print(
        "Reporting firmware update success...\r\n"
    );

    if (!Modem_ReportFirmwareUpdateSuccess(
            firmwareFilename))
    {
        Debug_Print(
            "WARNING: Could not report firmware update success.\r\n"
        );

        /*
         * The STM32 application has already been successfully written
         * and verified.
         *
         * Therefore we do NOT erase or modify the new application.
         *
         * We also do NOT delete the firmware from S3 because the cloud
         * has not received confirmation that the update succeeded.
         */

        return 0;
    }

    Debug_Print(
        "Firmware update success reported to cloud.\r\n"
    );

    /* ---------------------------------------------------------------------- */
    /* Delete firmware from S3                                                */
    /* ---------------------------------------------------------------------- */

    Debug_Print(
        "Deleting firmware from S3...\r\n"
    );

    if (!Modem_DeleteFirmwareFromS3())
    {
        Debug_Print(
            "WARNING: Could not delete firmware from S3.\r\n"
        );

        /*
         * The firmware is already successfully installed.
         *
         * Do not treat this as an application failure.
         *
         * The cloud firmware may remain in S3 and can be cleaned up
         * on a later attempt.
         */
    }
    else
    {
        Debug_Print(
            "Firmware deleted from S3 successfully.\r\n"
        );
    }

    /* ---------------------------------------------------------------------- */
    /* Delete temporary firmware file from modem                              */
    /* ---------------------------------------------------------------------- */

    Debug_Print(
        "Deleting local firmware.bin...\r\n"
    );

    if (!Modem_DeleteFirmware())
    {
        Debug_Print(
            "WARNING: Could not delete local firmware.bin.\r\n"
        );
    }
    else
    {
        Debug_Print(
            "firmware.bin deleted from modem.\r\n"
        );
    }

    /* ---------------------------------------------------------------------- */
    /* Update successful                                                       */
    /* ---------------------------------------------------------------------- */

    Debug_Print(
        "\r\n========================================\r\n"
    );

    Debug_Print(
        "       FIRMWARE UPDATE SUCCESSFUL      \r\n"
    );

    Debug_Print(
        "========================================\r\n"
    );

    return 1;
}


/* -------------------------------------------------------------------------- */

/* Jump to application                                                        */

/* -------------------------------------------------------------------------- */

static void Jump_To_Application(void)

{

    uint32_t applicationStack;

    uint32_t applicationResetHandler;


    typedef void (*pFunction)(void);

    pFunction Application;

    /*

     * Read application stack pointer.

     */

    applicationStack =

        *(__IO uint32_t *)

        APPLICATION_ADDRESS;

    /*

     * Read application Reset_Handler.

     */

    applicationResetHandler =

        *(__IO uint32_t *)

        (APPLICATION_ADDRESS + 4U);

    /*

     * Check valid application.

     */

    if ((applicationStack & 0x2FFE0000U)

        != 0x20000000U)

    {

        Debug_Print(

            "ERROR: No valid application found!\r\n"

        );

        while (1)

        {

        }

    }

    Debug_Print(

        "Jumping to application...\r\n"

    );


    /*

     * Disable interrupts while cleaning

     * bootloader state.

     */

    __disable_irq();

    /*

     * Stop bootloader SysTick.

     */

    SysTick->CTRL = 0;

    SysTick->LOAD = 0;

    SysTick->VAL = 0;

    /*

     * Disable and clear NVIC interrupts.

     */

    for (uint32_t i = 0;

         i < 8;

         i++)

    {

        NVIC->ICER[i] =

            0xFFFFFFFFU;

        NVIC->ICPR[i] =

            0xFFFFFFFFU;

    }

    /*

     * Set application vector table.

     */

    SCB->VTOR =

        APPLICATION_ADDRESS;


    __DSB();

    __ISB();


    /*

     * Set application's stack pointer.

     */

    __set_MSP(

        applicationStack

    );

    /*

     * Get application's Reset_Handler.

     */

    Application =

        (pFunction)

        applicationResetHandler;


    /*

     * IMPORTANT:

     *

     * Re-enable interrupts before

     * entering application.

     */

    __enable_irq();


    /*

     * Jump.

     */

    Application();

    /*

     * Should never return.

     */

    while (1)

    {

    }

}


/* -------------------------------------------------------------------------- */

/* Main                                                                       */

/* -------------------------------------------------------------------------- */

int main(void)

{

    /*

     * Initialize HAL.

     */

    HAL_Init();

    /*

     * Configure system clock.

     */

    SystemClock_Config();

    /*

     * Initialize GPIO.

     */

    MX_GPIO_Init();


    /*

     * USART2:

     * ST-LINK debug.

     */

    MX_USART2_UART_Init();

    /*

     * USART3:

     * EC200U modem.

     */

    MX_USART3_UART_Init();
    MX_USART3_DMA_Init();

    /*

     * Bootloader banner.

     */

    Debug_Print("\r\n");

    Debug_Print("================================\r\n");

    Debug_Print("       STM32 OTA BOOTLOADER       \r\n" );

    Debug_Print("================================\r\n");

    Debug_Print("Bootloader detected.\r\n");

    /*

     * Give EC200U enough time to boot after MCU reset.

     *

     * This matches the known-working EC200U application.

     */

    Debug_Print(

        "Waiting for EC200U to boot...\r\n");

    HAL_Delay(5000);



    Debug_Print("Testing EC200U...\r\n");

    {



        char response[1024];
        for(int i=0;i<3;i++)
        {

        if (Modem_SendCommand(

                "AT\r\n",

                response,

                sizeof(response),

                3000))

        {

            Debug_Print("EC200U responded OK.\r\n");

            /*

             * We use only TX/RX/GND between STM32 and EC200U.

             * Explicitly disable RTS/CTS in the modem before entering

             * file data mode.

             */

            {

                char flowResponse[256];

                if (Modem_SendCommand(

                        "AT+IFC=0,0\r",

                        flowResponse,

                        sizeof(flowResponse),

                        3000))

                {

                    Debug_Print(

                        "EC200U hardware flow control disabled.\r\n");

                }

                else

                {

                    Debug_Print(

                        "WARNING: Could not configure EC200U flow control.\r\n"

                    );

                }

            }

        }


        else

        {

            Debug_Print(

                "WARNING: EC200U did not respond.\r\n"

            );


        }

    }


        int internetAvailable = Modem_ConnectToInternet();

        /*
         * OTA priority:
         *   1. Existing UFS:firmware.bin
         *   2. Authenticated AWS API Gateway firmware if no local file exists
         *   3. Existing application if neither exists
         */

        if (Modem_CheckForUpdate())
        {
            Debug_Print(
                "Local firmware.bin found. Installing it now...\r\n"
            );

            if (!Perform_Firmware_Update(g_firmware_filename))
            {
                Debug_Print(
                    "ERROR: Local firmware update failed.\r\n"
                );
            }
        }
        else if (internetAvailable)
        {
            /*
             * First ask AWS which firmware file is available.
             */

        	Modem_SendCommand(
        	    "AT+QHTTPSTOP\r",
        	    response,
        	    sizeof(response),
        	    10000U);

        	Modem_SendCommand(
        	    "AT+QHTTPCFG=\"contextid\",1\r",
        	    response,
        	    sizeof(response),
        	    10000U);

        	Modem_SendCommand("AT+QHTTPCFG=\"sslctxid\",1\r",
        	    response,
        	    sizeof(response),
        	    10000U);

        	Modem_SendCommand("AT+QHTTPCFG=\"sslctxid\",1\r",
        	    response,
        	    sizeof(response),
        	    10000U);


        	Modem_SendCommand(
        	    "AT+QHTTPCFG=\"requestheader\",1\r",
        	    response,
        	    sizeof(response),
        	    10000U
        	);
            if (Modem_GetFirmwareFilename(
                    g_firmware_filename,
                    sizeof(g_firmware_filename)))
            {
                Debug_Print(
                    "Firmware filename received. Downloading firmware...\r\n"
                );

                /*
                 * Download firmware from S3/API Gateway.
                 */
                {
                    uint32_t cloudFirmwareSize = 0U;

                    if (Modem_DownloadFirmwareFromS3(&cloudFirmwareSize))
                    {
                        Debug_Print(
                            "Firmware streamed to STM32. Verifying and finalizing...\r\n"
                        );

                        if (!Perform_Firmware_Update_Streaming(
                                g_firmware_filename,
                                cloudFirmwareSize))
                        {
                            Debug_Print(
                                "ERROR: Firmware installation/finalization failed.\r\n"
                            );
                        }
                    }
                    else
                    {
                        Debug_Print(
                            "ERROR: Firmware download failed.\r\n"
                        );

                        if (cloudFirmwareSize != 0U)
                        {
                            Debug_Print(
                                "CRITICAL: Application Flash was erased; staying in bootloader.\r\n"
                            );

                            while (1)
                            {
                                HAL_Delay(1000U);
                            }
                        }
                    }
                }
            }
            else
            {
                Debug_Print(
                    "No valid firmware filename received. "
                    "Keeping current application.\r\n"
                );
            }
        }
        else
        {
            Debug_Print(
                "No local firmware and no internet connection. "
                "Keeping current application.\r\n"
            );
        }

        Jump_To_Application();

        /*
         * Should never reach here.
         */
        while (1)
        {
        }
    /*

     * Should never reach here.

     */

    while (1)

    {

    }
    }

}

/* -------------------------------------------------------------------------- */

/* System Clock                                                               */

/* -------------------------------------------------------------------------- */

void SystemClock_Config(void)

{

    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /*

     * HSI = 8 MHz

     *

     * PLL x9 = 72 MHz

     */

    RCC_OscInitStruct.OscillatorType =

        RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState =

        RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue =

        RCC_HSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.PLL.PLLState =

        RCC_PLL_ON;

    RCC_OscInitStruct.PLL.PLLSource =

        RCC_PLLSOURCE_HSI;

    RCC_OscInitStruct.PLL.PLLMUL =

        RCC_PLL_MUL9;

    if (HAL_RCC_OscConfig(

            &RCC_OscInitStruct)

        != HAL_OK)

    {

        Error_Handler();

    }

    RCC_ClkInitStruct.ClockType =

          RCC_CLOCKTYPE_HCLK

        | RCC_CLOCKTYPE_SYSCLK

        | RCC_CLOCKTYPE_PCLK1

        | RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource =

        RCC_SYSCLKSOURCE_PLLCLK;

    RCC_ClkInitStruct.AHBCLKDivider =

        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =

        RCC_HCLK_DIV2;

    RCC_ClkInitStruct.APB2CLKDivider =

        RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(

            &RCC_ClkInitStruct,

            FLASH_LATENCY_2)

        != HAL_OK)

    {

        Error_Handler();

    }

}

/* -------------------------------------------------------------------------- */

/* USART2 - ST-LINK Debug                                                     */

/* -------------------------------------------------------------------------- */

static void MX_USART2_UART_Init(void)

{

    huart2.Instance =

        USART2;

    huart2.Init.BaudRate =

        115200;

    huart2.Init.WordLength =

        UART_WORDLENGTH_8B;

    huart2.Init.StopBits =

        UART_STOPBITS_1;

    huart2.Init.Parity =

        UART_PARITY_NONE;

    huart2.Init.Mode =

        UART_MODE_TX_RX;

    huart2.Init.HwFlowCtl =

        UART_HWCONTROL_NONE;

    huart2.Init.OverSampling =

        UART_OVERSAMPLING_16;

    if (HAL_UART_Init(

            &huart2)

        != HAL_OK)

    {

        Error_Handler();

    }

}

/* -------------------------------------------------------------------------- */

/* USART3 - EC200U                                                            */

/* -------------------------------------------------------------------------- */

static void MX_USART3_UART_Init(void)

{

    huart3.Instance =

        USART3;

    huart3.Init.BaudRate =

        115200;

    huart3.Init.WordLength =

        UART_WORDLENGTH_8B;

    huart3.Init.StopBits =

        UART_STOPBITS_1;

    huart3.Init.Parity =

        UART_PARITY_NONE;

    huart3.Init.Mode =

        UART_MODE_TX_RX;

    huart3.Init.HwFlowCtl =

        UART_HWCONTROL_NONE;

    huart3.Init.OverSampling =

        UART_OVERSAMPLING_16;

    if (HAL_UART_Init(

            &huart3)

        != HAL_OK)

    {

        Error_Handler();

    }

}

/* -------------------------------------------------------------------------- */

/* GPIO                                                                       */

/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* USART3 RX circular DMA                                                     */
/* STM32F303 USART3_RX -> DMA1 Channel 3.                                   */
/* -------------------------------------------------------------------------- */
static int OTA_StreamDMA_Start(void)
{
    HAL_StatusTypeDef status;
    if (huart3.hdmarx == NULL)
    {
        Debug_Print("ERROR: USART3 RX DMA is not linked.\r\n");
        return 0;
    }
    HAL_UART_DMAStop(&huart3);
    g_otaStreamReadPos = 0U;
    g_otaStreamDmaActive = 0U;
    memset(g_otaStreamDmaBuffer, 0, sizeof(g_otaStreamDmaBuffer));
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    status = HAL_UART_Receive_DMA(&huart3, g_otaStreamDmaBuffer,
                                  OTA_STREAM_DMA_BUFFER_SIZE);
    if (status != HAL_OK)
    {
        Debug_Print("ERROR: Could not start USART3 RX DMA.\r\n");
        return 0;
    }
    g_otaStreamDmaActive = 1U;
    return 1;
}
static void OTA_StreamDMA_Stop(void)
{
    if (g_otaStreamDmaActive != 0U)
    {
        HAL_UART_DMAStop(&huart3);
        g_otaStreamDmaActive = 0U;
    }
    __HAL_UART_CLEAR_OREFLAG(&huart3);
}
static uint32_t OTA_StreamDMA_WritePos(void)
{
    uint32_t remaining;
    if (huart3.hdmarx == NULL) return 0U;
    remaining = __HAL_DMA_GET_COUNTER(huart3.hdmarx);
    if (remaining > OTA_STREAM_DMA_BUFFER_SIZE)
        remaining = OTA_STREAM_DMA_BUFFER_SIZE;
    return (OTA_STREAM_DMA_BUFFER_SIZE - remaining) % OTA_STREAM_DMA_BUFFER_SIZE;
}
static uint32_t OTA_StreamDMA_Available(void)
{
    uint32_t writePos;
    if (g_otaStreamDmaActive == 0U) return 0U;
    writePos = OTA_StreamDMA_WritePos();
    if (writePos >= g_otaStreamReadPos)
        return writePos - g_otaStreamReadPos;
    return (OTA_STREAM_DMA_BUFFER_SIZE - g_otaStreamReadPos) + writePos;
}
static int OTA_StreamDMA_ReadByte(uint8_t *byte, uint32_t timeout)
{
    uint32_t start;
    if (byte == NULL) return 0;
    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout)
    {
        if (OTA_StreamDMA_Available() > 0U)
        {
            *byte = g_otaStreamDmaBuffer[g_otaStreamReadPos++];
            if (g_otaStreamReadPos >= OTA_STREAM_DMA_BUFFER_SIZE)
                g_otaStreamReadPos = 0U;
            return 1;
        }
    }
    return 0;
}
static int OTA_StreamDMA_Read(uint8_t *buffer, uint32_t length, uint32_t timeout)
{
    uint32_t received = 0U;
    uint32_t lastDataTick = HAL_GetTick();
    if (buffer == NULL || length == 0U) return 0;
    while (received < length)
    {
        uint32_t available = OTA_StreamDMA_Available();
        if (available == 0U)
        {
            if ((HAL_GetTick() - lastDataTick) >= timeout)
                return 0;
            continue;
        }
        {
            uint32_t contiguous = OTA_STREAM_DMA_BUFFER_SIZE - g_otaStreamReadPos;
            uint32_t copyLength = length - received;
            if (available < copyLength) copyLength = available;
            if (contiguous < copyLength) copyLength = contiguous;
            memcpy(&buffer[received], &g_otaStreamDmaBuffer[g_otaStreamReadPos], copyLength);
            g_otaStreamReadPos += copyLength;
            if (g_otaStreamReadPos >= OTA_STREAM_DMA_BUFFER_SIZE)
                g_otaStreamReadPos = 0U;
            received += copyLength;
            lastDataTick = HAL_GetTick();
        }
    }
    return 1;
}
static void MX_USART3_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma_usart3_rx.Instance = DMA1_Channel3;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
        Error_Handler();
    __HAL_LINKDMA(&huart3, hdmarx, hdma_usart3_rx);
}

static void MX_GPIO_Init(void)

{

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*

     * GPIOA clock.

     */

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*

     * LD2 = PA5.

     */

    HAL_GPIO_WritePin(

        GPIOA,

        GPIO_PIN_5,

        GPIO_PIN_RESET

    );

    GPIO_InitStruct.Pin =

        GPIO_PIN_5;

    GPIO_InitStruct.Mode =

        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =

        GPIO_NOPULL;

    GPIO_InitStruct.Speed =

        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(

        GPIOA,

        &GPIO_InitStruct

    );

}

/* -------------------------------------------------------------------------- */

/* Error Handler                                                              */

/* -------------------------------------------------------------------------- */

void Error_Handler(void)

{

    __disable_irq();

    while (1)

    {

    }

}

#ifdef USE_FULL_ASSERT

void assert_failed(

    uint8_t *file,

    uint32_t line

)

{

    /*

     * User can add implementation here.

     */

}


#endif
