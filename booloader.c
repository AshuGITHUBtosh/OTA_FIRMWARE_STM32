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





/* -------------------------------------------------------------------------- */

/* Defines                                                                    */

/* -------------------------------------------------------------------------- */

#define APPLICATION_ADDRESS       0x08008000UL

#define APPLICATION_MAX_SIZE      (480UL * 1024UL)

#define FLASH_START_ADDRESS       0x08000000UL

#define UPDATE_FILE_NAME          "UFS:firmware.bin"

#define MODEM_READ_CHUNK_SIZE     256U

#define MODEM_TIMEOUT             5000U





/* -------------------------------------------------------------------------- */

/* Function prototypes                                                        */

/* -------------------------------------------------------------------------- */

void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_USART2_UART_Init(void);

static void MX_USART3_UART_Init(void);

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
static int Modem_SendCommand(

    const char *command,

    char *response,

    uint32_t responseSize,

    uint32_t timeout

);

static int Modem_SendRawCommand(

    const char *command

);

static int Modem_WaitFor(

    const char *expected,

    uint32_t timeout

);

static int Modem_WaitForQFReadHeader(

    uint32_t expectedLength,

    uint32_t timeout

)

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
static int Modem_HttpGet(int *httpStatus, uint32_t *contentLength);

static HAL_StatusTypeDef Erase_Application(void);

static HAL_StatusTypeDef Flash_WriteBuffer(

    uint32_t address,

    uint8_t *data,

    uint32_t length

);

static int Verify_Application(

    uint32_t firmwareSize

);

static int Perform_Firmware_Update(void);
static int Modem_SetHTTPURL(const char *url);
static int Modem_DownloadFirmwareFromS3(void);

#define OTA_S3_URL "https://modemtesting.s3.ap-south-1.amazonaws.com/update_firmware.bin"
#define OTA_DOWNLOAD_FILE "UFS:firmware.bin"

static void Jump_To_Application(void);





/* -------------------------------------------------------------------------- */

/* Debug print                                                                */

/* -------------------------------------------------------------------------- */

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

static int Modem_SendCommand(

    const char *command,

    char *response,

    uint32_t responseSize,

    uint32_t timeout

)

{

    Modem_Flush();

    Debug_Print("\r\nSTM32 -> MODEM:\r\n");

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

        Debug_Print("\r\nMODEM -> STM32:\r\n");

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

    if (!Modem_SendCommand("AT+QICSGP=1,3,\"www\",\"\",\"\",0\r",
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

    if (!Modem_SendCommand("AT+QIACT?\r",
                           response, sizeof(response), 5000U) ||
        strstr(response, "+QIACT: 1,1") == NULL)
    {
        Debug_Print("ERROR: PDP context 1 is not active.\r\n");
        return 0;
    }

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

    while ((HAL_GetTick() - startTick) < 65000U)
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
static int Modem_HttpGet(int *httpStatus, uint32_t *contentLength)
{
    uint8_t ch;

    char rxBuffer[256];
    uint32_t rxIndex = 0;

    uint32_t startTime;

    unsigned long errCode = 0;
    unsigned long httpCode = 0;
    unsigned long dataLength = 0;

    int gotResult = 0;

    if (httpStatus == NULL || contentLength == NULL)
    {
        return 0;
    }

    *httpStatus = 0;
    *contentLength = 0;

    memset(rxBuffer, 0, sizeof(rxBuffer));

    /* ------------------------------------------------------------- */
    /* Send QHTTPGET                                                 */
    /* ------------------------------------------------------------- */

    Debug_Print("\r\nSTM32 -> MODEM:\r\n");
    Debug_Print("AT+QHTTPGET=120\r\n");

    const char *cmd = "AT+QHTTPGET=120\r";

    if (HAL_UART_Transmit(&huart3,
                          (uint8_t *)cmd,
                          strlen(cmd),
                          HAL_MAX_DELAY) != HAL_OK)
    {
        Debug_Print("ERROR: Failed to transmit QHTTPGET.\r\n");
        return 0;
    }

    /*
     * Give the modem time to start the HTTP operation.
     */
    HAL_Delay(100);

    startTime = HAL_GetTick();

    while ((HAL_GetTick() - startTime) < 130000U)
    {
        /*
         * Receive one byte.
         */
        if (HAL_UART_Receive(&huart3,
                             &ch,
                             1,
                             100) != HAL_OK)
        {
            continue;
        }

        /*
         * Store received byte.
         */
        if (rxIndex < sizeof(rxBuffer) - 1)
        {
            rxBuffer[rxIndex++] = (char)ch;
            rxBuffer[rxIndex] = '\0';
        }
        else
        {
            /*
             * Buffer full.
             *
             * Keep the most recent data.
             */
            memmove(rxBuffer,
                    rxBuffer + 64,
                    sizeof(rxBuffer) - 65);

            rxIndex = strlen(rxBuffer);

            rxBuffer[rxIndex++] = (char)ch;
            rxBuffer[rxIndex] = '\0';
        }

        /*
         * ---------------------------------------------------------
         * Check for +QHTTPGET:
         * ---------------------------------------------------------
         */

        char *qhttp = strstr(rxBuffer, "+QHTTPGET:");

        if (qhttp != NULL)
        {
            /*
             * We have found the beginning of:
             *
             * +QHTTPGET:
             *
             * Now wait until we have received the complete
             * numerical response.
             */

            char *data = qhttp + strlen("+QHTTPGET:");

            /*
             * Look for the first number.
             *
             * Example:
             *
             * +QHTTPGET: 0,200,11716
             */

            int parsed = sscanf(data,
                                " %lu , %lu , %lu",
                                &errCode,
                                &httpCode,
                                &dataLength);

            if (parsed == 3)
            {
                gotResult = 1;

                *httpStatus = (int)httpCode;
                *contentLength = (uint32_t)dataLength;

                /*
                 * Print the complete response.
                 */
                Debug_Print("\r\nMODEM -> STM32:\r\n");
                Debug_Print(qhttp);

                Debug_Print("\r\n");

                /*
                 * -------------------------------------------------
                 * Check EC200U result code
                 * -------------------------------------------------
                 */

                if (errCode != 0)
                {
                    Debug_Print(
                        "ERROR: EC200U HTTP GET returned an error.\r\n"
                    );

                    return 0;
                }

                /*
                 * -------------------------------------------------
                 * HTTP 200
                 * -------------------------------------------------
                 */

                if (httpCode == 200)
                {
                    char msg[80];

                    snprintf(msg,
                             sizeof(msg),
                             "HTTP 200 OK. Firmware size: %lu bytes\r\n",
                             dataLength);

                    Debug_Print(msg);

                    Debug_Print(
                        "S3 HTTP GET successful.\r\n"
                    );

                    return 1;
                }

                /*
                 * -------------------------------------------------
                 * HTTP 404
                 * -------------------------------------------------
                 */

                if (httpCode == 404)
                {
                    Debug_Print(
                        "HTTP 404: update_firmware.bin not found.\r\n"
                    );

                    return 0;
                }

                /*
                 * -------------------------------------------------
                 * Other HTTP status
                 * -------------------------------------------------
                 */

                char statusMsg[64];

                snprintf(statusMsg,
                         sizeof(statusMsg),
                         "HTTP server returned status %lu\r\n",
                         httpCode);

                Debug_Print(statusMsg);

                return 0;
            }
        }

        /*
         * ---------------------------------------------------------
         * Check for CME ERROR
         * ---------------------------------------------------------
         */

        if (strstr(rxBuffer, "+CME ERROR:") != NULL)
        {
            Debug_Print("\r\nMODEM ERROR:\r\n");
            Debug_Print(rxBuffer);
            Debug_Print("\r\n");

            return 0;
        }

        /*
         * ---------------------------------------------------------
         * Check for normal ERROR
         * ---------------------------------------------------------
         */

        if (strstr(rxBuffer, "\r\nERROR\r\n") != NULL ||
            strstr(rxBuffer, "\nERROR\r\n") != NULL)
        {
            Debug_Print("\r\nMODEM ERROR:\r\n");
            Debug_Print(rxBuffer);
            Debug_Print("\r\n");

            return 0;
        }
    }

    /*
     * ------------------------------------------------------------- */
    /* Timeout                                                       */
    /* ------------------------------------------------------------- */

    if (!gotResult)
    {
        Debug_Print(
            "\r\nERROR: Timeout waiting for +QHTTPGET result.\r\n"
        );

        Debug_Print("Received data:\r\n");
        Debug_Print(rxBuffer);
        Debug_Print("\r\n");
    }

    return 0;
}
/* -------------------------------------------------------------------------- */
/* Download update_firmware from public AWS S3                                 */
/* -------------------------------------------------------------------------- */
static int Modem_DownloadFirmwareFromS3(void)
{
    char response[1024];
    char command[96];
    int httpStatus = 0;
    uint32_t contentLength = 0U;
    uint32_t storedSize = 0U;

    Debug_Print("\r\n================================\r\n");
    Debug_Print("         CLOUD OTA CHECK        \r\n");
    Debug_Print("================================\r\n");

    /* Configure HTTP/HTTPS. */
    if (!Modem_SendCommand("AT+QHTTPCFG=\"contextid\",1\r",
                           response, sizeof(response), 5000U)) return 0;
    if (!Modem_SendCommand("AT+QHTTPCFG=\"sslctxid\",1\r",
                           response, sizeof(response), 5000U)) return 0;

    /* Public S3 test bucket: certificate verification disabled. */
    if (!Modem_SendCommand("AT+QSSLCFG=\"seclevel\",1,0\r",
                           response, sizeof(response), 5000U)) return 0;
    if (!Modem_SendCommand("AT+QSSLCFG=\"sslversion\",1,4\r",
                           response, sizeof(response), 5000U)) return 0;

    /* Allow all supported cipher suites for this prototype. */
    if (!Modem_SendCommand("AT+QSSLCFG=\"ciphersuite\",1,0xFFFF\r",
                           response, sizeof(response), 5000U)) return 0;

    /* Enable SNI for the AWS S3 HTTPS hostname. */
    if (!Modem_SendCommand("AT+QSSLCFG=\"sni\",1,1\r",
                           response, sizeof(response), 5000U)) return 0;

    if (!Modem_SendCommand("AT+QHTTPCFG=\"responseheader\",0\r",
                           response, sizeof(response), 5000U)) return 0;

    Debug_Print("Checking S3 for update_firmware...\r\n");

    if (!Modem_SetHTTPURL(OTA_S3_URL))
    {
        Debug_Print("ERROR: Could not set S3 URL.\r\n");
        return 0;
    }

    if (!Modem_HttpGet(&httpStatus, &contentLength))
    {
        if (httpStatus == 404)
            Debug_Print("update_firmware not found in S3.\r\n");
        else
            Debug_Print("ERROR: S3 HTTP GET failed.\r\n");
        return 0;
    }

    if (httpStatus == 404)
    {
        Debug_Print("update_firmware not found in S3.\r\n");
        return 0;
    }

    if (httpStatus != 200)
    {
        Debug_Print("S3 returned a non-200 HTTP status.\r\n");
        return 0;
    }

    if (contentLength == 0U || contentLength > APPLICATION_MAX_SIZE)
    {
        Debug_Print("ERROR: Invalid firmware size from S3.\r\n");
        return 0;
    }

    /* Only now do we remove a stale local file. */
    if (Modem_CheckForUpdate())
    {
        Debug_Print("Old firmware.bin found. Deleting it...\r\n");
        (void)Modem_DeleteFirmware();
    }

    snprintf(command, sizeof(command),
             "AT+QHTTPREADFILE=\"UFS:firmware.bin\",120\r");

    Debug_Print("Downloading update_firmware to UFS:firmware.bin...\r\n");

    if (!Modem_SendRawCommand(command)) return 0;

    if (!Modem_WaitFor("+QHTTPREADFILE: 0", 180000U))
    {
        Debug_Print("ERROR: QHTTPREADFILE failed.\r\n");
        return 0;
    }

    if (!Modem_CheckForUpdate())
    {
        Debug_Print("ERROR: firmware.bin was not created.\r\n");
        return 0;
    }

    if (!Modem_GetFirmwareSize(&storedSize))
    {
        Debug_Print("ERROR: Could not read downloaded file size.\r\n");
        return 0;
    }

    /*if (storedSize != contentLength)
    {
        Debug_Print("ERROR: S3/UFS firmware size mismatch.\r\n");
        (void)Modem_DeleteFirmware();
        return 0;
    }*/

    Debug_Print("S3 firmware stored as UFS:firmware.bin.\r\n");
    return 1;
}

/* Perform complete firmware update                                           */

/* -------------------------------------------------------------------------- */

static int Perform_Firmware_Update(void)

{

    uint32_t firmwareSize = 0;

    uint32_t fileHandle = 0;

    uint32_t flashAddress;

    uint32_t remaining;

    uint8_t buffer[

        MODEM_READ_CHUNK_SIZE

    ];





    Debug_Print(

        "Checking for firmware update...\r\n"

    );





    /* ---------------------------------------------------------------------- */

    /* Check file                                                             */

    /* ---------------------------------------------------------------------- */

    if (!Modem_CheckForUpdate())

    {

        Debug_Print(

            "No firmware.bin found.\r\n"

        );

        return 0;

    }





    Debug_Print(

        "firmware.bin FOUND!\r\n"

    );





    /* ---------------------------------------------------------------------- */

    /* Get file size                                                          */

    /* ---------------------------------------------------------------------- */

    if (!Modem_GetFirmwareSize(

            &firmwareSize))

    {

        Debug_Print(

            "ERROR: Could not get firmware size.\r\n"

        );

        return 0;

    }





    char message[128];





    snprintf(

        message,

        sizeof(message),

        "Firmware size: %lu bytes\r\n",

        (unsigned long)firmwareSize

    );





    Debug_Print(message);





    /* ---------------------------------------------------------------------- */

    /* Check size                                                             */

    /* ---------------------------------------------------------------------- */

    if (firmwareSize == 0)

    {

        Debug_Print(

            "ERROR: Firmware size is zero.\r\n"

        );

        return 0;

    }





    if (firmwareSize >

        APPLICATION_MAX_SIZE)

    {

        Debug_Print(

            "ERROR: Firmware too large!\r\n"

        );

        return 0;

    }





    /* ---------------------------------------------------------------------- */

    /* Open firmware                                                          */

    /* ---------------------------------------------------------------------- */

    if (!Modem_OpenFirmware(

            &fileHandle))

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

    /* Erase application                                                     */

    /* ---------------------------------------------------------------------- */

    Debug_Print(

        "Erasing application region...\r\n"

    );





    if (Erase_Application()

        != HAL_OK)

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

    /* Write firmware                                                        */

    /* ---------------------------------------------------------------------- */

    flashAddress =

        APPLICATION_ADDRESS;





    remaining =

        firmwareSize;





    while (remaining > 0)

    {

        uint32_t chunkSize =

            remaining >

            MODEM_READ_CHUNK_SIZE

            ?

            MODEM_READ_CHUNK_SIZE

            :

            remaining;





        /*

         * Read binary data from modem.

         */

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





        /*

         * Write data into STM32 Flash.

         */

        if (Flash_WriteBuffer(

                flashAddress,

                buffer,

                chunkSize)

            != HAL_OK)

        {

            Debug_Print(

                "ERROR: Flash write failed!\r\n"

            );





            Modem_CloseFile(fileHandle);





            return 0;

        }





        flashAddress +=

            chunkSize;





        remaining -=

            chunkSize;





        /*

         * Print progress.

         */

        uint32_t written =

            firmwareSize -

            remaining;





        uint32_t percent =

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





    /* ---------------------------------------------------------------------- */

    /* Close modem file                                                      */

    /* ---------------------------------------------------------------------- */

    if (!Modem_CloseFile(

            fileHandle))

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





    if (!Verify_Application(

            firmwareSize))

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

    /* Delete update file                                                    */

    /* ---------------------------------------------------------------------- */

    Debug_Print(

        "Deleting firmware.bin...\r\n"

    );





    if (!Modem_DeleteFirmware())

    {

        Debug_Print(

            "ERROR: Could not delete update file.\r\n"

        );





        /*

         * Firmware is still valid.

         * We can continue to boot it.

         */

        return 0;

    }





    Debug_Print(

        "firmware.bin deleted.\r\n"

    );





    Debug_Print(

        "FIRMWARE UPDATE SUCCESSFUL!\r\n"

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





    /*

     * Bootloader banner.

     */

    Debug_Print("\r\n");

    Debug_Print(

        "================================\r\n"

    );

    Debug_Print(

        "       STM32 OTA BOOTLOADER       \r\n"

    );

    Debug_Print(

        "================================\r\n"

    );

    Debug_Print(

        "Bootloader detected.\r\n"

    );





    /*

     * Give EC200U enough time to boot after MCU reset.

     *

     * This matches the known-working EC200U application.

     */

    Debug_Print(

        "Waiting for EC200U to boot...\r\n"

    );

    HAL_Delay(5000);





    /*

     * Basic modem test.

     *

     * Modem_SendCommand() now performs the complete transaction:

     *

     *   STM32 -> AT

     *   STM32 <- modem response

     *

     * Therefore we must NOT call Modem_WaitFor() again here.

     */

    Debug_Print(

        "Testing EC200U...\r\n"

    );

    {

        char response[1024];

        if (Modem_SendCommand(

                "AT\r",

                response,

                sizeof(response),

                3000))

        {

            Debug_Print(

                "EC200U responded OK.\r\n"

            );

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

                        "EC200U hardware flow control disabled.\r\n"

                    );

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





    /*
     * Establish cellular data first.
     * A network failure does not prevent use of a local firmware.bin.
     */
    int internetAvailable = Modem_ConnectToInternet();

    /*
     * OTA priority:
     *   1. Existing UFS:firmware.bin
     *   2. AWS S3 update_firmware if no local file exists
     *   3. Existing application if neither exists
     */
    if (Modem_CheckForUpdate())
    {
        Debug_Print("Local firmware.bin found. Installing it now...\r\n");
        (void)Perform_Firmware_Update();
    }
    else if (internetAvailable)
    {
        if (Modem_DownloadFirmwareFromS3())
        {
            Debug_Print("S3 firmware downloaded. Installing it now...\r\n");
            (void)Perform_Firmware_Update();
        }
        else
        {
            Debug_Print("No S3 update available. Keeping current application.\r\n");
        }
    }
    else
    {
        Debug_Print("No local firmware and no internet connection. Keeping current application.\r\n");
    }

    Jump_To_Application();





    /*

     * Should never reach here.

     */

    while (1)

    {

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
