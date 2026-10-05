#include <stdint.h>

#define OTA_CONFIG_MAGIC    0x4F544143UL
#define OTA_CONFIG_VERSION  1U

typedef struct
{
    uint32_t magic;
    uint32_t version;

    char firmware_download_url[256];
    char update_success_url[256];

    char firmware_auth_token[64];
    char success_auth_token[64];

    uint32_t crc;

} OTA_Config_t;


__attribute__((section(".ota_config"), used))
const OTA_Config_t ota_config =
{
    .magic = OTA_CONFIG_MAGIC,

    .version = OTA_CONFIG_VERSION,

    .firmware_download_url =
        "https://70br5ujjbb.execute-api.ap-south-1.amazonaws.com/firmware",

    .update_success_url =
        "https://70br5ujjbb.execute-api.ap-south-1.amazonaws.com/firmware-success",

    .firmware_auth_token =
        "GREENLEAP",

    .success_auth_token =
        "GREENLEAP",

    .crc = 0
};
