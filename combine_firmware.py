from pathlib import Path

# STM32F303RE flash layout
FLASH_START_ADDRESS = 0x08000000
FLASH_SIZE = 512 * 1024  # 512 KB

BOOTLOADER_ADDRESS = 0x08000000
BOOTLOADER_SIZE = 64 * 1024  # 64 KB

APPLICATION_ADDRESS = 0x08010000

OTA_CONFIG_ADDRESS = 0x0807F800
OTA_CONFIG_SIZE = 2 * 1024  # 2 KB

# File paths
BOOTLOADER_FILE = Path("custom_bootloader_3.0.bin")
APPLICATION_FILE = Path("lll.bin")
OUTPUT_FILE = Path("combined_firmware.bin")

# Calculate offsets within the combined binary
BOOTLOADER_OFFSET = BOOTLOADER_ADDRESS - FLASH_START_ADDRESS
APPLICATION_OFFSET = APPLICATION_ADDRESS - FLASH_START_ADDRESS
OTA_CONFIG_OFFSET = OTA_CONFIG_ADDRESS - FLASH_START_ADDRESS


def main():
    if not BOOTLOADER_FILE.is_file():
        raise FileNotFoundError(f"Missing file: {BOOTLOADER_FILE}")

    if not APPLICATION_FILE.is_file():
        raise FileNotFoundError(f"Missing file: {APPLICATION_FILE}")

    bootloader_full = BOOTLOADER_FILE.read_bytes()
    application = APPLICATION_FILE.read_bytes()

    print(f"Original bootloader BIN size: {len(bootloader_full)} bytes")

    # Extract OTA configuration from its absolute flash offset.
    # The bootloader BIN must start at 0x08000000.
    ota_config = bytearray(b"\xFF" * OTA_CONFIG_SIZE)

    if len(bootloader_full) > OTA_CONFIG_OFFSET:
        available = min(
            OTA_CONFIG_SIZE,
            len(bootloader_full) - OTA_CONFIG_OFFSET
        )

        ota_config[:available] = bootloader_full[
            OTA_CONFIG_OFFSET:OTA_CONFIG_OFFSET + available
        ]

        print(f"Extracted OTA config bytes: {available}")
    else:
        print(
            "WARNING: No OTA config data found at the expected offset. "
            "The config region will remain erased (0xFF)."
        )

    # Keep only the bootloader's 64 KB flash region.
    bootloader = bootloader_full[:BOOTLOADER_SIZE]

    # Calculate maximum application size.
    application_capacity = OTA_CONFIG_ADDRESS - APPLICATION_ADDRESS

    if len(bootloader) > BOOTLOADER_SIZE:
        raise ValueError("Bootloader exceeds its allocated region.")

    if len(application) > application_capacity:
        raise ValueError(
            f"Application too large: {len(application)} bytes. "
            f"Maximum allowed: {application_capacity} bytes."
        )

    if BOOTLOADER_OFFSET + len(bootloader) > APPLICATION_OFFSET:
        raise ValueError("Bootloader overlaps the application.")

    # Create the complete 512 KB flash image.
    # Unused flash bytes are filled with 0xFF.
    combined = bytearray(b"\xFF" * FLASH_SIZE)

    # Place bootloader at 0x08000000.
    combined[
        BOOTLOADER_OFFSET:BOOTLOADER_OFFSET + len(bootloader)
    ] = bootloader

    # Place application at 0x08010000.
    combined[
        APPLICATION_OFFSET:APPLICATION_OFFSET + len(application)
    ] = application

    # Restore OTA config at 0x0807F800.
    combined[
        OTA_CONFIG_OFFSET:OTA_CONFIG_OFFSET + OTA_CONFIG_SIZE
    ] = ota_config

    # Write combined firmware.
    OUTPUT_FILE.write_bytes(combined)

    print("\nCombined firmware created successfully.")
    print(f"Output file: {OUTPUT_FILE.resolve()}")
    print(f"Combined size: {len(combined)} bytes")
    print(f"Bootloader: 0x{BOOTLOADER_ADDRESS:08X}")
    print(f"Application: 0x{APPLICATION_ADDRESS:08X}")
    print(f"OTA config: 0x{OTA_CONFIG_ADDRESS:08X}")
    print(f"OTA config size: {OTA_CONFIG_SIZE} bytes")


if __name__ == "__main__":
    main()