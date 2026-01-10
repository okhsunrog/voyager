#!/usr/bin/env python3
"""Reboot Voyager device into bootloader (DFU mode) via BLE."""

import asyncio

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "Voyager"
TIME_SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
REBOOT_DFU_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef2"


async def main():
    print(f"Scanning for '{DEVICE_NAME}'...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)

    if device is None:
        print(f"Could not find device '{DEVICE_NAME}'")
        return

    print(f"Found {device.name} at {device.address}")

    async with BleakClient(device) as client:
        print("Connected!")
        print("Sending reboot to DFU command...")

        # Write any value to trigger reboot
        await client.write_gatt_char(REBOOT_DFU_CHAR_UUID, b"\x01")
        print("Command sent! Device should reboot into bootloader.")
        print("You can now use UF2 drag-and-drop or serial DFU to update firmware.")


if __name__ == "__main__":
    asyncio.run(main())
