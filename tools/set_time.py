#!/usr/bin/env python3
"""Set time on Voyager device via BLE."""

import asyncio
import struct
import time

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "Voyager"
TIME_SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
TIMESTAMP_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef1"


async def main():
    print(f"Scanning for '{DEVICE_NAME}'...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)

    if device is None:
        print(f"Could not find device '{DEVICE_NAME}'")
        return

    print(f"Found {device.name} at {device.address}")

    async with BleakClient(device) as client:
        print("Connected!")

        # Get current Unix timestamp
        timestamp = int(time.time())
        print(f"Setting time to: {timestamp} ({time.ctime(timestamp)})")

        # Pack as 8-byte little-endian unsigned integer
        data = struct.pack("<Q", timestamp)

        await client.write_gatt_char(TIMESTAMP_CHAR_UUID, data)
        print("Time set successfully!")


if __name__ == "__main__":
    asyncio.run(main())
