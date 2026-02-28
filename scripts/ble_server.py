"""Connects to the Arduino via BLE and sends a single byte"""

import asyncio
from bleak import BleakScanner, BleakClient

SERVICE_UUID = "88aadeff-64a4-47ae-8798-7d7e51b24e55"
CHARACTERISTIC_UUID = "88aadeff-64a4-47ae-8798-7d7e51b24e55"


async def run():
    print("Scanning for ArduinoNano33...")
    device = await BleakScanner.find_device_by_filter(
        lambda d, ad: d.name == "Nano33BLE"
    )

    if not device:
        print("Device not found.")
        return

    async with BleakClient(device) as client:
        print(f"Connected: {client.is_connected}")

        await client.write_gatt_char(CHARACTERISTIC_UUID, bytearray([0x01]))


asyncio.run(run())
