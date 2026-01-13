import asyncio
import platform
from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError
from threading import Thread
import time

# Windows-specific imports for connection parameter tuning
if platform.system() == "Windows":
    try:
        from winrt.windows.devices.bluetooth import BluetoothLEPreferredConnectionParameters
        WINRT_AVAILABLE = True
    except ImportError:
        WINRT_AVAILABLE = False
else:
    WINRT_AVAILABLE = False

READ_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
WRITE_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"


class APP3Backend:

    def __init__(self, address, callback_function):
        self.address = address
        self.callback_function = callback_function
        self.exit_flag = False
        self.output_buff = []
        self.status = 'Disconnected'

    def disconnect(self):
        self.exit_flag = True

    def send_data(self, data: bytes):
        self.output_buff.append(data)

    def handle_disconnect(self, _: BleakClient):
        self.status = 'Disconnected'
        print("Device was disconnected, goodbye.")
        # cancelling all tasks effectively ends the program
        for task in asyncio.all_tasks():
            task.cancel()

    async def _tune_connection_for_throughput(self, client: BleakClient):
        """
        Tune BLE connection parameters for maximum throughput on Windows.
        Uses BluetoothLEDevice.from_bluetooth_address_async to get a WinRT device object.
        """
        if not WINRT_AVAILABLE:
            return False

        try:
            from winrt.windows.devices.bluetooth import BluetoothLEDevice

            # Convert MAC address string to integer (required by WinRT API)
            # Format: "C4:4D:5E:E3:AD:D7" -> 0xC44D5EE3ADD7
            mac_str = self.address.replace(":", "")
            mac_int = int(mac_str, 16)

            print(f"  [BLE-Tune] 正在通过地址 {self.address} 获取 WinRT 设备...")

            # Get the BluetoothLEDevice from address - this should work since we're already connected
            device = await BluetoothLEDevice.from_bluetooth_address_async(mac_int)

            if device is None:
                print("  [BLE-Tune] 无法通过地址获取 WinRT 设备")
                return False

            print(f"  [BLE-Tune] 设备连接状态: {device.connection_status}")
            print("  [BLE-Tune] 正在请求高吞吐模式 (ThroughputOptimized)...")

            # Request ThroughputOptimized connection parameters
            # Note: pywinrt uses non-async method name
            result = device.request_preferred_connection_parameters(
                BluetoothLEPreferredConnectionParameters.throughput_optimized
            )

            # Status: 0=Unspecified, 1=Success, 2=DeviceNotConnected, 3=AccessDenied
            if result and result.status == 1:  # 1 = Success
                print("  [BLE-Tune] [OK] 成功切换到高吞吐模式! 等待 2 秒让参数生效...")
                import time as sync_time
                sync_time.sleep(2)  # Wait for parameter negotiation
                return True
            else:
                status_map = {0: "Unspecified", 1: "Success", 2: "DeviceNotConnected", 3: "AccessDenied"}
                status_name = status_map.get(result.status, f"Unknown({result.status})") if result else "NoResult"
                print(f"  [BLE-Tune] 请求返回状态: {status_name}")
                return False

        except Exception as e:
            print(f"  [BLE-Tune] 调优失败: {e}")
            import traceback
            traceback.print_exc()
            return False

    async def _connection_handler(self):
        device = await BleakScanner.find_device_by_address(self.address)
        print(device)
        try:
            async with BleakClient(device, disconnected_callback=self.handle_disconnect) as client:
                # Tune connection parameters for high throughput BEFORE starting notifications
                await self._tune_connection_for_throughput(client)

                await client.start_notify(READ_UUID, self.callback_function)
                self.status = 'Connected'
                while True:
                    try:
                        if len(self.output_buff) > 0:
                            print('write: ', self.output_buff)
                            await client.write_gatt_char(WRITE_UUID, self.output_buff.pop())
                        await asyncio.sleep(0.001)
                        if self.exit_flag:
                            print("Disconnecting")
                            await client.stop_notify(READ_UUID)
                            self.exit_flag = False
                            break
                    except asyncio.CancelledError:
                        print("Shutdown Request Received")
                        break
        except asyncio.CancelledError:
            print("Shutdown Request Received")

    def connect(self):
        notification_routine = self._connection_handler()
        connection_thread = Thread(target=asyncio.run, args=(notification_routine,), daemon=True)
        connection_thread.start()
        time.sleep(2)




def get_device_list(template):
    return asyncio.run(_get_device_list(template))


async def _get_device_list(filter):
    scanner = BleakScanner(filters={"DuplicateData":False})
    board_list = []
    points = '...'
    index = 3
    while not len(board_list):
        await scanner.start()
        await asyncio.sleep(2.0)
        await scanner.stop()
        devices = scanner.discovered_devices
        if len(devices):
            for b in devices:
                if isinstance(b.name, str) and filter in b.name:
                    board_list.append(b)
        if not len(board_list):
            print('No boards found{}   '.format(points[0:index]), end='\r', flush=True)
            index += 1
            if index > 3:
                index = 1
    return board_list


async def get_services(ble_address: str):
    device = await BleakScanner.find_device_by_address(ble_address, timeout=20.0)
    if not device:
        raise BleakError(f"A device with address {ble_address} could not be found.")
    async with BleakClient(device) as client:
        svcs = await client.get_services()
        return svcs


async def get_uart_uuid(ble_address: str):
    services = await get_services(ble_address)
    for service in services:
        if service.description == 'Nordic UART Service':
            service_uuid = service.uuid
            for characteristic in service.characteristics:
                if 'write' in characteristic.properties:
                    rx_uuid = characteristic.uuid
                if 'notify' in characteristic.properties:
                    tx_uuid = characteristic.uuid
            return service_uuid, rx_uuid, tx_uuid
    return None
