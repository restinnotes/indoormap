package com.example.scs.ble

import android.annotation.SuppressLint
import android.bluetooth.*
import android.content.Context
import android.util.Log
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.*

/**
 * SCS Sensor BLE Manager (Enhanced)
 * Supports both Quaternion and Raw IMU data.
 */
class ScsBleManager(private val context: Context, private val dataCallback: (ScsData) -> Unit) {

    companion object {
        const val TAG = "ScsBleManager"
        val SERVICE_UUID: UUID = UUID.fromString("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
        val WRITE_UUID: UUID = UUID.fromString("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")
        val READ_UUID: UUID = UUID.fromString("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")
        val CCD_UUID: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

        const val TYPE_QUATERNION = 131
        const val TYPE_RAW_DATA = 125
    }

    private var bluetoothGatt: BluetoothGatt? = null
    private var writeChar: BluetoothGattCharacteristic? = null

    @SuppressLint("MissingPermission")
    fun connect(device: BluetoothDevice) {
        bluetoothGatt = device.connectGatt(context, false, gattCallback)
    }

    @SuppressLint("MissingPermission")
    private val gattCallback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            Log.d(TAG, "onConnectionStateChange: status=$status, newState=$newState")
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                Log.i(TAG, "GATT Connected. Requesting MTU 512...")
                if (!gatt.requestMtu(512)) {
                    Log.e(TAG, "Request MTU failed! Proceeding anyway...")
                    gatt.discoverServices()
                }
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                Log.w(TAG, "GATT Disconnected.")
            }
        }

        override fun onMtuChanged(gatt: BluetoothGatt, mtu: Int, status: Int) {
            Log.d(TAG, "onMtuChanged: mtu=$mtu, status=$status")
            Log.i(TAG, "Discovering services...")
            gatt.discoverServices()
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            Log.d(TAG, "onServicesDiscovered: status=$status")
            if (status == BluetoothGatt.GATT_SUCCESS) {
                val service = gatt.getService(SERVICE_UUID)
                if (service != null) {
                    Log.i(TAG, "SCS Service Found!")
                    writeChar = service.getCharacteristic(WRITE_UUID)
                    service.getCharacteristic(READ_UUID)?.let {
                        Log.i(TAG, "Enabling Notifications...")
                        enableNotifications(gatt, it)
                    }
                } else {
                    Log.e(TAG, "SCS Service NOT found on this device!")
                }
            }
        }

        override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic) {
            // Log highly frequent data only if needed using verbose
            // Log.v(TAG, "onCharacteristicChanged: size=${characteristic.value.size}")
            parsePacket(characteristic.value)
        }
    }

    @SuppressLint("MissingPermission")
    private fun enableNotifications(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic) {
        gatt.setCharacteristicNotification(characteristic, true)
        val descriptor = characteristic.getDescriptor(CCD_UUID)
        descriptor.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
        gatt.writeDescriptor(descriptor)
    }

    @SuppressLint("MissingPermission")
    fun sendCommand(bytes: ByteArray) {
        writeChar?.value = bytes
        bluetoothGatt?.writeCharacteristic(writeChar)
    }

    /**
     * Send MAC configuration command (Central Config)
     * Must be called BEFORE startStreaming()
     * @param macAddress Device MAC in format "XX:XX:XX:XX:XX:XX"
     */
    fun sendConfigCommand(macAddress: String) {
        // Convert MAC string to bytes (reversed order as per Python SDK)
        val macBytes = macAddress.split(":").map { it.toInt(16).toByte() }.toByteArray().reversedArray()

        // Command: 0x00 0x38 + (MAC 6 bytes + Control 1 byte) * 8 slots
        // We fill slot 0, leave other 7 slots zeroed
        val cmd = ByteBuffer.allocate(58).order(ByteOrder.LITTLE_ENDIAN) // 2 + 7*8 = 58
        cmd.put(0x00.toByte())
        cmd.put(0x38.toByte())

        // Slot 0: MAC (6 bytes, reversed) + Control (1 byte)
        cmd.put(macBytes)
        cmd.put(0x01.toByte()) // Enable this device

        // Fill remaining 7 slots with zeros (7 bytes each)
        for (i in 0 until 7) {
            cmd.put(ByteArray(7))
        }

        val bytes = cmd.array()
        Log.d(TAG, "Sending Config Command (${bytes.size} bytes)")
        sendCommand(bytes)
    }

    /**
     * Start stream based on type
     * @param type 125 for Raw, 131 for Quat
     */
    fun startStreaming(type: Int, freq: Int = 50) {
        // Corrected based on Python _scs_reference.py
        // 2 (Op) + 4 (Time) + 4 (Flags) + 1 (Freq) + 1 (ID) + 1 (Mock) + 1 (Pad) = 14 Bytes
        val cmd = ByteBuffer.allocate(14).order(ByteOrder.LITTLE_ENDIAN)
        cmd.put(0x19.toByte())
        cmd.put(0x0C.toByte())

        // 8 Bytes of "Padding" (Time + Flags)
        cmd.put(ByteArray(8))

        cmd.put(freq.toByte())
        cmd.put(if (type == TYPE_QUATERNION) 0xF0.toByte() else 0x00.toByte())
        cmd.put(0x00.toByte()) // Real
        cmd.put(0x00.toByte())

        val bytes = cmd.array()
        Log.d(TAG, "Sending Stream Command (${bytes.size} bytes): ${bytes.contentToString()}")
        sendCommand(bytes)
    }

    private fun parsePacket(data: ByteArray) {
        if (data.isEmpty()) return
        val buffer = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN)
        val type = buffer.get(0).toInt() and 0xFF

        when (type) {
            TYPE_QUATERNION -> {
                val ts = buffer.getShort(3).toLong() and 0xFFFF
                val qx = buffer.getShort(5) / 16384.0f
                val qy = buffer.getShort(7) / 16384.0f
                val qz = buffer.getShort(9) / 16384.0f
                val qw = buffer.getShort(11) / 16384.0f
                Log.d(TAG, "Parsed Quat: ts=$ts, qx=$qx, qy=$qy, qz=$qz, qw=$qw")
                dataCallback(ScsData(ts, qx, qy, qz, qw))
            }
            TYPE_RAW_DATA -> {
                // [Type 1] [Padding 1] [TS 4] [ax 2] [ay 2] [az 2] [gx 2] [gy 2] [gz 2]
                val ts = buffer.getInt(2).toLong()
                val ax = buffer.getShort(6) / (16384.0f * 2 / 16)
                val ay = buffer.getShort(8) / (16384.0f * 2 / 16)
                val az = buffer.getShort(10) / (16384.0f * 2 / 16)
                val gx = buffer.getShort(12) / (16384.0f * 2 / 4096)
                val gy = buffer.getShort(14) / (16384.0f * 2 / 4096)
                val gz = buffer.getShort(16) / (16384.0f * 2 / 4096)
                dataCallback(ScsData(ts, qx=0f, qy=0f, qz=0f, qw=0f, ax, ay, az, gx, gy, gz, isRaw=true))
            }
        }
    }
}

data class ScsData(
    val timestamp: Long,
    val qx: Float, val qy: Float, val qz: Float, val qw: Float,
    val ax: Float = 0f, val ay: Float = 0f, val az: Float = 0f,
    val gx: Float = 0f, val gy: Float = 0f, val gz: Float = 0f,
    val isRaw: Boolean = false
)
