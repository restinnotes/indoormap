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

        override fun onDescriptorWrite(gatt: BluetoothGatt, descriptor: BluetoothGattDescriptor, status: Int) {
            Log.d(TAG, "onDescriptorWrite: uuid=${descriptor.uuid}, status=$status")
            if (status == BluetoothGatt.GATT_SUCCESS) {
                Log.i(TAG, "Notifications ENABLED successfully!")
            } else {
                Log.e(TAG, "Failed to enable notifications! status=$status")
            }
        }

        // DEPRECATED API (for older Android)
        @Deprecated("Deprecated in API 33")
        override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic) {
            Log.d(TAG, "onCharacteristicChanged (old API): size=${characteristic.value?.size}")
            characteristic.value?.let { parsePacket(it) }
        }

        // NEW API for Android 13+ (API 33+)
        override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic, value: ByteArray) {
            Log.d(TAG, "onCharacteristicChanged (new API): size=${value.size}")
            parsePacket(value)
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
     * Send Leaf Enable Command (Direct Connection Mode)
     * Replaces Central Config for direct phone-to-sensor connection
     */
    fun sendLeafEnable() {
        // Cmd: 0x32 0x04 0x01 0x00 0x00 0x00
        val cmd = ByteBuffer.allocate(6).order(ByteOrder.LITTLE_ENDIAN)
        cmd.put(0x32.toByte())
        cmd.put(0x04.toByte())
        cmd.put(0x01.toByte()) // Enable
        cmd.put(0x00.toByte())
        cmd.put(0x00.toByte())
        cmd.put(0x00.toByte())

        val bytes = cmd.array()
        Log.d(TAG, "Sending Leaf Enable (${bytes.size} bytes): ${bytes.contentToString()}")
        sendCommand(bytes)
    }

    /**
     * Start stream (Leaf Mode)
     * @param type 125 for Raw, 131 for Quat
     */
    fun startStreaming(type: Int, freq: Int = 50) {
        // Leaf Stream: 0x33 0x08 (2) + 5 zeros + Freq(1) + ID(1) + Mock(1) = 10 Bytes
        val cmd = ByteBuffer.allocate(10).order(ByteOrder.LITTLE_ENDIAN)
        cmd.put(0x33.toByte())
        cmd.put(0x08.toByte())

        cmd.put(ByteArray(5)) // Padding/Time

        cmd.put(freq.toByte())
        cmd.put(if (type == TYPE_QUATERNION) 0xF0.toByte() else 0x00.toByte())
        cmd.put(0x00.toByte()) // Mock = 0

        val bytes = cmd.array()
        Log.d(TAG, "Sending Leaf Stream (${bytes.size} bytes): ${bytes.contentToString()}")
        sendCommand(bytes)
    }

    private fun parsePacket(data: ByteArray) {
        if (data.isEmpty()) return
        val buffer = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN)
        val type = buffer.get(0).toInt() and 0xFF

        Log.d(TAG, "parsePacket: type=$type, size=${data.size}, raw=${data.take(20).map { it.toInt() and 0xFF }}")

        when (type) {
            TYPE_QUATERNION -> {
                if (data.size < 13) {
                    Log.w(TAG, "Quaternion packet too short: ${data.size}")
                    return
                }
                val ts = buffer.getShort(3).toLong() and 0xFFFF
                val qx = buffer.getShort(5) / 16384.0f
                val qy = buffer.getShort(7) / 16384.0f
                val qz = buffer.getShort(9) / 16384.0f
                val qw = buffer.getShort(11) / 16384.0f
                Log.d(TAG, "Parsed Quat: ts=$ts, qx=$qx, qy=$qy, qz=$qz, qw=$qw")
                dataCallback(ScsData(ts, qx, qy, qz, qw))
            }
            TYPE_RAW_DATA -> {
                if (data.size < 18) {
                    Log.w(TAG, "Raw packet too short: ${data.size}")
                    return
                }
                val ts = buffer.getInt(2).toLong()
                val ax = buffer.getShort(6) / (16384.0f * 2 / 16)
                val ay = buffer.getShort(8) / (16384.0f * 2 / 16)
                val az = buffer.getShort(10) / (16384.0f * 2 / 16)
                val gx = buffer.getShort(12) / (16384.0f * 2 / 4096)
                val gy = buffer.getShort(14) / (16384.0f * 2 / 4096)
                val gz = buffer.getShort(16) / (16384.0f * 2 / 4096)
                Log.d(TAG, "Parsed Raw: ts=$ts, ax=$ax, ay=$ay, az=$az")
                dataCallback(ScsData(ts, qx=0f, qy=0f, qz=0f, qw=0f, ax, ay, az, gx, gy, gz, isRaw=true))
            }
            else -> {
                Log.w(TAG, "Unknown packet type: $type (size=${data.size})")
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
