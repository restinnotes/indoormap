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
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                gatt.discoverServices()
            }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            val service = gatt.getService(SERVICE_UUID)
            writeChar = service?.getCharacteristic(WRITE_UUID)
            service?.getCharacteristic(READ_UUID)?.let { enableNotifications(gatt, it) }
        }

        override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic) {
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
     * Start stream based on type
     * @param type 125 for Raw, 131 for Quat
     */
    fun startStreaming(type: Int, freq: Int = 50) {
        val cmd = ByteBuffer.allocate(13).order(ByteOrder.LITTLE_ENDIAN)
        cmd.put(0x19.toByte())
        cmd.put(0x0C.toByte())
        cmd.put(ByteArray(5)) // Padding
        cmd.put(freq.toByte())
        cmd.put(if (type == TYPE_QUATERNION) 0xF0.toByte() else 0x00.toByte())
        cmd.put(0x00.toByte()) // Real
        cmd.put(0x00.toByte())
        sendCommand(cmd.array())
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
