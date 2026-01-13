package com.example.indoorpdr

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.util.Log
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import com.amap.api.maps.AMap
import com.amap.api.maps.CameraUpdateFactory
import com.amap.api.maps.MapView
import com.amap.api.maps.model.LatLng
import com.example.indoorpdr.databinding.ActivityIndoorMapBinding
import com.example.scs.ble.ScsBleManager
import com.example.scs.ble.ScsData

class IndoorMapActivity : AppCompatActivity() {

    companion object {
        const val TAG = "IndoorMapActivity"
        const val REQUEST_BLE_PERMISSIONS = 1001
        // SCS Sensor MAC Address - Update this to your device!
        const val SCS_MAC_ADDRESS = "E3:AD:D7:5E:4D:C4" // Example: right_forearm from settings
    }

    private lateinit var binding: ActivityIndoorMapBinding
    private lateinit var mapView: MapView
    private var aMap: AMap? = null

    // Hybrid PDR
    private var pdrEngine: HybridPdrEngine? = null

    // SCS BLE
    private var scsBleManager: ScsBleManager? = null

    // For Logging
    private val pdrLog = mutableListOf<String>()

    // Configuration
    private val START_LAT_LNG = LatLng(39.9042, 116.4074)

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        binding = ActivityIndoorMapBinding.inflate(layoutInflater)
        setContentView(binding.root)

        mapView = binding.mapView
        mapView.onCreate(savedInstanceState)

        initMap()
        setupControls()
        requestBlePermissions()
    }

    private fun requestBlePermissions() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            val permissions = arrayOf(
                Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.BLUETOOTH_CONNECT,
                Manifest.permission.ACCESS_FINE_LOCATION
            )
            if (permissions.any { checkSelfPermission(it) != PackageManager.PERMISSION_GRANTED }) {
                ActivityCompat.requestPermissions(this, permissions, REQUEST_BLE_PERMISSIONS)
            } else {
                initBle()
            }
        } else {
            if (checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
                ActivityCompat.requestPermissions(this, arrayOf(Manifest.permission.ACCESS_FINE_LOCATION), REQUEST_BLE_PERMISSIONS)
            } else {
                initBle()
            }
        }
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == REQUEST_BLE_PERMISSIONS && grantResults.all { it == PackageManager.PERMISSION_GRANTED }) {
            initBle()
        } else {
            Toast.makeText(this, "BLE permissions denied. Using phone sensors only.", Toast.LENGTH_LONG).show()
        }
    }

    @SuppressLint("MissingPermission")
    private fun initBle() {
        val bluetoothManager = getSystemService(BLUETOOTH_SERVICE) as BluetoothManager
        val adapter = bluetoothManager.adapter

        if (adapter == null || !adapter.isEnabled) {
            Toast.makeText(this, "Please enable Bluetooth", Toast.LENGTH_LONG).show()
            return
        }

        // Find the SCS device
        val device: BluetoothDevice? = adapter.bondedDevices.find { it.address == SCS_MAC_ADDRESS }

        if (device != null) {
            scsBleManager = ScsBleManager(this) { scsData ->
                // Feed quaternion to PDR Engine
                if (!scsData.isRaw) {
                    pdrEngine?.setScsQuaternion(scsData.qx, scsData.qy, scsData.qz, scsData.qw)
                }
            }
            scsBleManager?.connect(device)
            Toast.makeText(this, "Connecting to SCS: ${device.name}", Toast.LENGTH_SHORT).show()

            // Start streaming Quaternion after a delay (allow GATT discovery)
            binding.root.postDelayed({
                scsBleManager?.startStreaming(ScsBleManager.TYPE_QUATERNION, 50)
                Log.d(TAG, "Started SCS Quaternion streaming @ 50Hz")
            }, 3000)
        } else {
            Toast.makeText(this, "SCS not paired. Using phone only. Pair '$SCS_MAC_ADDRESS' in Bluetooth settings.", Toast.LENGTH_LONG).show()
        }
    }

    private fun setupControls() {
        binding.btnReset.setOnClickListener {
            binding.trajectoryView.clear()
            pdrLog.clear()
            pdrLog.add("Timestamp,Steps,HeadingDeg,X,Y,StepLen,Source")
            Toast.makeText(this, "Cleared", Toast.LENGTH_SHORT).show()
        }

        binding.btnSave.setOnClickListener {
            saveToCsv()
        }

        pdrLog.add("Timestamp,Steps,HeadingDeg,X,Y,StepLen,Source")
    }

    private fun saveToCsv() {
        if (pdrLog.size <= 1) {
            Toast.makeText(this, "No data to save", Toast.LENGTH_SHORT).show()
            return
        }

        try {
            val fileName = "PDR_Log_${System.currentTimeMillis()}.csv"
            val fileContents = pdrLog.joinToString("\n")
            val file = java.io.File(getExternalFilesDir(null), fileName)
            file.writeText(fileContents)
            Toast.makeText(this, "Saved to: ${file.name}", Toast.LENGTH_LONG).show()
            Log.d(TAG, "Log saved: ${file.absolutePath}")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to save CSV", e)
            Toast.makeText(this, "Save Failed: ${e.message}", Toast.LENGTH_SHORT).show()
        }
    }

    private fun initMap() {
        if (aMap == null) {
            aMap = mapView.map
        }

        aMap?.let { map ->
            map.moveCamera(CameraUpdateFactory.newLatLngZoom(START_LAT_LNG, 18f))
            map.uiSettings.isScaleControlsEnabled = true

            val visualizer = PdrVisualizer(map)
            pdrEngine = HybridPdrEngine(this)

            pdrEngine?.listener = object : HybridPdrEngine.PdrListener {
                override fun onPositionUpdated(x: Double, y: Double, latLng: LatLng, stepCount: Int, headingDeg: Int, stepLength: Double, source: String) {
                    runOnUiThread {
                        visualizer.updatePosition(latLng)
                        binding.trajectoryView.addPoint(x, y)

                        // Show source (Phone or SCS) in debug info
                        binding.tvDebugInfo.text = "[$source] Steps: $stepCount | Heading: $headingDeg° | X: ${"%.1f".format(x)} Y: ${"%.1f".format(y)}"

                        pdrLog.add("${System.currentTimeMillis()},$stepCount,$headingDeg,$x,$y,${"%.2f".format(stepLength)},$source")
                    }
                }

                override fun onDebugMessage(msg: String) {
                    runOnUiThread { Log.d(TAG, msg) }
                }
            }

            pdrEngine?.start(START_LAT_LNG)
        }
    }

    override fun onResume() {
        super.onResume()
        mapView.onResume()
    }

    override fun onPause() {
        super.onPause()
        mapView.onPause()
    }

    override fun onSaveInstanceState(outState: Bundle) {
        super.onSaveInstanceState(outState)
        mapView.onSaveInstanceState(outState)
    }

    override fun onDestroy() {
        super.onDestroy()
        pdrEngine?.stop()
        mapView.onDestroy()
    }
}
