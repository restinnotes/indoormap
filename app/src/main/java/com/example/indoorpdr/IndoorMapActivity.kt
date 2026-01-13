package com.example.indoorpdr

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.*
import android.bluetooth.le.*
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
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
        // Updated MAC from logs: C4:4D:5E:E3:AD:D7
        const val SCS_MAC_ADDRESS = "C4:4D:5E:E3:AD:D7"
        const val SCAN_TIMEOUT_MS = 20000L
    }

    private lateinit var binding: ActivityIndoorMapBinding
    private lateinit var mapView: MapView
    private var aMap: AMap? = null

    // BLE
    private var bluetoothAdapter: BluetoothAdapter? = null
    private var bleScanner: BluetoothLeScanner? = null
    private var isScanning = false
    private val handler = Handler(Looper.getMainLooper())

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
        val permissions = mutableListOf<String>()

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            permissions.add(Manifest.permission.BLUETOOTH_SCAN)
            permissions.add(Manifest.permission.BLUETOOTH_CONNECT)
        }
        permissions.add(Manifest.permission.ACCESS_FINE_LOCATION)

        val needed = permissions.filter { checkSelfPermission(it) != PackageManager.PERMISSION_GRANTED }
        if (needed.isNotEmpty()) {
            ActivityCompat.requestPermissions(this, needed.toTypedArray(), REQUEST_BLE_PERMISSIONS)
        } else {
            initBle()
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
        bluetoothAdapter = bluetoothManager.adapter

        if (bluetoothAdapter == null || !bluetoothAdapter!!.isEnabled) {
            Toast.makeText(this, "Please enable Bluetooth", Toast.LENGTH_LONG).show()
            return
        }

        bleScanner = bluetoothAdapter?.bluetoothLeScanner
        if (bleScanner == null) {
            Toast.makeText(this, "BLE Scanner not available", Toast.LENGTH_LONG).show()
            return
        }

        Log.d(TAG, "Starting BLE scan for SCS device: $SCS_MAC_ADDRESS")
        Toast.makeText(this, "Scanning for SCS: $SCS_MAC_ADDRESS ...", Toast.LENGTH_SHORT).show()
        startBleScan()
    }

    @SuppressLint("MissingPermission")
    private fun startBleScan() {
        if (isScanning) return
        isScanning = true

        // Stop scan after timeout
        handler.postDelayed({
            stopBleScan()
            if (scsBleManager == null) {
                Log.w(TAG, "SCS device not found after scan timeout")
                Toast.makeText(this, "SCS not found. Using phone only.", Toast.LENGTH_LONG).show()
            }
        }, SCAN_TIMEOUT_MS)

        val settings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
            .build()

        // Optional: Filter by MAC address (some phones support, some don't)
        // val filter = ScanFilter.Builder().setDeviceAddress(SCS_MAC_ADDRESS).build()
        // bleScanner?.startScan(listOf(filter), settings, scanCallback)

        // Scan all devices and filter manually (more reliable)
        bleScanner?.startScan(null, settings, scanCallback)
        Log.d(TAG, "BLE scan started")
    }

    @SuppressLint("MissingPermission")
    private fun stopBleScan() {
        if (!isScanning) return
        isScanning = false
        bleScanner?.stopScan(scanCallback)
        Log.d(TAG, "BLE scan stopped")
    }

    // Flag to prevent double connections
    private var isConnecting = false

    private val scanCallback = object : ScanCallback() {
        @SuppressLint("MissingPermission")
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val device = result.device
            val mac = device.address.uppercase()
            val name = device.name ?: "Unknown"

            Log.v(TAG, "Scanned: $name [$mac] (Target: ${SCS_MAC_ADDRESS.uppercase()})")

            if ((mac == SCS_MAC_ADDRESS.uppercase() || name.contains("BAN Z-NODE", ignoreCase = true)) && !isConnecting) {
                isConnecting = true
                Log.d(TAG, ">>> Target SCS found: $name [$mac]! Connecting...")
                stopBleScan()
                connectToScs(device)
            }
        }

        override fun onScanFailed(errorCode: Int) {
            Log.e(TAG, "BLE scan failed: $errorCode")
            isScanning = false
        }
    }

    @SuppressLint("MissingPermission")
    private fun connectToScs(device: BluetoothDevice) {
        Toast.makeText(this, "Connecting to SCS: ${device.name ?: device.address}", Toast.LENGTH_SHORT).show()

        scsBleManager = ScsBleManager(this) { scsData ->
            // Feed quaternion to PDR Engine
            if (!scsData.isRaw) {
                pdrEngine?.setScsQuaternion(scsData.qx, scsData.qy, scsData.qz, scsData.qw)
                Log.v(TAG, "SCS Quat: qw=${scsData.qw}, qx=${scsData.qx}, qy=${scsData.qy}, qz=${scsData.qz}")
            }
        }
        scsBleManager?.connect(device)

        // Step 1: Send Leaf Enable Command (Protocol V3 Leaf Mode)
        handler.postDelayed({
            Log.d(TAG, "Sending Leaf Enable Command...")
            scsBleManager?.sendLeafEnable()
        }, 4000)

        // Step 2: Start Streaming after Config (allow 3 sec more)
        handler.postDelayed({
            scsBleManager?.startStreaming(ScsBleManager.TYPE_QUATERNION, 50)
            Log.d(TAG, "Started SCS Quaternion streaming @ 50Hz")
            Toast.makeText(this, "SCS streaming started!", Toast.LENGTH_SHORT).show()
        }, 7000)
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

    @SuppressLint("MissingPermission")
    override fun onDestroy() {
        super.onDestroy()
        stopBleScan()
        pdrEngine?.stop()
        mapView.onDestroy()
    }
}
