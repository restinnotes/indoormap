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
            // Feed Raw Data to PDR Engine for Swing Plane Detection
            if (scsData.isRaw) {
                // Plot Input Data (Raw Accel)
                runOnUiThread {
                    binding.chartInput.addPoint("Ax", android.graphics.Color.RED, scsData.ax)
                    binding.chartInput.addPoint("Ay", android.graphics.Color.GREEN, scsData.ay)
                    binding.chartInput.addPoint("Az", android.graphics.Color.BLUE, scsData.az)
                }

                pdrEngine?.setScsRawData(
                    scsData.ax, scsData.ay, scsData.az,
                    scsData.gx, scsData.gy, scsData.gz
                )
            } else {
                // ...
            }
        }
        scsBleManager?.connect(device)

        // Step 1: Send Central Configuration
        handler.postDelayed({
            Log.d(TAG, "Sending Central Config (MAC: ${device.address})...")
            scsBleManager?.sendCentralConfig(device.address)
        }, 2000)

        // Step 2: Start Streaming RAW DATA (Type 125)
        handler.postDelayed({
            Log.d(TAG, "Starting SCS RAW stream (Central Mode @ 50Hz)...")
            // CHANGE: Request TYPE_RAW_DATA (125) instead of QUATERNION
            scsBleManager?.startStreamingCentral(ScsBleManager.TYPE_RAW_DATA, 50)
            Toast.makeText(this, "SCS Raw Stream Started!", Toast.LENGTH_SHORT).show()
        }, 5000)
    }

    // ...

    private fun initMap() {
        if (aMap == null) {
            aMap = mapView.map
        }

        aMap?.let { map ->
            map.moveCamera(CameraUpdateFactory.newLatLngZoom(START_LAT_LNG, 18f))
            map.uiSettings.isScaleControlsEnabled = true

            val visualizer = PdrVisualizer(map)
            pdrEngine = HybridPdrEngine(this)

            // Initialize Charts
            binding.chartInput.init("Input Accel (m/s^2)", -20f, 20f)
            binding.chartConfidence.init("Swing Confidence", 0f, 10f)
            binding.chartOutput.init("Swing Heading (Rad)", -3.14159f, 3.14159f) // -PI to PI

            pdrEngine?.listener = object : HybridPdrEngine.PdrListener {
                override fun onPositionUpdated(x: Double, y: Double, latLng: LatLng, stepCount: Int, headingDeg: Int, stepLength: Double, source: String) {
                    runOnUiThread {
                        visualizer.updatePosition(latLng)
                        binding.trajectoryView.addPoint(x, y)
                        // Binding Debug Info text is handled below via Swing updates for now
                        // pdrLog.add(...) - Keep if needed
                    }
                }

                override fun onDebugMessage(msg: String) {
                    runOnUiThread {
                        if (msg.startsWith("SWING_PLANE")) {
                            // Format: SWING_PLANE,nx,ny,nz,hx,hy,hz,quality
                            val parts = msg.split(",")
                            if (parts.size >= 8) {
                                val quality = parts[7].toFloat()
                                val hx = parts[4].toFloat()
                                val hy = parts[5].toFloat()

                                // Plot 2: Confidence
                                binding.chartConfidence.addPoint("Score", android.graphics.Color.GREEN, quality)
                                binding.chartConfidence.addPoint("Threshold", android.graphics.Color.RED, 4.0f) // Ref line

                                // Plot 3: Heading (Just X component for now or Atan2)
                                // Let's plot the computed Heading Angle (Rad)
                                val headingRad = Math.atan2(hy.toDouble(), hx.toDouble()).toFloat()
                                binding.chartOutput.addPoint("SwingHeading", android.graphics.Color.CYAN, headingRad)

                                // Update Text
                                binding.tvDebugInfo.text = "Swing Score: %.1f\nHeading: %.2f".format(quality, headingRad)
                                if (quality > 4.0f) binding.tvDebugInfo.setBackgroundColor(0xAA00AA00.toInt())
                                else binding.tvDebugInfo.setBackgroundColor(0xAA550000.toInt())
                            }
                        } else {
                             Log.d(TAG, msg)
                        }
                    }
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
