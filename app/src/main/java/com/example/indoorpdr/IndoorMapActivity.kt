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

class IndoorMapActivity : AppCompatActivity() {

    companion object {
        const val TAG = "IndoorMapActivity"
        const val REQUEST_BLE_PERMISSIONS = 1001
        const val SCS_MAC_ADDRESS = "C4:4D:5E:E3:AD:D7"
        const val SCAN_TIMEOUT_MS = 20000L
    }

    private lateinit var binding: ActivityIndoorMapBinding
    private lateinit var mapView: MapView
    private var aMap: AMap? = null

    private var bluetoothAdapter: BluetoothAdapter? = null
    private var bleScanner: BluetoothLeScanner? = null
    private var isScanning = false
    private val handler = Handler(Looper.getMainLooper())
    private var pdrEngine: HybridPdrEngine? = null
    private var scsBleManager: ScsBleManager? = null
    private val pdrLog = mutableListOf<String>()
    private val START_LAT_LNG = LatLng(39.9042, 116.4074)

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityIndoorMapBinding.inflate(layoutInflater)
        setContentView(binding.root)
        mapView = binding.mapView
        mapView.onCreate(savedInstanceState)
        initMap()
        requestBlePermissions()
        setupControls()
    }

    private fun requestBlePermissions() {
        val permissions = mutableListOf<String>()
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            permissions.add(Manifest.permission.BLUETOOTH_SCAN)
            permissions.add(Manifest.permission.BLUETOOTH_CONNECT)
        }
        permissions.add(Manifest.permission.ACCESS_FINE_LOCATION)
        val needed = permissions.filter { checkSelfPermission(it) != PackageManager.PERMISSION_GRANTED }
        if (needed.isNotEmpty()) { ActivityCompat.requestPermissions(this, needed.toTypedArray(), REQUEST_BLE_PERMISSIONS) }
        else { initBle() }
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == REQUEST_BLE_PERMISSIONS && grantResults.all { it == PackageManager.PERMISSION_GRANTED }) { initBle() }
    }

    @SuppressLint("MissingPermission")
    private fun initBle() {
        bluetoothAdapter = (getSystemService(BLUETOOTH_SERVICE) as BluetoothManager).adapter
        if (bluetoothAdapter == null || !bluetoothAdapter!!.isEnabled) { Toast.makeText(this, "Enable Bluetooth", Toast.LENGTH_LONG).show(); return }
        bleScanner = bluetoothAdapter?.bluetoothLeScanner
        if (bleScanner != null) startBleScan()
    }

    @SuppressLint("MissingPermission")
    private fun startBleScan() {
        if (isScanning) return; isScanning = true
        handler.postDelayed({ stopBleScan() }, SCAN_TIMEOUT_MS)
        bleScanner?.startScan(null, ScanSettings.Builder().setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build(), scanCallback)
    }

    @SuppressLint("MissingPermission")
    private fun stopBleScan() { if (!isScanning) return; isScanning = false; bleScanner?.stopScan(scanCallback) }

    private var isConnecting = false
    private val scanCallback = object : ScanCallback() {
        @SuppressLint("MissingPermission")
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val device = result.device; val mac = device.address.uppercase(); val name = device.name ?: ""
            if ((mac == SCS_MAC_ADDRESS.uppercase() || name.contains("BAN Z-NODE", true)) && !isConnecting) {
                isConnecting = true; stopBleScan(); connectToScs(device)
            }
        }
    }

    @SuppressLint("MissingPermission")
    private fun connectToScs(device: BluetoothDevice) {
        Toast.makeText(this, "Connecting SCS...", Toast.LENGTH_SHORT).show()
        scsBleManager = ScsBleManager(this) { scsData ->
            if (scsData.isRaw) pdrEngine?.setScsRawData(scsData.ax, scsData.ay, scsData.az, scsData.gx, scsData.gy, scsData.gz)
        }
        scsBleManager?.connect(device)
        handler.postDelayed({ scsBleManager?.sendCentralConfig(device.address) }, 2000)
        handler.postDelayed({ scsBleManager?.startStreamingCentral(ScsBleManager.TYPE_RAW_DATA, 50); Toast.makeText(this, "SCS Started", Toast.LENGTH_SHORT).show() }, 5000)
    }

    private fun setupControls() {
        binding.btnReset.setOnClickListener {
            binding.trajectoryView.clear(); pdrLog.clear()
            pdrLog.add("Timestamp,Type,Val1,Val2,Val3,Val4,Val5,Val6,Val7,Val8")
            binding.chartInput.clear(); binding.chartConfidence.clear(); binding.chartOutput.clear()
            Toast.makeText(this, "Cleared", Toast.LENGTH_SHORT).show()
        }
        binding.btnToggleSource.setOnClickListener {
            if (pdrEngine == null) return@setOnClickListener
            if (pdrEngine!!.swingSource == HybridPdrEngine.SwingSource.SCS) {
                pdrEngine!!.swingSource = HybridPdrEngine.SwingSource.PHONE
                binding.btnToggleSource.text = "Source: PHONE"
                binding.chartInput.init("Phone Accel", -20f, 20f)
            } else {
                pdrEngine!!.swingSource = HybridPdrEngine.SwingSource.SCS
                binding.btnToggleSource.text = "Source: SCS"
                binding.chartInput.init("SCS Accel", -20f, 20f)
            }
            binding.chartInput.clear()
        }
        binding.btnSave.setOnClickListener { saveToCsv() }
        pdrLog.add("Timestamp,Type,Val1,Val2,Val3,Val4,Val5,Val6,Val7,Val8")
    }

    private fun saveToCsv() {
        if (pdrLog.size <= 1) return
        try {
            val fileName = "PDR_Log_${System.currentTimeMillis()}.csv"
            val file = java.io.File(getExternalFilesDir(null), fileName)
            file.printWriter().use { out -> pdrLog.forEach { out.println(it) } }
            Toast.makeText(this, "Saved: ${file.name}", Toast.LENGTH_SHORT).show()
        } catch (e: Exception) { Log.e(TAG, "Save failed", e) }
    }

    private fun initMap() {
        if (aMap == null) aMap = mapView.map
        aMap?.let { map ->
            map.moveCamera(CameraUpdateFactory.newLatLngZoom(START_LAT_LNG, 18f))
            map.uiSettings.isScaleControlsEnabled = true
            val visualizer = PdrVisualizer(map)
            pdrEngine = HybridPdrEngine(this)

            // Charts: Input, Confidence (with Motion Variance), Heading (Degrees)
            binding.chartInput.init("Input Accel", -20f, 20f)
            binding.chartConfidence.init("Confidence & Motion", 0f, 10f)
            binding.chartConfidence.autoScale = true
            binding.chartOutput.init("Heading (Deg)", 0f, 360f) // Now in degrees!

            pdrEngine?.listener = object : HybridPdrEngine.PdrListener {
                override fun onRawData(ax: Float, ay: Float, az: Float) {
                    runOnUiThread {
                        binding.chartInput.addPoint("Ax", android.graphics.Color.RED, ax)
                        binding.chartInput.addPoint("Ay", android.graphics.Color.GREEN, ay)
                        binding.chartInput.addPoint("Az", android.graphics.Color.BLUE, az)
                    }
                    if (pdrLog.size < 100000) pdrLog.add("${System.currentTimeMillis()},RAW,$ax,$ay,$az,,,,,")
                }
                override fun onPositionUpdated(x: Double, y: Double, latLng: LatLng, stepCount: Int, headingDeg: Int, stepLength: Double, source: String) {
                    runOnUiThread { visualizer.updatePosition(latLng); binding.trajectoryView.addPoint(x, y) }
                    pdrLog.add("${System.currentTimeMillis()},PDR,$stepCount,$headingDeg,$x,$y,$stepLength,$source,,")
                }
                override fun onPhoneHeading(headingDeg: Float) {
                    runOnUiThread {
                         binding.chartOutput.addPoint("Phone", android.graphics.Color.WHITE, headingDeg)
                    }
                }
                override fun onDebugMessage(msg: String) {
                    runOnUiThread {
                        if (msg.startsWith("SWING_PLANE")) {
                            // Format: SWING_PLANE,nx,ny,nz,hx,hy,hz,rawQ,smoothQ,state,motionVar,headingDeg
                            val parts = msg.split(",")
                            if (parts.size >= 12) {
                                val rawQ = parts[7].toFloatOrNull() ?: 0f
                                val smoothQ = parts[8].toFloatOrNull() ?: 0f
                                val state = parts[9]
                                val motionVar = parts[10].toFloatOrNull() ?: 0f
                                val headingDeg = parts[11].toFloatOrNull() ?: 0f

                                // Chart 2: Confidence & Motion Variance
                                binding.chartConfidence.addPoint("Raw", android.graphics.Color.YELLOW, rawQ)
                                binding.chartConfidence.addPoint("Smooth", android.graphics.Color.GREEN, smoothQ)
                                binding.chartConfidence.addPoint("Motion", android.graphics.Color.MAGENTA, motionVar)
                                binding.chartConfidence.addPoint("Thresh", android.graphics.Color.RED, 1.5f)

                                // Chart 3: Heading in Degrees
                                binding.chartOutput.addPoint("Swing", android.graphics.Color.CYAN, headingDeg)
                                // binding.chartOutput.addPoint("Phone", android.graphics.Color.WHITE, phoneHeading) // TODO: Get phone heading

                                val isSwinging = state == "SWINGING"
                                binding.tvDebugInfo.text = "[$state] M:%.1f Q:%.1f H:%.0f°".format(motionVar, smoothQ, headingDeg)
                                binding.tvDebugInfo.setBackgroundColor(if (isSwinging) 0xAA00AA00.toInt() else 0xAA550000.toInt())

                                pdrLog.add("${System.currentTimeMillis()},SWING,$rawQ,$smoothQ,$headingDeg,$motionVar,${parts[1]},${parts[2]},${parts[3]},$state")
                            }
                        }
                    }
                }
            }
            pdrEngine?.start(START_LAT_LNG)
        }
    }

    override fun onResume() { super.onResume(); mapView.onResume() }
    override fun onPause() { super.onPause(); mapView.onPause() }
    override fun onSaveInstanceState(outState: Bundle) { super.onSaveInstanceState(outState); mapView.onSaveInstanceState(outState) }
    override fun onDestroy() { super.onDestroy(); stopBleScan(); pdrEngine?.stop(); mapView.onDestroy() }
}
