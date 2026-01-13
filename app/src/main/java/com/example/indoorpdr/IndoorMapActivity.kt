package com.example.indoorpdr

import android.content.Context
import android.os.Bundle
import android.util.Log
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import com.amap.api.maps.AMap
import com.amap.api.maps.CameraUpdateFactory
import com.amap.api.maps.MapView
import com.amap.api.maps.model.LatLng
import com.example.indoorpdr.databinding.ActivityIndoorMapBinding

class IndoorMapActivity : AppCompatActivity() {

    private lateinit var binding: ActivityIndoorMapBinding
    private lateinit var mapView: MapView
    private var aMap: AMap? = null

    // For Logging
    private val pdrLog = mutableListOf<String>()

    // Configuration: Starting Point
    private val START_LAT_LNG = LatLng(39.9042, 116.4074)

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        binding = ActivityIndoorMapBinding.inflate(layoutInflater)
        setContentView(binding.root)

        mapView = binding.mapView
        mapView.onCreate(savedInstanceState)

        initMap()
        setupControls()
    }

    private fun setupControls() {
        binding.btnReset.setOnClickListener {
            binding.trajectoryView.clear()
            pdrLog.clear()
            pdrLog.add("Timestamp,Steps,HeadingDeg,X,Y")
            Toast.makeText(this, "Cleared", Toast.LENGTH_SHORT).show()
        }

        binding.btnSave.setOnClickListener {
            saveToCsv()
        }

        // Initial header
        pdrLog.add("Timestamp,Steps,HeadingDeg,X,Y")
    }

    private fun saveToCsv() {
        if (pdrLog.size <= 1) {
            Toast.makeText(this, "No data to save", Toast.LENGTH_SHORT).show()
            return
        }

        try {
            val fileName = "PDR_Log_${System.currentTimeMillis()}.csv"
            val fileContents = pdrLog.joinToString("\n")

            // Save to app's external files directory
            val file = java.io.File(getExternalFilesDir(null), fileName)
            file.writeText(fileContents)

            Toast.makeText(this, "Saved to: ${file.name}", Toast.LENGTH_LONG).show()
            Log.d("IndoorPDR", "Log saved: ${file.absolutePath}")

        } catch (e: Exception) {
            Log.e("IndoorPDR", "Failed to save CSV", e)
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
            val pdrEngine = PdrEngine(this)

            pdrEngine.listener = object : PdrEngine.PdrListener {
                override fun onPdrPositionUpdated(x: Double, y: Double, currentLocation: LatLng, stepCount: Int, heading: Float) {
                    runOnUiThread {
                        visualizer.updatePosition(currentLocation)
                        binding.trajectoryView.addPoint(x, y)

                        val angleDeg = Math.toDegrees(heading.toDouble()).toInt()
                        binding.tvDebugInfo.text = "Steps: $stepCount | Heading: $angleDeg° | X: ${"%.1f".format(x)} Y: ${"%.1f".format(y)}"

                        pdrLog.add("${System.currentTimeMillis()},$stepCount,$angleDeg,$x,$y")
                    }
                }

                override fun onDebugMessage(msg: String) {
                    runOnUiThread {
                        Log.d("PDR", msg)
                    }
                }
            }

            pdrEngine.start(START_LAT_LNG)
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
        mapView.onDestroy()
    }
}
