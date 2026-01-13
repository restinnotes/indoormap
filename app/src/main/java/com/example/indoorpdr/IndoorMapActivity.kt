package com.example.indoorpdr

import android.content.Context
import android.graphics.BitmapFactory
import android.os.Bundle
import android.util.Log
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import com.amap.api.maps.AMap
import com.amap.api.maps.CameraUpdateFactory
import com.amap.api.maps.MapView
import com.amap.api.maps.model.BitmapDescriptorFactory
import com.amap.api.maps.model.GroundOverlayOptions
import com.amap.api.maps.model.LatLng
import com.amap.api.maps.model.LatLngBounds
import com.example.indoorpdr.databinding.ActivityIndoorMapBinding

class IndoorMapActivity : AppCompatActivity() {

    private lateinit var binding: ActivityIndoorMapBinding
    private lateinit var mapView: MapView
    private var aMap: AMap? = null

    // Configuration: Starting Point (e.g., A randomly chosen lab location)
    // Replace these with your actual location or the location of your floor plan
    private val START_LAT_LNG = LatLng(39.9042, 116.4074) // Beijing placeholder

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        binding = ActivityIndoorMapBinding.inflate(layoutInflater)
        setContentView(binding.root)

        mapView = binding.mapView
        mapView.onCreate(savedInstanceState) // Must call this

        initMap()
    }

    private fun initMap() {
        if (aMap == null) {
            aMap = mapView.map
        }

        aMap?.let { map ->
            // 1. Move camera to start position
            map.moveCamera(CameraUpdateFactory.newLatLngZoom(START_LAT_LNG, 18f))

            // 2. Setup Ground Overlay (The Floor Plan)
            setupGroundOverlay(map)

            // 3. UI Tweaks
            map.uiSettings.isScaleControlsEnabled = true

            // 4. Initialize Visualizer
            val visualizer = PdrVisualizer(map)

            // 5. Start PDR Engine (Internal Sensors)
            val pdrEngine = PdrEngine(this)
            pdrEngine.listener = object : PdrEngine.PdrListener {
                override fun onPdrPositionUpdated(x: Double, y: Double, currentLocation: LatLng, stepCount: Int, heading: Float) {
                    runOnUiThread {
                        // Update Map Visualizer (Still kept in case we switch back)
                        visualizer.updatePosition(currentLocation)

                        // Update Trajectory View (Whiteboard)
                        val trajectoryView = findViewById<TrajectoryView>(R.id.trajectory_view)
                        trajectoryView.addPoint(x, y)

                        val angleDeg = Math.toDegrees(heading.toDouble()).toInt()
                        binding.tvDebugInfo.text = "Steps: $stepCount | Heading: $angleDeg° | X: ${"%.1f".format(x)} Y: ${"%.1f".format(y)}"
                    }
                }

                override fun onDebugMessage(msg: String) {
                    runOnUiThread {
                        // Toast.makeText(this@IndoorMapActivity, msg, Toast.LENGTH_SHORT).show()
                    }
                }
            }

            // Start PDR (Assuming Permissions are granted! In real app, check permissions first)
            // For this quick demo, we assume the user grants it or we manually grant in settings.
            pdrEngine.start(START_LAT_LNG)

        }
    }

    private fun setupGroundOverlay(map: AMap) {
        // Try to load 'floor_plan.png' from assets or drawable
        // For this demo, we assume there's a resource drawable or asset.
        // If not found, we just toast.

        try {
            // Using a resource ID approach for safety if we had one.
            // Since we don't have the file yet, we'll log it.
            // val descriptor = BitmapDescriptorFactory.fromResource(R.drawable.floor_plan)

            // Or from Assets
            // val image = BitmapFactory.decodeStream(assets.open("floor_plan.png"))
            // val descriptor = BitmapDescriptorFactory.fromBitmap(image)

            // Placeholder: Add a dummy overlay just to prove code works if image existed
            // map.addGroundOverlay(GroundOverlayOptions()...)

            binding.tvDebugInfo.text = "Map Ready. Waiting for floor_plan.png..."

        } catch (e: Exception) {
            Log.e("IndoorPDR", "Error loading overlay: ${e.message}")
            Toast.makeText(this, "Could not load floor plan image", Toast.LENGTH_SHORT).show()
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
