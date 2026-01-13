package com.example.indoorpdr

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.util.Log
import com.amap.api.maps.model.LatLng
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

class PdrEngine(private val context: Context) : SensorEventListener {

    interface PdrListener {
        // Updated: Pass X, Y (meters) in addition to LatLng
        fun onPdrPositionUpdated(x: Double, y: Double, currentLocation: LatLng, stepCount: Int, heading: Float)
        fun onDebugMessage(msg: String)
    }

    var listener: PdrListener? = null

    private var sensorManager: SensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager

    // State
    private var isRunning = false
    private var stepCount = 0

    // Position (Meters relative to start)
    private var currentX = 0.0
    private var currentY = 0.0

    // Position (Lat/Lng)
    private var currentLat = 0.0
    private var currentLng = 0.0


    // Step Detection Config
    private val STEP_THRESHOLD = 11.0f // Magnitude (Gravity ~ 9.8)
    private val MIN_STEP_DELAY_MS = 300L
    private var lastStepTime = 0L

    // Heading Config
    private var currentHeadingRad = 0.0f
    private var enableHdr = true // "Square Mode" (Heading Rectification)

    // Constants
    private val EARTH_RADIUS = 6378137.0
    private val STEP_LENGTH = 0.7 // Meters (Fixed for simplicity)

    fun start(startPoint: LatLng) {
        if (isRunning) return

        // Reset Logic
        currentX = 0.0
        currentY = 0.0

        currentLat = startPoint.latitude
        currentLng = startPoint.longitude
        stepCount = 0

        // Register Sensors
        val accel = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        val rotation = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) // Best for Orientation

        if (accel != null) sensorManager.registerListener(this, accel, SensorManager.SENSOR_DELAY_GAME)
        if (rotation != null) sensorManager.registerListener(this, rotation, SensorManager.SENSOR_DELAY_GAME)

        isRunning = true
        listener?.onDebugMessage("PDR Started. Sensors Registered. Walk to test.")
    }

    fun stop() {
        if (!isRunning) return
        sensorManager.unregisterListener(this)
        isRunning = false
    }

    override fun onSensorChanged(event: SensorEvent?) {
        if (event == null) return

        when (event.sensor.type) {
            Sensor.TYPE_ROTATION_VECTOR -> handleRotation(event)
            Sensor.TYPE_ACCELEROMETER -> handleAccel(event)
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    private fun handleRotation(event: SensorEvent) {
        val rotationMatrix = FloatArray(9)
        SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values)
        val orientationAngles = FloatArray(3)
        SensorManager.getOrientation(rotationMatrix, orientationAngles)

        // orientationAngles[0] is Azimuth (Rotation around Z-axis) in Radians
        // Range: -PI to PI
        // 0 = North, PI/2 = East, PI = South, -PI/2 = West
        val rawAzimuth = orientationAngles[0]

        // HDR Logic: Snap to nearest PI/2 (90 degrees) if enabled
        currentHeadingRad = if (enableHdr) {
            snapToGrid(rawAzimuth)
        } else {
            rawAzimuth
        }
    }

    private fun snapToGrid(roughRad: Float): Float {
        // Normalize to 0..2PI for easier math, or just handle the quadrants
        // Simple snapping: Find nearest k * (PI/2)
        val step = Math.PI / 2.0
        val multiple = Math.round(roughRad / step)
        val snapped = multiple * step

        // Soft snap? or Hard snap?
        // For strict "Square" walking, Hard Snap is very effective if the user aligns well.
        // Let's allow a "Tolerance" zone.
        val diff = Math.abs(roughRad - snapped)
        return if (diff < 0.35) { // ~20 degrees tolerance
             snapped.toFloat()
        } else {
             roughRad
        }
    }

    private fun handleAccel(event: SensorEvent) {
        val x = event.values[0]
        val y = event.values[1]
        val z = event.values[2]

        // Simple Magnitude Peak Detection
        val magnitude = sqrt((x*x + y*y + z*z).toDouble())

        // NOTE: Real-world Step detection needs Low-Pass Filter (Gravity Removal) + Dynamic Threshold
        // For this demo: Raw Magnitude > Threshold is "Good Enough" for distinct walking
        // Gravity is ~9.8. Mid-step peak is often > 11 or 12.

        if (magnitude > STEP_THRESHOLD) {
            val now = System.currentTimeMillis()
            if (now - lastStepTime > MIN_STEP_DELAY_MS) {
                // Step Detected!
                lastStepTime = now
                stepCount++

                updateLocation()
            }
        }
    }

    private fun updateLocation() {
        // Delta in Meters
        // Azimuth 0 (North) -> +Y axis in Cartesian?
        // Let's align: North = +Y, East = +X
        // Heading is Azimuth (0=N, PI/2=E).
        // math.sin(0)=0, cos(0)=1 -> +Y. Correct.
        // math.sin(PI/2)=1, cos(PI/2)=0 -> +X. Correct.

        val dY = STEP_LENGTH * cos(currentHeadingRad) // North
        val dX = STEP_LENGTH * sin(currentHeadingRad) // East

        currentY += dY
        currentX += dX

        // Update LatLng (approximate) for Map compatibility
        val dLat = (dY / EARTH_RADIUS) * (180.0 / Math.PI)
        val dLng = (dX / (EARTH_RADIUS * cos(currentLat * Math.PI / 180.0))) * (180.0 / Math.PI)

        currentLat += dLat
        currentLng += dLng

        val newPos = LatLng(currentLat, currentLng)
        listener?.onPdrPositionUpdated(currentX, currentY, newPos, stepCount, currentHeadingRad)
    }
}
