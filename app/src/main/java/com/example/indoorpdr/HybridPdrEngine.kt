package com.example.indoorpdr

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.util.Log
import com.amap.api.maps.model.LatLng
import com.example.scs.math.QuaternionUtils
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Hybrid PDR Engine
 * - Step Detection: from Phone Accelerometer
 * - Heading: from SCS Quaternion (if connected), fallback to Phone Rotation Vector
 */
class HybridPdrEngine(private val context: Context) : SensorEventListener {

    interface PdrListener {
        fun onPositionUpdated(x: Double, y: Double, latLng: LatLng, stepCount: Int, headingDeg: Int, stepLength: Double, source: String)
        fun onDebugMessage(msg: String)
    }

    var listener: PdrListener? = null

    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager

    // State
    private var isRunning = false
    private var stepCount = 0
    private var currentX = 0.0
    private var currentY = 0.0
    private var currentLat = 0.0
    private var currentLng = 0.0

    // Step Detection
    private val STEP_THRESHOLD = 11.5f
    private val MIN_STEP_DELAY_MS = 280L
    private var lastStepTime = 0L

    // Weinberg Step Length
    private var WEINBERG_K = 0.42
    private var currentStepMaxAccel = 0.0
    private var currentStepMinAccel = 100.0

    // Heading (Internal)
    private var phoneHeadingRad = 0.0f

    // Heading (External SCS) - Set from outside via setScsHeading()
    private var scsHeadingRad: Float? = null
    private var lastScsUpdateTime = 0L

    // HDR - Enhanced with 45° diagonal support
    var useHdr = true
    private val HDR_TOLERANCE_RAD = 0.26f // ~15 degrees

    // Direction Lock - reduces jitter when walking consistently
    private var lockedHeadingRad: Float? = null
    private var consecutiveDirectionCount = 0
    private val DIRECTION_LOCK_THRESHOLD = 2  // Lock after 2 consistent steps (was 3)
    private val DIRECTION_UNLOCK_TOLERANCE = 0.79f // ~45 degrees to unlock (was 30)

    // Step Length Smoothing
    private val stepLengthHistory = mutableListOf<Double>()
    private val MAX_STEP_LENGTH = 0.75
    private val MIN_STEP_LENGTH = 0.55
    private val STEP_HISTORY_SIZE = 5

    // Constants
    private val EARTH_RADIUS = 6378137.0

    fun start(startPoint: LatLng) {
        if (isRunning) return

        currentX = 0.0
        currentY = 0.0
        currentLat = startPoint.latitude
        currentLng = startPoint.longitude
        stepCount = 0

        // Reset enhanced PDR state
        lockedHeadingRad = null
        consecutiveDirectionCount = 0
        stepLengthHistory.clear()

        val accel = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        val rotation = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)

        if (accel != null) sensorManager.registerListener(this, accel, SensorManager.SENSOR_DELAY_GAME)
        if (rotation != null) sensorManager.registerListener(this, rotation, SensorManager.SENSOR_DELAY_GAME)

        isRunning = true
        listener?.onDebugMessage("Hybrid PDR Started. Walk to test.")
    }

    fun stop() {
        if (!isRunning) return
        sensorManager.unregisterListener(this)
        isRunning = false
    }

    /**
     * Call this from BLE callback when new SCS quaternion arrives.
     */
    fun setScsQuaternion(qx: Float, qy: Float, qz: Float, qw: Float) {
        val q = QuaternionUtils.Quaternion(qx, qy, qz, qw)
        val euler = QuaternionUtils.toEuler(q)
        scsHeadingRad = euler.z // Yaw
        lastScsUpdateTime = System.currentTimeMillis()
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
        phoneHeadingRad = orientationAngles[0]
    }

    private fun handleAccel(event: SensorEvent) {
        val x = event.values[0]
        val y = event.values[1]
        val z = event.values[2]
        val magnitude = sqrt((x * x + y * y + z * z).toDouble())

        // Track peaks/valleys for Weinberg
        if (magnitude > currentStepMaxAccel) currentStepMaxAccel = magnitude
        if (magnitude < currentStepMinAccel) currentStepMinAccel = magnitude

        if (magnitude > STEP_THRESHOLD) {
            val now = System.currentTimeMillis()
            if (now - lastStepTime > MIN_STEP_DELAY_MS) {
                // Step Detected!
                lastStepTime = now
                stepCount++

                // Weinberg Step Length with smoothing
                val accelRange = (currentStepMaxAccel - currentStepMinAccel).coerceAtLeast(1.0)
                val rawStepLength = WEINBERG_K * Math.pow(accelRange, 0.25)
                val stepLength = smoothStepLength(rawStepLength)

                // Reset for next step
                currentStepMaxAccel = 0.0
                currentStepMinAccel = 100.0

                // Determine Heading Source
                val headingSource: String
                val rawHeadingRad: Float

                val scsAge = now - lastScsUpdateTime
                if (scsHeadingRad != null && scsAge < 500) {
                    // SCS data is fresh (< 500ms old)
                    rawHeadingRad = scsHeadingRad!!
                    headingSource = "SCS"
                } else {
                    rawHeadingRad = phoneHeadingRad
                    headingSource = "Phone"
                }

                // Apply HDR if enabled, then direction lock
                val snappedHeading = if (useHdr) snapToGrid(rawHeadingRad) else rawHeadingRad
                val finalHeadingRad = applyDirectionLock(snappedHeading)

                updateLocation(stepLength, finalHeadingRad.toDouble(), headingSource)
            }
        } else {
            // Between steps, keep tracking valleys
            if (magnitude < currentStepMinAccel) currentStepMinAccel = magnitude
        }
    }

    /**
     * Enhanced HDR: Snap to 45° grid (supports diagonal directions)
     */
    private fun snapToGrid(roughRad: Float): Float {
        val step = Math.PI / 4.0  // 45 degrees instead of 90
        val multiple = Math.round(roughRad / step)
        val snapped = multiple * step
        val diff = Math.abs(roughRad - snapped)
        return if (diff < HDR_TOLERANCE_RAD) snapped.toFloat() else roughRad
    }

    /**
     * Direction Lock: Maintains heading when walking consistently in one direction
     * Fixed: Handles angle wrapping correctly (e.g., -π and +π are the same direction)
     */
    private fun applyDirectionLock(currentHeading: Float): Float {
        val locked = lockedHeadingRad

        if (locked != null) {
            // Calculate angular difference with wrapping
            val diff = normalizeAngle(currentHeading - locked)
            val absDiff = Math.abs(diff)

            if (absDiff < DIRECTION_UNLOCK_TOLERANCE) {
                consecutiveDirectionCount++
                return locked  // Stay locked
            } else {
                // Unlock - direction changed significantly
                lockedHeadingRad = null
                consecutiveDirectionCount = 1
                return currentHeading
            }
        } else {
            // No lock yet - check if we should establish one
            consecutiveDirectionCount++
            if (consecutiveDirectionCount >= DIRECTION_LOCK_THRESHOLD) {
                lockedHeadingRad = currentHeading
            }
            return currentHeading
        }
    }

    /**
     * Normalize angle to [-π, π] range
     */
    private fun normalizeAngle(angle: Float): Float {
        var normalized = angle
        while (normalized > Math.PI) normalized -= (2 * Math.PI).toFloat()
        while (normalized < -Math.PI) normalized += (2 * Math.PI).toFloat()
        return normalized
    }

    /**
     * Step Length Smoothing: Reduces step length variance
     */
    private fun smoothStepLength(raw: Double): Double {
        val clamped = raw.coerceIn(MIN_STEP_LENGTH, MAX_STEP_LENGTH)
        stepLengthHistory.add(clamped)
        if (stepLengthHistory.size > STEP_HISTORY_SIZE) {
            stepLengthHistory.removeAt(0)
        }
        return stepLengthHistory.average()
    }

    private fun updateLocation(stepLength: Double, headingRad: Double, source: String) {
        val dY = stepLength * cos(headingRad)
        val dX = stepLength * sin(headingRad)

        currentY += dY
        currentX += dX

        val dLat = (dY / EARTH_RADIUS) * (180.0 / Math.PI)
        val dLng = (dX / (EARTH_RADIUS * cos(currentLat * Math.PI / 180.0))) * (180.0 / Math.PI)

        currentLat += dLat
        currentLng += dLng

        val headingDeg = Math.toDegrees(headingRad).toInt()
        listener?.onPositionUpdated(currentX, currentY, LatLng(currentLat, currentLng), stepCount, headingDeg, stepLength, source)
    }
}
