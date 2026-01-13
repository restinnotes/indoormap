package com.example.scs.pdr

import com.example.scs.math.QuaternionUtils
import kotlin.math.cos
import kotlin.math.sin

/**
 * PDR Engine
 * Implements Step Detection and Heuristic Drift Reduction (HDR)
 */
class PdrEngine(var stepLength: Float = 0.7f) {

    private var currentX = 0f
    private var currentY = 0f
    private var lastAccelMag = 0f
    private var isStepDetected = false

    // HDR Settings
    var useHDR = true
    private var lastHeading = 0f

    /**
     * Update PDR state using latest sensor data
     * @param accel (x, y, z) linear acceleration
     * @param heading current yaw from quaternion
     */
    fun update(accel: QuaternionUtils.Vector3, heading: Float): Pair<Float, Float>? {
        // 1. Simple Step Detection (Peak Detection)
        val mag = kotlin.math.sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z)
        val delta = mag - lastAccelMag
        lastAccelMag = mag

        // Thresholding for step detection (Simplified)
        if (mag > 12.0f && !isStepDetected) { // 12 m/s^2 peak
            isStepDetected = true

            // 2. Apply HDR if enabled
            var processedHeading = heading
            if (useHDR) {
                processedHeading = applyHDR(heading)
            }

            // 3. Update Position
            currentX += stepLength * cos(processedHeading)
            currentY += stepLength * sin(processedHeading)

            return Pair(currentX, currentY)
        }

        if (mag < 9.8f) {
            isStepDetected = false
        }

        return null
    }

    private fun applyHDR(currentHeading: Float): Float {
        // Snap to cardinal directions (0, 90, 180, 270) if change is small
        val degrees = Math.toDegrees(currentHeading.toDouble()).toFloat()
        val snapped = Math.round(degrees / 90.0) * 90.0

        // Only snap if we are within 15 degrees of a cardinal
        return if (Math.abs(degrees - snapped) < 15.0) {
            Math.toRadians(snapped).toFloat()
        } else {
            currentHeading
        }
    }

    fun reset(x: Float = 0f, y: Float = 10f) {
        currentX = x
        currentY = y
    }
}
