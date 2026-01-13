package com.example.scs.math

import kotlin.math.*

/**
 * Quaternion and Vector Math Utilities
 * Ported from `ur3_mujoco_bridge.py`
 */
object QuaternionUtils {

    data class Quaternion(val x: Float, val y: Float, val z: Float, val w: Float)
    data class Vector3(val x: Float, val y: Float, val z: Float)

    fun normalize(q: Quaternion): Quaternion {
        val mag = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        return if (mag > 0) {
            Quaternion(q.x / mag, q.y / mag, q.z / mag, q.w / mag)
        } else {
            Quaternion(0f, 0f, 0f, 1f)
        }
    }

    fun conjugate(q: Quaternion): Quaternion {
        return Quaternion(-q.x, -q.y, -q.z, q.w)
    }

    fun multiply(a: Quaternion, b: Quaternion): Quaternion {
        return Quaternion(
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
        )
    }

    /**
     * Converts a quaternion to Euler angles (Yaw, Pitch, Roll) in radians.
     * Useful for PDR heading extraction.
     */
    fun toEuler(q: Quaternion): Vector3 {
        // Roll (x-axis rotation)
        val sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        val cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        val roll = atan2(sinr_cosp, cosr_cosp)

        // Pitch (y-axis rotation)
        val sinp = 2 * (q.w * q.y - q.z * q.x)
        val pitch = if (abs(sinp) >= 1) {
            (PI / 2).toFloat() * sign(sinp)
        } else {
            asin(sinp)
        }

        // Yaw (z-axis rotation) - This is our PDR Heading
        val siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        val cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        val yaw = atan2(siny_cosp, cosy_cosp)

        return Vector3(roll, pitch, yaw)
    }

    /**
     * Calculates the relative rotation between two quaternions
     * Delta = q_current * conj(q_reference)
     */
    fun getRelativeRotation(current: Quaternion, reference: Quaternion): Quaternion {
        return multiply(current, conjugate(reference))
    }
}
