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

    // Old State commented out for Prototype
    // private var phoneHeadingRad = 0.0f
    // private var scsHeadingRad: Float? = null
    // ...

    // --- New Prototype: Swing Plane Detection ---
    private val swingDetector = SwingPlaneDetector(this)

    // Call this from Activity with RAW data
    fun setScsRawData(ax: Float, ay: Float, az: Float, gx: Float, gy: Float, gz: Float) {
        swingDetector.addSample(ax, ay, az)
    }

    /**
     * Call this from BLE callback when new SCS quaternion arrives.
     * (COMMENTED OUT FOR SWING PLANE PROTOYPING)
     */
    fun setScsQuaternion(qx: Float, qy: Float, qz: Float, qw: Float) {
        // ... (Old Logic Disabled)
    }

    override fun onSensorChanged(event: SensorEvent?) {
        if (event == null) return
        when (event.sensor.type) {
            Sensor.TYPE_ROTATION_VECTOR -> {
                // handleRotation(event) // Disabled
            }
            Sensor.TYPE_ACCELEROMETER -> {
                // Keep minimal step detection for now, but disable fusion
                // handleAccel(event)
            }
        }
    }

    // New Listener Method
    fun onSwingPlaneDetected(normal: FloatArray, heading: FloatArray, quality: Float) {
        listener?.onDebugMessage("Swing Detected! Quality: %.2f".format(quality))
        // Send this data to UI for visualization
        // Encoded as: "SWING_PLANE,nx,ny,nz,hx,hy,hz"
        listener?.onDebugMessage("SWING_PLANE,${normal[0]},${normal[1]},${normal[2]},${heading[0]},${heading[1]},${heading[2]}")
    }

    // --- Inner Class: Swing Plane Detector ---
    class SwingPlaneDetector(val engine: HybridPdrEngine) {
        private val WINDOW_SIZE = 100 // 2 seconds @ 50Hz
        private val axBuffer = FloatArray(WINDOW_SIZE)
        private val ayBuffer = FloatArray(WINDOW_SIZE)
        private val azBuffer = FloatArray(WINDOW_SIZE)
        private var ptr = 0
        private var isFull = false
        private var lastProcessTime = 0L
        private val PROCESS_INTERVAL_MS = 500L // Run PCA every 0.5s

        fun addSample(ax: Float, ay: Float, az: Float) {
            axBuffer[ptr] = ax
            ayBuffer[ptr] = ay
            azBuffer[ptr] = az
            ptr = (ptr + 1) % WINDOW_SIZE
            if (ptr == 0) isFull = true

            val now = System.currentTimeMillis()
            if (isFull && now - lastProcessTime > PROCESS_INTERVAL_MS) {
                processWindow()
                lastProcessTime = now
            }
        }

        private fun processWindow() {
            // 1. Calculate Mean
            var sumX = 0f; var sumY = 0f; var sumZ = 0f
            for (i in 0 until WINDOW_SIZE) {
                sumX += axBuffer[i]
                sumY += ayBuffer[i]
                sumZ += azBuffer[i]
            }
            val meanX = sumX / WINDOW_SIZE
            val meanY = sumY / WINDOW_SIZE
            val meanZ = sumZ / WINDOW_SIZE
            val gravity = floatArrayOf(meanX, meanY, meanZ) // Approx Gravity Vector

            // 2. Build Covariance Matrix (3x3 symmetric)
            // xx xy xz
            // yx yy yz
            // zx zy zz
            var xx=0f; var xy=0f; var xz=0f; var yy=0f; var yz=0f; var zz=0f

            for (i in 0 until WINDOW_SIZE) {
                val dx = axBuffer[i] - meanX
                val dy = ayBuffer[i] - meanY
                val dz = azBuffer[i] - meanZ
                xx += dx*dx
                xy += dx*dy
                xz += dx*dz
                yy += dy*dy
                yz += dy*dz
                zz += dz*dz
            }
            // Normalize
            xx/=WINDOW_SIZE; xy/=WINDOW_SIZE; xz/=WINDOW_SIZE
            yy/=WINDOW_SIZE; yz/=WINDOW_SIZE; zz/=WINDOW_SIZE

            // 3. Eigen Decomposition (Jacobi Algorithm for 3x3)
            // We want the eigenvector with the SMALLEST eigenvalue (Normal to plane)
            // and LARGEST (Swing Tangent)
            val eigen = solveEigen3x3(xx, xy, xz, yy, yz, zz)

            // Sort eigenvalues: e[0] <= e[1] <= e[2]
            // Normal = v[0] (Smallest variance - Left/Right)
            // Tangent = v[2] (Largest variance - Forward/Back arc)

            val normal = eigen.vectors[0]
            val tangent = eigen.vectors[2]

            // Quality metric: Ratio of Tangent Variance to Normal Variance
            // If planar, Tangent >> Normal.
            val quality = if (eigen.values[0] > 0) eigen.values[2] / eigen.values[0] else 999f

            if (quality > 4.0) { // Threshold for "Planar enough"
                // 4. Compute Heading Vector
                // Heading = Cross(Normal, Gravity)
                // Assuming Normal is roughly Left/Right and Gravity is Down.
                var hx = normal[1]*gravity[2] - normal[2]*gravity[1]
                var hy = normal[2]*gravity[0] - normal[0]*gravity[2]
                var hz = normal[0]*gravity[1] - normal[1]*gravity[0]

                // Normalize Heading
                val normH = sqrt(hx*hx + hy*hy + hz*hz)
                if (normH > 0.001) {
                    hx /= normH
                    hy /= normH
                    hz /= normH
                    engine.onSwingPlaneDetected(normal, floatArrayOf(hx, hy, hz), quality)
                }
            }
        }
    }

    // --- Minimal 3x3 Eigen Solver (Jacobi) ---
    data class EigenResult(val values: FloatArray, val vectors: Array<FloatArray>)

    // Using a simpler approximation or hardcoded Jacobi for 3x3
    // But since I cannot import external maths, I will copy a tiny solver here.
    private fun solveEigen3x3(xx: Float, xy: Float, xz: Float, yy: Float, yz: Float, zz: Float): EigenResult {
        // Identity init
        val V = Array(3) { FloatArray(3) }
        V[0][0]=1f; V[1][1]=1f; V[2][2]=1f

        var d0=xx; var d1=yy; var d2=zz
        var m01=xy; var m02=xz; var m12=yz

        // 5 Iterations is usually enough for 3x3
        for (iter in 0 until 5) {
             // Rotate 0-1
             if (Math.abs(m01) > 1e-6) {
                 val theta = 0.5 * Math.atan2(2.0*m01, (d1-d0).toDouble())
                 val c = cos(theta).toFloat()
                 val s = sin(theta).toFloat()
                 // Update d
                 val d0_ = c*c*d0 - 2*c*s*m01 + s*s*d1
                 val d1_ = s*s*d0 + 2*c*s*m01 + c*c*d1
                 d0=d0_; d1=d1_
                 m01=0f // clear
                 // Update off-diagonals
                 val m02_ = c*m02 - s*m12
                 val m12_ = s*m02 + c*m12
                 m02=m02_; m12=m12_
                 // Update V
                 for(k in 0..2) { val v0=V[k][0]; val v1=V[k][1]; V[k][0]=c*v0-s*v1; V[k][1]=s*v0+c*v1 }
             }
             // Rotate 0-2
             if (Math.abs(m02) > 1e-6) {
                 val theta = 0.5 * Math.atan2(2.0*m02, (d2-d0).toDouble())
                 val c = cos(theta).toFloat()
                 val s = sin(theta).toFloat()
                 val d0_ = c*c*d0 - 2*c*s*m02 + s*s*d2
                 val d2_ = s*s*d0 + 2*c*s*m02 + c*c*d2
                 d0=d0_; d2=d2_
                 m02=0f
                 val m01_ = c*m01 - s*m12 // m01 is 0 but let's be safe
                 val m12_ = s*m01 + c*m12
                 m01=m01_; m12=m12_
                 for(k in 0..2) { val v0=V[k][0]; val v2=V[k][2]; V[k][0]=c*v0-s*v2; V[k][2]=s*v0+c*v2 }
             }
             // Rotate 1-2
             if (Math.abs(m12) > 1e-6) {
                 val theta = 0.5 * Math.atan2(2.0*m12, (d2-d1).toDouble())
                 val c = cos(theta).toFloat()
                 val s = sin(theta).toFloat()
                 val d1_ = c*c*d1 - 2*c*s*m12 + s*s*d2
                 val d2_ = s*s*d1 + 2*c*s*m12 + c*c*d2
                 d1=d1_; d2=d2_
                 m12=0f
                 val m01_ = c*m01 - s*m02
                 val m02_ = s*m01 + c*m02
                 m01=m01_; m02=m02_
                 for(k in 0..2) { val v1=V[k][1]; val v2=V[k][2]; V[k][1]=c*v1-s*v2; V[k][2]=s*v1+c*v2 }
             }
        }

        // Return sorted results
        // Bubblesort the 3 values/vectors
        val valArr = floatArrayOf(d0, d1, d2)
        val vecArr = arrayOf(floatArrayOf(V[0][0], V[1][0], V[2][0]), floatArrayOf(V[0][1], V[1][1], V[2][1]), floatArrayOf(V[0][2], V[1][2], V[2][2]))

        // Sort Ascending
        for(i in 0..1) {
            for(j in 0 until 2-i) {
                if(valArr[j] > valArr[j+1]) {
                    val t = valArr[j]; valArr[j]=valArr[j+1]; valArr[j+1]=t
                    val tv = vecArr[j]; vecArr[j]=vecArr[j+1]; vecArr[j+1]=tv
                }
            }
        }
        return EigenResult(valArr, vecArr)
    }

    // Keep stop() and listener interface
    fun start(startPoint: LatLng) {
        isRunning = true
        listener?.onDebugMessage("Swing Plane Prototype Started. Walk and swing arms!")
    }
    fun stop() { isRunning = false }


    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    private fun handleRotation(event: SensorEvent) {
        val rotationMatrix = FloatArray(9)
        SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values)
        val orientationAngles = FloatArray(3)
        SensorManager.getOrientation(rotationMatrix, orientationAngles)
        phoneHeadingRad = orientationAngles[0]
        phoneAverager.add(phoneHeadingRad) // Add to averager
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

                // --- Dynamic Complementary Filter Fusion ---

                val isScsAvailable = (scsHeadingRad != null && scsAge < 500)

                if (isScsAvailable) {
                    val scsH = scsHeadingRad!!
                    val phoneH = phoneHeadingRad

                    if (!isScsUnstable) {
                        // Case A: SCS is Stable -> Trust SCS & Learn Offset
                        rawHeadingRad = scsH
                        headingSource = "SCS"

                        // Update Offset using AVERAGES to cancel out swing noise
                        val avgScs = scsAverager.getAverage()
                        val avgPhone = phoneAverager.getAverage()
                        val currentOffset = normalizeAngle(avgScs - avgPhone)

                        if (!isOffsetInitialized) {
                            headingOffsetRad = currentOffset
                            isOffsetInitialized = true
                        } else {
                            // Smoothly update offset (EMA)
                            // We use a smaller alpha because averaging is already slow
                            val offsetDiff = normalizeAngle(currentOffset - headingOffsetRad)
                            headingOffsetRad = normalizeAngle(headingOffsetRad + 0.02f * offsetDiff)
                        }

                    } else {
                        // Case B: SCS is Unstable (Scratching) -> Use Phone + Offset
                        if (isOffsetInitialized) {
                            rawHeadingRad = normalizeAngle(phoneH + headingOffsetRad)
                            headingSource = "Phone(Fused)"
                        } else {
                            // Fallback if offset not ready (rare)
                            rawHeadingRad = phoneH
                            headingSource = "Phone(Raw)"
                        }
                    }
                } else {
                    // Case C: SCS Not Connected -> Use Phone
                    rawHeadingRad = phoneHeadingRad
                    headingSource = "Phone"
                }

                // Apply HDR (45° Snap) to the fused heading
                val finalHeadingRad = if (useHdr) snapToGrid(rawHeadingRad) else rawHeadingRad

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

    /**
     * Circular Buffer to calculate Mean Angle
     */
    private class AngleAverager(val size: Int) {
        private val sinBuffer = FloatArray(size)
        private val cosBuffer = FloatArray(size)
        private var index = 0
        private var count = 0
        private var currentAvg = 0.0f

        fun add(angleRad: Float) {
            sinBuffer[index] = sin(angleRad)
            cosBuffer[index] = cos(angleRad)
            index = (index + 1) % size
            if (count < size) count++

            // Re-calc average
            var sumSin = 0.0f
            var sumCos = 0.0f
            for (i in 0 until count) {
                sumSin += sinBuffer[i]
                sumCos += cosBuffer[i]
            }
            currentAvg = kotlin.math.atan2(sumSin, sumCos)
        }

        fun getAverage(): Float {
             return currentAvg
        }
    }
}
