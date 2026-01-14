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

/**
 * Hybrid PDR Engine
 * - Step Detection: from Phone Accelerometer
 * - Heading: from SCS Quaternion (if connected), fallback to Phone Rotation Vector
 * - Swing Plane: PCA-based arm swing analysis
 */
class HybridPdrEngine(private val context: Context) : SensorEventListener {

    // --- Interfaces ---
    interface PdrListener {
        fun onPositionUpdated(x: Double, y: Double, latLng: LatLng, stepCount: Int, headingDeg: Int, stepLength: Double, source: String)
        fun onDebugMessage(msg: String)
        fun onRawData(ax: Float, ay: Float, az: Float)
    }

    // --- Enums ---
    enum class SwingSource { SCS, PHONE }

    // --- Public Properties ---
    var listener: PdrListener? = null
    var swingSource = SwingSource.SCS

    // --- Private Components ---
    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val swingDetector = SwingPlaneDetector(this)
    private var isRunning = false

    // --- Control Methods ---

    fun start(startPoint: LatLng) {
        isRunning = true
        listener?.onDebugMessage("Swing Plane Prototype Started. Walk and swing arms!")

        // Register Phone Sensors
        val accel = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        sensorManager.registerListener(this, accel, SensorManager.SENSOR_DELAY_GAME)
    }

    fun stop() {
        isRunning = false
        sensorManager.unregisterListener(this)
    }

    fun setSwingSource(source: SwingSource) {
        this.swingSource = source
        swingDetector.clear()
        listener?.onDebugMessage("Switched Swing Source to: $source")
    }

    // --- Data Input ---

    // Call this from Activity with RAW data
    fun setScsRawData(ax: Float, ay: Float, az: Float, gx: Float, gy: Float, gz: Float) {
        if (swingSource == SwingSource.SCS) {
            swingDetector.addSample(ax, ay, az)
        }
    }

    // --- SensorEventListener ---

    override fun onSensorChanged(event: SensorEvent?) {
        if (event == null) return
        if (event.sensor.type == Sensor.TYPE_ACCELEROMETER) {
            val ax = event.values[0]
            val ay = event.values[1]
            val az = event.values[2]

            if (swingSource == SwingSource.PHONE) {
                swingDetector.addSample(ax, ay, az)
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    // --- Callbacks ---

    fun onSwingPlaneDetected(normal: FloatArray, heading: FloatArray, quality: Float) {
        // Send this data to UI for visualization
        // Encoded as: "SWING_PLANE,nx,ny,nz,hx,hy,hz,quality"
        listener?.onDebugMessage("SWING_PLANE,${normal[0]},${normal[1]},${normal[2]},${heading[0]},${heading[1]},${heading[2]},$quality")
    }

    // --- Internal Math Helpers ---

    data class EigenResult(val values: FloatArray, val vectors: Array<FloatArray>)

    fun solveEigen3x3(xx: Float, xy: Float, xz: Float, yy: Float, yz: Float, zz: Float): EigenResult {
        // Identity init
        val V = Array(3) { FloatArray(3) }
        V[0][0]=1f; V[1][1]=1f; V[2][2]=1f
        var d0=xx; var d1=yy; var d2=zz
        var m01=xy; var m02=xz; var m12=yz

        for (iter in 0 until 5) {
             if (Math.abs(m01) > 1e-6) {
                 val theta = 0.5 * Math.atan2(2.0*m01, (d1-d0).toDouble())
                 val c = cos(theta).toFloat(); val s = sin(theta).toFloat()
                 val d0_ = c*c*d0 - 2*c*s*m01 + s*s*d1
                 val d1_ = s*s*d0 + 2*c*s*m01 + c*c*d1
                 d0=d0_; d1=d1_; m01=0f
                 val m02_ = c*m02 - s*m12
                 val m12_ = s*m02 + c*m12
                 m02=m02_; m12=m12_
                 for(k in 0..2) { val v0=V[k][0]; val v1=V[k][1]; V[k][0]=c*v0-s*v1; V[k][1]=s*v0+c*v1 }
             }
             if (Math.abs(m02) > 1e-6) {
                 val theta = 0.5 * Math.atan2(2.0*m02, (d2-d0).toDouble())
                 val c = cos(theta).toFloat(); val s = sin(theta).toFloat()
                 val d0_ = c*c*d0 - 2*c*s*m02 + s*s*d2
                 val d2_ = s*s*d0 + 2*c*s*m02 + c*c*d2
                 d0=d0_; d2=d2_; m02=0f
                 val m01_ = c*m01 - s*m12; val m12_ = s*m01 + c*m12
                 m01=m01_; m12=m12_
                 for(k in 0..2) { val v0=V[k][0]; val v2=V[k][2]; V[k][0]=c*v0-s*v2; V[k][2]=s*v0+c*v2 }
             }
             if (Math.abs(m12) > 1e-6) {
                 val theta = 0.5 * Math.atan2(2.0*m12, (d2-d1).toDouble())
                 val c = cos(theta).toFloat(); val s = sin(theta).toFloat()
                 val d1_ = c*c*d1 - 2*c*s*m12 + s*s*d2
                 val d2_ = s*s*d1 + 2*c*s*m12 + c*c*d2
                 d1=d1_; d2=d2_; m12=0f
                 val m01_ = c*m01 - s*m02; val m02_ = s*m01 + c*m02
                 m01=m01_; m02=m02_
                 for(k in 0..2) { val v1=V[k][1]; val v2=V[k][2]; V[k][1]=c*v1-s*v2; V[k][2]=s*v1+c*v2 }
             }
        }
        val valArr = floatArrayOf(d0, d1, d2)
        val vecArr = arrayOf(floatArrayOf(V[0][0], V[1][0], V[2][0]), floatArrayOf(V[0][1], V[1][1], V[2][1]), floatArrayOf(V[0][2], V[1][2], V[2][2]))

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

    // --- Inner Class: Swing Plane Detector ---
    class SwingPlaneDetector(val engine: HybridPdrEngine) {
        private val WINDOW_SIZE = 100 // 2 seconds @ 50Hz
        private val axBuffer = FloatArray(WINDOW_SIZE)
        private val ayBuffer = FloatArray(WINDOW_SIZE)
        private val azBuffer = FloatArray(WINDOW_SIZE)
        private var ptr = 0
        private var isFull = false
        private var lastProcessTime = 0L
        private val PROCESS_INTERVAL_MS = 500L

        fun clear() {
            ptr = 0
            isFull = false
        }

        fun addSample(ax: Float, ay: Float, az: Float) {
            engine.listener?.onRawData(ax, ay, az)
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
            var sumX = 0f; var sumY = 0f; var sumZ = 0f
            for (i in 0 until WINDOW_SIZE) {
                sumX += axBuffer[i]
                sumY += ayBuffer[i]
                sumZ += azBuffer[i]
            }
            val meanX = sumX / WINDOW_SIZE
            val meanY = sumY / WINDOW_SIZE
            val meanZ = sumZ / WINDOW_SIZE
            val gravity = floatArrayOf(meanX, meanY, meanZ)

            var xx=0f; var xy=0f; var xz=0f; var yy=0f; var yz=0f; var zz=0f
            for (i in 0 until WINDOW_SIZE) {
                val dx = axBuffer[i] - meanX
                val dy = ayBuffer[i] - meanY
                val dz = azBuffer[i] - meanZ
                xx += dx*dx; xy += dx*dy; xz += dx*dz
                yy += dy*dy; yz += dy*dz; zz += dz*dz
            }
            xx/=WINDOW_SIZE; xy/=WINDOW_SIZE; xz/=WINDOW_SIZE
            yy/=WINDOW_SIZE; yz/=WINDOW_SIZE; zz/=WINDOW_SIZE

            val eigen = engine.solveEigen3x3(xx, xy, xz, yy, yz, zz)
            val normal = eigen.vectors[0]
            val quality = if (eigen.values[0] > 0) eigen.values[2] / eigen.values[0] else 999f

            if (quality > 4.0) {
                var hx = normal[1]*gravity[2] - normal[2]*gravity[1]
                var hy = normal[2]*gravity[0] - normal[0]*gravity[2]
                var hz = normal[0]*gravity[1] - normal[1]*gravity[0]
                val normH = sqrt(hx*hx + hy*hy + hz*hz)
                if (normH > 0.001) {
                    hx /= normH; hy /= normH; hz /= normH
                    engine.onSwingPlaneDetected(normal, floatArrayOf(hx, hy, hz), quality)
                }
            } else {
                engine.onSwingPlaneDetected(floatArrayOf(0f,0f,0f), floatArrayOf(0f,0f,0f), quality)
            }
        }
    }
}
