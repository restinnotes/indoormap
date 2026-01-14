package com.example.indoorpdr

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.util.AttributeSet
import android.view.View

/**
 * A lightweight Line Chart for real-time sensor data debugging.
 * Draws multiple lines with auto-scaling Y-axis (optional) and scrolling X-axis.
 */
class SimpleLineChart @JvmOverloads constructor(
    context: Context, attrs: AttributeSet? = null, defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    private val MAX_POINTS = 300 // Window size
    private val lines = HashMap<String, MutableList<Float>>()
    private val lineColors = HashMap<String, Int>()
    private val paint = Paint().apply {
        strokeWidth = 3f
        style = Paint.Style.STROKE
        isAntiAlias = true
    }
    private val axisPaint = Paint().apply {
        color = Color.LTGRAY
        strokeWidth = 1f
        textSize = 24f
    }
    private val path = Path()

    // Config
    var yMin = -20f
    var yMax = 20f
    var autoScale = false

    fun addPoint(seriesName: String, color: Int, value: Float) {
        if (!lines.containsKey(seriesName)) {
            lines[seriesName] = mutableListOf()
            lineColors[seriesName] = color
        }
        val list = lines[seriesName]!!
        list.add(value)
        if (list.size > MAX_POINTS) {
            list.removeAt(0)
        }

        if (autoScale) {
            // Update min/max dynamically
            var minVal = Float.MAX_VALUE
            var maxVal = Float.MIN_VALUE
            for (l in lines.values) {
                for (v in l) {
                    if (v < minVal) minVal = v
                    if (v > maxVal) maxVal = v
                }
            }
            if (minVal < Float.MAX_VALUE) {
                // Add some padding
                val range = maxVal - minVal
                yMin = minVal - range * 0.1f
                yMax = maxVal + range * 0.1f
                if (range == 0f) { yMin -= 1f; yMax += 1f }
            }
        }
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        canvas.drawColor(0x80000000.toInt()) // Semi-transparent black bg

        val w = width.toFloat()
        val h = height.toFloat()

        // Draw Axes/Grid
        canvas.drawLine(0f, h/2, w, h/2, axisPaint) // Zero line approx (if symmetrical)

        // Draw Min/Max Labels
        canvas.drawText("%.1f".format(yMax), 10f, 30f, axisPaint)
        canvas.drawText("%.1f".format(yMin), 10f, h - 10f, axisPaint)

        // Draw Lines
        val xStep = w / (MAX_POINTS - 1)

        for ((name, points) in lines) {
            if (points.isEmpty()) continue

            paint.color = lineColors[name] ?: Color.WHITE
            path.reset()

            // Map value to Y pixel:
            // yPixel = h - (value - yMin) / (yMax - yMin) * h
            val yRange = yMax - yMin

            for (i in points.indices) {
                val value = points[i]
                val x = i * xStep
                // Clamp Y
                val safeVal = value.coerceIn(yMin, yMax)
                val y = h - ((safeVal - yMin) / yRange) * h

                if (i == 0) path.moveTo(x, y)
                else path.lineTo(x, y)
            }
            canvas.drawPath(path, paint)
        }
    }

    fun clear() {
        lines.clear()
        invalidate()
    }
}
