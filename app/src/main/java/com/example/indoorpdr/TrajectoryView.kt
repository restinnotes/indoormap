package com.example.indoorpdr

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import android.view.View

class TrajectoryView(context: Context, attrs: AttributeSet?) : View(context, attrs) {

    private val paint = Paint().apply {
        color = Color.BLACK
        strokeWidth = 5f
        style = Paint.Style.STROKE
        isAntiAlias = true
    }

    private val dotPaint = Paint().apply {
        color = Color.RED
        style = Paint.Style.FILL
        isAntiAlias = true
    }

    // Path Data: list of Pair(x, y) in meters
    private val points = mutableListOf<Pair<Double, Double>>()

    // Scale: Pixels per Meter
    private var scale = 50f // Initial: 1 meter = 50 pixels
    private var centerX = 0f
    private var centerY = 0f

    init {
        // Add start point
        points.add(Pair(0.0, 0.0))
    }

    fun addPoint(x: Double, y: Double) {
        points.add(Pair(x, y))
        invalidate() // Trigger redraw
    }

    fun clear() {
        points.clear()
        points.add(Pair(0.0, 0.0))
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)

        val w = width.toFloat()
        val h = height.toFloat()
        centerX = w / 2
        centerY = h / 2

        // Draw Axis
        paint.color = Color.LTGRAY
        paint.strokeWidth = 2f
        canvas.drawLine(0f, centerY, w, centerY, paint) // X Axis
        canvas.drawLine(centerX, 0f, centerX, h, paint) // Y Axis

        // Draw Grid (every 1 meter)
        paint.color = 0xFFEEEEEE.toInt()
        // ... grid logic omitted for simplicity ...

        // Draw Path
        if (points.isEmpty()) return

        paint.color = Color.BLUE
        paint.strokeWidth = 8f

        // Convert Meter coordinates to Screen coordinates
        // Screen X = CenterX + (MeterX * Scale)
        // Screen Y = CenterY - (MeterY * Scale)  <-- Y is inverted in screen coords (Up is minus)

        for (i in 0 until points.size - 1) {
            val p1 = points[i]
            val p2 = points[i+1]

            val x1 = centerX + (p1.first * scale).toFloat()
            val y1 = centerY - (p1.second * scale).toFloat()
            val x2 = centerX + (p2.first * scale).toFloat()
            val y2 = centerY - (p2.second * scale).toFloat()

            canvas.drawLine(x1, y1, x2, y2, paint)
        }

        // Draw Current Pos (Red Dot)
        val last = points.last()
        val lx = centerX + (last.first * scale).toFloat()
        val ly = centerY - (last.second * scale).toFloat()
        canvas.drawCircle(lx, ly, 15f, dotPaint)
    }
}
