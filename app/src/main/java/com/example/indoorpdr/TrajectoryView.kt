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

        if (points.isEmpty()) return

        val w = width.toFloat()
        val h = height.toFloat()
        val padding = 50f

        // 1. Calculate Bounds
        var minX = 0.0
        var maxX = 0.0
        var minY = 0.0
        var maxY = 0.0

        for (p in points) {
            if (p.first < minX) minX = p.first
            if (p.first > maxX) maxX = p.first
            if (p.second < minY) minY = p.second
            if (p.second > maxY) maxY = p.second
        }

        // Add some buffer to bounds so point isn't on the edge
        val rangeX = (maxX - minX).coerceAtLeast(2.0) // Min 2 meters range
        val rangeY = (maxY - minY).coerceAtLeast(2.0)

        // 2. Calculate Scale to fit screen
        val scaleX = (w - 2 * padding) / rangeX.toFloat()
        val scaleY = (h - 2 * padding) / rangeY.toFloat()
        scale = scaleX.coerceAtMost(scaleY)

        // 3. Calculate Translation (Centering)
        // Center of gravity or center of bounds? Let's use center of bounds.
        val midX = (maxX + minX) / 2.0
        val midY = (maxY + minY) / 2.0

        centerX = w / 2 - (midX * scale).toFloat()
        centerY = h / 2 + (midY * scale).toFloat()

        // Helper for Coordinate Conversion
        fun mToPxX(m: Double) = centerX + (m * scale).toFloat()
        fun mToPxY(m: Double) = centerY - (m * scale).toFloat()

        // 4. Draw Axis & Grid
        paint.color = 0xFFF0F0F0.toInt() // Very light gray
        paint.strokeWidth = 1f
        // Grid every 1 meter
        for (i in (minX.toInt()-5)..(maxX.toInt()+5)) {
            val px = mToPxX(i.toDouble())
            canvas.drawLine(px, 0f, px, h, paint)
        }
        for (i in (minY.toInt()-5)..(maxY.toInt()+5)) {
            val py = mToPxY(i.toDouble())
            canvas.drawLine(0f, py, w, py, paint)
        }

        paint.color = Color.LTGRAY
        paint.strokeWidth = 3f
        canvas.drawLine(mToPxX(minX - 2), centerY, mToPxX(maxX + 2), centerY, paint) // X Axis
        canvas.drawLine(centerX, mToPxY(minY - 2), centerX, mToPxY(maxY + 2), paint) // Y Axis

        // 5. Draw Path
        paint.color = Color.BLUE
        paint.strokeWidth = 8f

        for (i in 0 until points.size - 1) {
            val p1 = points[i]
            val p2 = points[i+1]
            canvas.drawLine(mToPxX(p1.first), mToPxY(p1.second), mToPxX(p2.first), mToPxY(p2.second), paint)
        }

        // 6. Draw Origin (Blue Cross)
        paint.color = Color.GREEN
        canvas.drawCircle(mToPxX(0.0), mToPxY(0.0), 10f, paint)

        // 7. Draw Current Pos (Red Dot)
        val last = points.last()
        canvas.drawCircle(mToPxX(last.first), mToPxY(last.second), 20f, dotPaint)
    }
}
