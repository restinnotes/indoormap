package com.example.indoorpdr

import android.graphics.Color
import com.amap.api.maps.AMap
import com.amap.api.maps.model.BitmapDescriptorFactory
import com.amap.api.maps.model.Circle
import com.amap.api.maps.model.CircleOptions
import com.amap.api.maps.model.LatLng
import com.amap.api.maps.model.Marker
import com.amap.api.maps.model.MarkerOptions
import com.amap.api.maps.model.Polyline
import com.amap.api.maps.model.PolylineOptions

class PdrVisualizer(private val aMap: AMap) {

    private var currentMarker: Marker? = null
    private var pathPolyline: Polyline? = null
    private val pathPoints = mutableListOf<LatLng>()

    // Draw the current PDR position as a Red Dot
    fun updatePosition(latLng: LatLng) {
        // 1. Update Marker (Current Position)
        if (currentMarker == null) {
            val markerOptions = MarkerOptions()
                .position(latLng)
                .icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_RED))
                .title("Current Position")
            currentMarker = aMap.addMarker(markerOptions)
        } else {
            currentMarker?.position = latLng
        }

        // 2. Update Trajectory (History)
        pathPoints.add(latLng)
        if (pathPolyline == null) {
            val polylineOptions = PolylineOptions()
                .addAll(pathPoints)
                .width(10f)
                .color(Color.BLUE)
            pathPolyline = aMap.addPolyline(polylineOptions)
        } else {
            pathPolyline?.points = pathPoints
        }
    }

    // Clear history for reset
    fun clearTrace() {
        pathPoints.clear()
        pathPolyline?.remove()
        pathPolyline = null
        currentMarker?.remove()
        currentMarker = null
    }
}
