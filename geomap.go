/*
 * Copyright 2008, 2013 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package geomap

import (
	"math"
	"strings"
)

const DEFAULT_EARTH_RADIUS = float64(6378137)

type LatLng struct {
	Latitude  float64
	Longitude float64
}

type PolyUtil struct {
	EarthRadius float64
	Math        *MathUtil
	Spherical   *SphericalUtil
}

// NewPolyUtil creates a new PolyUtil instance with the specified earth radius
func NewPolyUtil(earthRadius float64) *PolyUtil {
	return &PolyUtil{
		EarthRadius: earthRadius,
		Math:        NewMathUtil(earthRadius),
		Spherical:   NewSphericalUtil(earthRadius),
	}
}

func (pu *PolyUtil) DistanceBetweenPoints(from, to LatLng) float64 {
	toLat := to.Latitude
	fromLat := from.Latitude
	toLong := to.Longitude
	fromLong := from.Longitude

	dLat := toRadians(toLat - fromLat)
	dLon := toRadians(toLong - fromLong)

	a := math.Sin(dLat/2)*math.Sin(dLat/2) + math.Cos(toRadians(fromLat))*math.Cos(toRadians(toLat))*math.Sin(dLon/2)*math.Sin(dLon/2)
	c := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))

	distance := pu.EarthRadius * c

	return distance
}

// TanLatGC returns tan(latitude-at-lng3) on the great circle (lat1, lng1) to (lat2, lng2)
func (pu *PolyUtil) TanLatGC(lat1, lat2, lng2, lng3 float64) float64 {
	return (math.Tan(lat1)*math.Sin(lng2-lng3) + math.Tan(lat2)*math.Sin(lng3)) / math.Sin(lng2)
}

// MercatorLatRhumb returns mercator(latitude-at-lng3) on the Rhumb line (lat1, lng1) to (lat2, lng2)
func (pu *PolyUtil) MercatorLatRhumb(lat1, lat2, lng2, lng3 float64) float64 {
	return (mercator(lat1)*(lng2-lng3) + mercator(lat2)*lng3) / lng2
}

// Intersects computes whether the vertical segment (lat3, lng3) to South Pole intersects the segment (lat1, lng1) to (lat2, lng2)
func (pu *PolyUtil) Intersects(lat1, lat2, lng2, lat3, lng3 float64, geodesic bool) bool {
	if (lng3 >= 0 && lng3 >= lng2) || (lng3 < 0 && lng3 < lng2) {
		return false
	}
	if lat3 <= -math.Pi/2 {
		return false
	}
	if lat1 <= -math.Pi/2 || lat2 <= -math.Pi/2 || lat1 >= math.Pi/2 || lat2 >= math.Pi/2 {
		return false
	}
	if lng2 <= -math.Pi {
		return false
	}
	linearLat := (lat1*(lng2-lng3) + lat2*lng3) / lng2
	if lat1 >= 0 && lat2 >= 0 && lat3 < linearLat {
		return false
	}
	if lat1 <= 0 && lat2 <= 0 && lat3 >= linearLat {
		return true
	}
	if lat3 >= math.Pi/2 {
		return true
	}

	if geodesic {
		return math.Tan(lat3) >= pu.TanLatGC(lat1, lat2, lng2, lng3)
	}

	return mercator(lat3) >= pu.MercatorLatRhumb(lat1, lat2, lng2, lng3)
}

// ContainsLocation computes whether the given point lies inside the specified polygon
func (pu *PolyUtil) ContainsLocation(latitude, longitude float64, polygon []LatLng, geodesic bool) bool {
	size := len(polygon)

	if size == 0 {
		return false
	}

	lat3 := toRadians(latitude)
	lng3 := toRadians(longitude)
	prev := polygon[size-1]
	lat1 := toRadians(prev.Latitude)
	lng1 := toRadians(prev.Longitude)
	nIntersect := 0
	for _, point2 := range polygon {
		dLng3 := wrap(lng3-lng1, -math.Pi, math.Pi)
		if lat3 == lat1 && dLng3 == 0 {
			return true
		}
		lat2 := toRadians(point2.Latitude)
		lng2 := toRadians(point2.Longitude)
		if pu.Intersects(lat1, lat2, wrap(lng2-lng1, -math.Pi, math.Pi), lat3, dLng3, geodesic) {
			nIntersect++
		}
		lat1 = lat2
		lng1 = lng2
	}
	return nIntersect&1 != 0
}

// IsLocationOnEdge computes whether the given point lies on or near the edge of a polygon, within a specified tolerance in meters
func (pu *PolyUtil) IsLocationOnEdge(point LatLng, polygon []LatLng, geodesic bool, tolerance float64) bool {
	return pu.isLocationOnEdgeOrPath(point, polygon, true, geodesic, tolerance)
}

// IsLocationOnEdgeWithDefaultTolerance is the same as IsLocationOnEdge with a default tolerance of 0.1 meters
func (pu *PolyUtil) IsLocationOnEdgeWithDefaultTolerance(point LatLng, polygon []LatLng, geodesic bool) bool {
	return pu.IsLocationOnEdge(point, polygon, geodesic, 0.1)
}

// IsLocationOnPath computes whether the given point lies on or near a polyline, within a specified tolerance in meters
func (pu *PolyUtil) IsLocationOnPath(point LatLng, polyline []LatLng, geodesic bool, tolerance float64) bool {
	return pu.isLocationOnEdgeOrPath(point, polyline, false, geodesic, tolerance)
}

// IsLocationOnPathWithDefaultTolerance is the same as IsLocationOnPath with a default tolerance of 0.1 meters
func (pu *PolyUtil) IsLocationOnPathWithDefaultTolerance(point LatLng, polyline []LatLng, geodesic bool) bool {
	return pu.IsLocationOnPath(point, polyline, geodesic, 0.1)
}

func (pu *PolyUtil) isLocationOnEdgeOrPath(point LatLng, poly []LatLng, closed, geodesic bool, toleranceEarth float64) bool {
	idx := pu.locationIndexOnEdgeOrPath(point, poly, closed, geodesic, toleranceEarth)
	return idx >= 0
}

func (pu *PolyUtil) locationIndexOnEdgeOrPath(point LatLng, poly []LatLng, closed, geodesic bool, toleranceEarth float64) int {
	size := len(poly)
	if size == 0 {
		return -1
	}
	tolerance := toleranceEarth / pu.EarthRadius
	havTolerance := hav(tolerance)
	lat3 := toRadians(point.Latitude)
	lng3 := toRadians(point.Longitude)
	prev := poly[0]
	if closed {
		prev = poly[size-1]
	}
	lat1 := toRadians(prev.Latitude)
	lng1 := toRadians(prev.Longitude)
	idx := 0
	if geodesic {
		for _, point2 := range poly {
			lat2 := toRadians(point2.Latitude)
			lng2 := toRadians(point2.Longitude)
			if isOnSegmentGC(lat1, lng1, lat2, lng2, lat3, lng3, havTolerance) {
				return maxInt(0, idx-1)
			}
			lat1 = lat2
			lng1 = lng2
			idx++
		}
	} else {
		minAcceptable := lat3 - tolerance
		maxAcceptable := lat3 + tolerance
		y1 := mercator(lat1)
		y3 := mercator(lat3)
		xTry := make([]float64, 3)
		for _, point2 := range poly {
			lat2 := toRadians(point2.Latitude)
			y2 := mercator(lat2)
			lng2 := toRadians(point2.Longitude)
			if math.Max(lat1, lat2) >= minAcceptable && math.Min(lat1, lat2) <= maxAcceptable {
				x2 := wrap(lng2-lng1, -math.Pi, math.Pi)
				x3Base := wrap(lng3-lng1, -math.Pi, math.Pi)
				xTry[0] = x3Base
				xTry[1] = x3Base + 2*math.Pi
				xTry[2] = x3Base - 2*math.Pi
				for _, x3 := range xTry {
					dy := y2 - y1
					len2 := x2*x2 + dy*dy
					t := 0.0
					if len2 > 0 {
						t = clamp((x3*x2+(y3-y1)*dy)/len2, 0, 1)
					}
					xClosest := t * x2
					yClosest := y1 + t*dy
					latClosest := inverseMercator(yClosest)
					havDist := havDistance(lat3, latClosest, x3-xClosest)
					if havDist < havTolerance {
						return maxInt(0, idx-1)
					}
				}
			}
			lat1 = lat2
			lng1 = lng2
			y1 = y2
			idx++
		}
	}
	return -1
}

func isOnSegmentGC(lat1, lng1, lat2, lng2, lat3, lng3, havTolerance float64) bool {
	havDist13 := havDistance(lat1, lat3, lng1-lng3)
	if havDist13 <= havTolerance {
		return true
	}
	havDist23 := havDistance(lat2, lat3, lng2-lng3)
	if havDist23 <= havTolerance {
		return true
	}
	sinBearing := sinDeltaBearing(lat1, lng1, lat2, lng2, lat3, lng3)
	sinDist13 := sinFromHav(havDist13)
	havCrossTrack := havFromSin(sinDist13 * sinBearing)
	if havCrossTrack > havTolerance {
		return false
	}
	havDist12 := havDistance(lat1, lat2, lng1-lng2)
	term := havDist12 + havCrossTrack*(1-2*havDist12)
	if havDist13 > term || havDist23 > term {
		return false
	}
	if havDist12 < 0.74 {
		return true
	}
	cosCrossTrack := 1 - 2*havCrossTrack
	havAlongTrack13 := (havDist13 - havCrossTrack) / cosCrossTrack
	havAlongTrack23 := (havDist23 - havCrossTrack) / cosCrossTrack
	sinSumAlongTrack := sinSumFromHav(havAlongTrack13, havAlongTrack23)
	return sinSumAlongTrack > 0
}

// Simplify simplifies the given poly (polyline or polygon) using the Douglas-Peucker decimation algorithm
func (pu *PolyUtil) Simplify(poly []LatLng, tolerance float64) []LatLng {
	n := len(poly)
	if n < 1 {
		panic("Polyline must have at least 1 point")
	}
	if tolerance <= 0 {
		panic("Tolerance must be greater than zero")
	}

	closedPolygon := isClosedPolygon(poly)
	var lastPoint *LatLng

	if closedPolygon {
		const OFFSET = 0.00000000001
		lastPoint = &poly[len(poly)-1]
		poly = poly[:len(poly)-1]
		poly = append(poly, LatLng{lastPoint.Latitude + OFFSET, lastPoint.Longitude + OFFSET})
	}

	idx := 0
	maxIdx := 0
	stack := [][]int{{0, n - 1}}
	dists := make([]float64, n)
	dists[0] = 1
	dists[n-1] = 1
	var maxDist, dist float64
	var current []int

	for len(stack) > 0 {
		current, stack = stack[len(stack)-1], stack[:len(stack)-1]
		maxDist = 0
		for idx = current[0] + 1; idx < current[1]; idx++ {
			dist = pu.DistanceToLine(poly[idx], poly[current[0]], poly[current[1]])
			if dist > maxDist {
				maxDist = dist
				maxIdx = idx
			}
		}
		if maxDist > tolerance {
			dists[maxIdx] = maxDist
			stack = append(stack, []int{current[0], maxIdx}, []int{maxIdx, current[1]})
		}
	}

	if closedPolygon {
		poly = poly[:len(poly)-1]
		poly = append(poly, *lastPoint)
	}

	simplifiedLine := []LatLng{}
	for i, l := range poly {
		if dists[i] != 0 {
			simplifiedLine = append(simplifiedLine, l)
		}
	}

	return simplifiedLine
}

// IsClosedPolygon returns true if the provided list of points is a closed polygon (i.e., the first and last points are the same)
func isClosedPolygon(poly []LatLng) bool {
	firstPoint := poly[0]
	lastPoint := poly[len(poly)-1]
	return firstPoint == lastPoint
}

// DistanceToLine computes the distance on the sphere between the point p and the line segment start to end
func (pu *PolyUtil) DistanceToLine(p, start, end LatLng) float64 {
	if start == end {
		return pu.Spherical.ComputeDistanceBetween(end, p)
	}

	s0lat := toRadians(p.Latitude)
	s0lng := toRadians(p.Longitude)
	s1lat := toRadians(start.Latitude)
	s1lng := toRadians(start.Longitude)
	s2lat := toRadians(end.Latitude)
	s2lng := toRadians(end.Longitude)

	s := pu.Spherical.ComputeDistanceBetween(start, end)
	d := pu.Spherical.ComputeDistanceBetween(start, p)

	if s == 0 {
		return d
	}

	t := ((s0lat-s1lat)*(s2lat-s1lat) + (s0lng-s1lng)*(s2lng-s1lng)) / (s * s)
	if t < 0 {
		return pu.Spherical.ComputeDistanceBetween(p, start)
	} else if t > 1 {
		return pu.Spherical.ComputeDistanceBetween(p, end)
	}

	interpolatedLat := start.Latitude + t*(end.Latitude-start.Latitude)
	interpolatedLng := start.Longitude + t*(end.Longitude-start.Longitude)
	interpolatedPoint := LatLng{Latitude: interpolatedLat, Longitude: interpolatedLng}

	return pu.Spherical.ComputeDistanceBetween(p, interpolatedPoint)
}

// Decode decodes an encoded path string into a sequence of LatLngs
func (pu *PolyUtil) Decode(encodedPath string) []LatLng {
	len := len(encodedPath)
	path := make([]LatLng, 0, len/2)
	index := 0
	lat := 0
	lng := 0

	for index < len {
		result := 1
		shift := 0
		var b int
		for {
			b = int(encodedPath[index]) - 63 - 1
			index++
			result += b << shift
			shift += 5
			if b < 0x1f {
				break
			}
		}

		if (result & 1) != 0 {
			lat += ^(result >> 1)
		} else {
			lat += result >> 1
		}

		result = 1
		shift = 0

		for {
			b = int(encodedPath[index]) - 63 - 1
			index++
			result += b << shift
			shift += 5
			if b < 0x1f {
				break
			}
		}

		if (result & 1) != 0 {
			lng += ^(result >> 1)
		} else {
			lng += result >> 1
		}

		path = append(path, LatLng{float64(lat) * 1e-5, float64(lng) * 1e-5})
	}

	return path
}

// Encode encodes a sequence of LatLngs into an encoded path string
func (pu *PolyUtil) Encode(path []LatLng) string {
	var lastLat int64
	var lastLng int64
	var result strings.Builder

	for _, point := range path {
		lat := int64(math.Round(point.Latitude * 1e5))
		lng := int64(math.Round(point.Longitude * 1e5))

		dLat := lat - lastLat
		dLng := lng - lastLng

		encode(dLat, &result)
		encode(dLng, &result)

		lastLat = lat
		lastLng = lng
	}

	return result.String()
}

func encode(v int64, result *strings.Builder) {
	v = v << 1
	if v < 0 {
		v = ^v
	}
	for v >= 0x20 {
		result.WriteByte(byte((0x20 | (v & 0x1f)) + 63))
		v >>= 5
	}
	result.WriteByte(byte(v + 63))
}

func toRadians(deg float64) float64 {
	return deg * math.Pi / 180
}

func toDegrees(rad float64) float64 {
	return rad * 180 / math.Pi
}

func wrap(n, min, max float64) float64 {
	if n >= min && n < max {
		return n
	}
	return mod(n-min, max-min) + min
}

func mod(x, m float64) float64 {
	return math.Mod(math.Mod(x, m)+m, m)
}

func mercator(lat float64) float64 {
	return math.Log(math.Tan(lat*0.5 + math.Pi/4))
}

func inverseMercator(y float64) float64 {
	return 2*math.Atan(math.Exp(y)) - math.Pi/2
}

func hav(x float64) float64 {
	sinHalf := math.Sin(x * 0.5)
	return sinHalf * sinHalf
}

func havDistance(lat1, lat2, dLng float64) float64 {
	return hav(lat1-lat2) + hav(dLng)*math.Cos(lat1)*math.Cos(lat2)
}

func sinFromHav(h float64) float64 {
	return 2 * math.Sqrt(h*(1-h))
}

func havFromSin(x float64) float64 {
	x2 := x * x
	return x2 / (1 + math.Sqrt(1-x2)) * 0.5
}

func sinSumFromHav(x, y float64) float64 {
	a := math.Sqrt(x * (1 - x))
	b := math.Sqrt(y * (1 - y))
	return 2 * (a + b - 2*(a*y+b*x))
}

func clamp(x, low, high float64) float64 {
	if x < low {
		return low
	}
	if x > high {
		return high
	}
	return x
}

func maxInt(a, b int) int {
	if a > b {
		return a
	}
	return b
}

func sinDeltaBearing(lat1, lng1, lat2, lng2, lat3, lng3 float64) float64 {
	sinLat1 := math.Sin(lat1)
	cosLat2 := math.Cos(lat2)
	cosLat3 := math.Cos(lat3)
	lat31 := lat3 - lat1
	lng31 := lng3 - lng1
	lat21 := lat2 - lat1
	lng21 := lng2 - lng1
	a := math.Sin(lng31) * cosLat3
	c := math.Sin(lng21) * cosLat2
	b := math.Sin(lat31) + 2*sinLat1*cosLat3*hav(lng31)
	d := math.Sin(lat21) + 2*sinLat1*cosLat2*hav(lng21)
	denom := (a*a + b*b) * (c*c + d*d)
	if denom <= 0 {
		return 1
	}
	return (a*d - b*c) / math.Sqrt(denom)
}
