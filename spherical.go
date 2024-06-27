package geomap

import (
	"math"
)

// SphericalUtil contains utility functions for spherical calculations
type SphericalUtil struct {
	EarthRadius float64
}

// NewSphericalUtil creates a new SphericalUtil instance with the specified earth radius
func NewSphericalUtil(earthRadius float64) *SphericalUtil {
	return &SphericalUtil{
		EarthRadius: earthRadius,
	}
}

// ComputeHeading returns the heading from one LatLng to another LatLng
func (su *SphericalUtil) ComputeHeading(from, to LatLng) float64 {
	fromLat := toRadians(from.Latitude)
	fromLng := toRadians(from.Longitude)
	toLat := toRadians(to.Latitude)
	toLng := toRadians(to.Longitude)
	dLng := toLng - fromLng
	heading := math.Atan2(
		math.Sin(dLng)*math.Cos(toLat),
		math.Cos(fromLat)*math.Sin(toLat)-math.Sin(fromLat)*math.Cos(toLat)*math.Cos(dLng))
	return wrap(toDegrees(heading), -180, 180)
}

// ComputeOffset returns the LatLng resulting from moving a distance from an origin in the specified heading
func (su *SphericalUtil) ComputeOffset(from LatLng, distance, heading float64) LatLng {
	distance /= su.EarthRadius
	heading = toRadians(heading)
	fromLat := toRadians(from.Latitude)
	fromLng := toRadians(from.Longitude)
	cosDistance := math.Cos(distance)
	sinDistance := math.Sin(distance)
	sinFromLat := math.Sin(fromLat)
	cosFromLat := math.Cos(fromLat)
	sinLat := cosDistance*sinFromLat + sinDistance*cosFromLat*math.Cos(heading)
	dLng := math.Atan2(
		sinDistance*cosFromLat*math.Sin(heading),
		cosDistance-sinFromLat*sinLat)
	return LatLng{toDegrees(math.Asin(sinLat)), toDegrees(fromLng + dLng)}
}

// ComputeOffsetOrigin returns the location of origin when provided with a LatLng destination, meters travelled and original heading
func (su *SphericalUtil) ComputeOffsetOrigin(to LatLng, distance, heading float64) *LatLng {
	heading = toRadians(heading)
	distance /= su.EarthRadius
	n1 := math.Cos(distance)
	n2 := math.Sin(distance) * math.Cos(heading)
	n3 := math.Sin(distance) * math.Sin(heading)
	n4 := math.Sin(toRadians(to.Latitude))
	n12 := n1 * n1
	discriminant := n2*n2*n12 + n12*n12 - n12*n4*n4
	if discriminant < 0 {
		return nil
	}
	b := n2*n4 + math.Sqrt(discriminant)
	b /= n1*n1 + n2*n2
	a := (n4 - n2*b) / n1
	fromLatRadians := math.Atan2(a, b)
	if fromLatRadians < -math.Pi/2 || fromLatRadians > math.Pi/2 {
		b = n2*n4 - math.Sqrt(discriminant)
		b /= n1*n1 + n2*n2
		fromLatRadians = math.Atan2(a, b)
	}
	if fromLatRadians < -math.Pi/2 || fromLatRadians > math.Pi/2 {
		return nil
	}
	fromLngRadians := toRadians(to.Longitude) - math.Atan2(n3, n1*math.Cos(fromLatRadians)-n2*math.Sin(fromLatRadians))
	return &LatLng{toDegrees(fromLatRadians), toDegrees(fromLngRadians)}
}

// Interpolate returns the LatLng which lies the given fraction of the way between the origin LatLng and the destination LatLng
func (su *SphericalUtil) Interpolate(from, to LatLng, fraction float64) LatLng {
	fromLat := toRadians(from.Latitude)
	fromLng := toRadians(from.Longitude)
	toLat := toRadians(to.Latitude)
	toLng := toRadians(to.Longitude)
	cosFromLat := math.Cos(fromLat)
	cosToLat := math.Cos(toLat)
	angle := computeAngleBetween(from, to)
	sinAngle := math.Sin(angle)
	if sinAngle < 1e-6 {
		return LatLng{
			from.Latitude + fraction*(to.Latitude-from.Latitude),
			from.Longitude + fraction*(to.Longitude-from.Longitude),
		}
	}
	a := math.Sin((1-fraction)*angle) / sinAngle
	b := math.Sin(fraction*angle) / sinAngle
	x := a*cosFromLat*math.Cos(fromLng) + b*cosToLat*math.Cos(toLng)
	y := a*cosFromLat*math.Sin(fromLng) + b*cosToLat*math.Sin(toLng)
	z := a*math.Sin(fromLat) + b*math.Sin(toLat)
	lat := math.Atan2(z, math.Sqrt(x*x+y*y))
	lng := math.Atan2(y, x)
	return LatLng{toDegrees(lat), toDegrees(lng)}
}

// ComputeDistanceBetween returns the distance between two LatLngs, in meters
func (su *SphericalUtil) ComputeDistanceBetween(from, to LatLng) float64 {
	return computeAngleBetween(from, to) * su.EarthRadius
}

// ComputeLength returns the length of the given path, in meters, on Earth
func (su *SphericalUtil) ComputeLength(path []LatLng) float64 {
	if len(path) < 2 {
		return 0
	}
	length := 0.0
	var prev *LatLng
	for _, point := range path {
		if prev != nil {
			prevLat := toRadians(prev.Latitude)
			prevLng := toRadians(prev.Longitude)
			lat := toRadians(point.Latitude)
			lng := toRadians(point.Longitude)
			length += distanceRadians(prevLat, prevLng, lat, lng)
		}
		prev = &point
	}
	return length * su.EarthRadius
}

// ComputeArea returns the area of a closed path on Earth
func (su *SphericalUtil) ComputeArea(path []LatLng) float64 {
	return math.Abs(su.ComputeSignedArea(path))
}

// ComputeSignedArea returns the signed area of a closed path on Earth
func (su *SphericalUtil) ComputeSignedArea(path []LatLng) float64 {
	return su.computeSignedAreaWithRadius(path, su.EarthRadius)
}

// computeSignedAreaWithRadius returns the signed area of a closed path on a sphere of given radius
func (su *SphericalUtil) computeSignedAreaWithRadius(path []LatLng, radius float64) float64 {
	size := len(path)
	if size < 3 {
		return 0
	}
	total := 0.0
	prev := path[size-1]
	prevTanLat := math.Tan((math.Pi/2 - toRadians(prev.Latitude)) / 2)
	prevLng := toRadians(prev.Longitude)
	for _, point := range path {
		tanLat := math.Tan((math.Pi/2 - toRadians(point.Latitude)) / 2)
		lng := toRadians(point.Longitude)
		total += polarTriangleArea(tanLat, lng, prevTanLat, prevLng)
		prevTanLat = tanLat
		prevLng = lng
	}
	return total * (radius * radius)
}

// polarTriangleArea returns the signed area of a triangle which has North Pole as a vertex
func polarTriangleArea(tan1, lng1, tan2, lng2 float64) float64 {
	deltaLng := lng1 - lng2
	t := tan1 * tan2
	return 2 * math.Atan2(t*math.Sin(deltaLng), 1+t*math.Cos(deltaLng))
}

func computeAngleBetween(from, to LatLng) float64 {
	return distanceRadians(toRadians(from.Latitude), toRadians(from.Longitude), toRadians(to.Latitude), toRadians(to.Longitude))
}

func distanceRadians(lat1, lng1, lat2, lng2 float64) float64 {
	return arcHav(havDistance(lat1, lat2, lng1-lng2))
}

func arcHav(x float64) float64 {
	return 2 * math.Asin(math.Sqrt(x))
}
