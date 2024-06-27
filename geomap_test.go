package geomap_test

import (
	"math"
	"testing"

	"github.com/sidwebworks/geomap-utils"
)

const EARTH_RADIUS = geomap.DEFAULT_EARTH_RADIUS

func TestSphericalUtilComputeHeading(t *testing.T) {
	instance := geomap.NewSphericalUtil(EARTH_RADIUS)

	from := geomap.LatLng{Latitude: 25.775, Longitude: -80.19}
	to := geomap.LatLng{Latitude: 21.774, Longitude: -80.19}

	result := instance.ComputeHeading(
		from, to,
	)

	if result != -180 {
		t.Errorf("Expected -180 but got %f", result)
	}
}

func TestSphericalUtilComputeDistanceBetween(t *testing.T) {
	instance := geomap.NewSphericalUtil(EARTH_RADIUS)

	from := geomap.LatLng{Latitude: 25.775, Longitude: -80.19}
	to := geomap.LatLng{Latitude: 21.774, Longitude: -80.19}

	result := instance.ComputeDistanceBetween(
		from, to,
	)

	const expected = float64(444891.52998049)

	if math.Abs(result-expected) > 0.1 {
		t.Errorf("Expected 444891.529980 but got %f", result)
	}
}

func TestPolyUtilIsLocationOnEdge(t *testing.T) {
	instance := geomap.NewPolyUtil(EARTH_RADIUS)

	point := geomap.LatLng{Latitude: 25.774, Longitude: -80.19}

	polygon := []geomap.LatLng{
		{Latitude: 25.774, Longitude: -80.19},
		{Latitude: 18.466, Longitude: -66.118},
		{Latitude: 32.321, Longitude: -64.757},
	}

	result := instance.IsLocationOnEdgeWithDefaultTolerance(
		point, polygon, true,
	)

	if !result {
		t.Errorf("Expected true but got false")
	}
}

func TestPolyUtilIsLocationOnPath(t *testing.T) {
	instance := geomap.NewPolyUtil(EARTH_RADIUS)

	point := geomap.LatLng{Latitude: 25.771, Longitude: -80.19}

	polygon := []geomap.LatLng{
		{Latitude: 25.774, Longitude: -80.19},
		{Latitude: 18.466, Longitude: -66.118},
		{Latitude: 32.321, Longitude: -64.757},
	}

	result := instance.IsLocationOnPathWithDefaultTolerance(
		point, polygon, true,
	)

	if result {
		t.Errorf("Expected false but got true")
	}
}

func TestPolyUtilContainsLocation(t *testing.T) {
	instance := geomap.NewPolyUtil(EARTH_RADIUS)

	point := geomap.LatLng{Latitude: 23.886, Longitude: -65.269}

	polygon := []geomap.LatLng{
		{Latitude: 25.774, Longitude: -80.19},
		{Latitude: 18.466, Longitude: -66.118},
		{Latitude: 32.321, Longitude: -64.757},
	}

	result := instance.ContainsLocation(
		point.Latitude, point.Latitude, polygon, true,
	)

	if result {
		t.Errorf("Expected false but got true")
	}
}

func TestPolyUtilDistanceToLine(t *testing.T) {
	instance := geomap.NewPolyUtil(EARTH_RADIUS)

	point := geomap.LatLng{Latitude: 61.387002, Longitude: 23.890636}
	start := geomap.LatLng{Latitude: 61.487002, Longitude: 23.790636}
	end := geomap.LatLng{Latitude: 60.48047, Longitude: 22.052754}

	result := instance.DistanceToLine(
		point, start, end,
	)

	const expected = float64(12325.124046196)

	if math.Abs(result-expected) > 0.1 {
		t.Errorf("Expected 12325.124046196 but got %f", result)
	}
}

func TestPolyUtilEncode(t *testing.T) {
	instance := geomap.NewPolyUtil(EARTH_RADIUS)

	result := instance.Encode([]geomap.LatLng{
		{Latitude: 38.5, Longitude: -120.2},
		{Latitude: 40.7, Longitude: -120.95},
		{Latitude: 43.252, Longitude: -126.453},
	})

	const expected = "_p~iF~ps|U_ulLnnqC_mqNvxq`@"

	if result != expected {
		t.Errorf("Expected %s but got %s", expected, result)
	}

}

func TestPolyUtilDecode(t *testing.T) {
	instance := geomap.NewPolyUtil(EARTH_RADIUS)

	result := instance.Decode("_p~iF~ps|U_ulLnnqC_mqNvxq`@")

	expected := []geomap.LatLng{
		{Latitude: 38.5, Longitude: -120.2},
		{Latitude: 40.7, Longitude: -120.95},
		{Latitude: 43.252, Longitude: -126.453},
	}

	if len(result) != len(expected) {
		t.Errorf("Expected decoded length of %v but got %v", expected, result)
	}

	for i, v := range result {
		if math.Abs(v.Latitude-expected[i].Latitude) > 0.1 {
			t.Errorf("Expected latitude %v but got %v", expected[i].Latitude, v.Latitude)
		}

		if math.Abs(v.Longitude-expected[i].Longitude) > 0.1 {
			t.Errorf("Expected longitude %v but got %v", expected[i].Longitude, v.Longitude)
		}
	}
}
