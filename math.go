package geomap

import (
	"math"
)

// MathUtil contains utility functions used by both PolyUtil and SphericalUtil
type MathUtil struct {
	EarthRadius float64
}

// NewMathUtil creates a new MathUtil instance with the specified earth radius
func NewMathUtil(earthRadius float64) *MathUtil {
	return &MathUtil{
		EarthRadius: earthRadius,
	}
}

func (mu *MathUtil) FloatEqual(a, b float64) bool {
	return math.Abs(a-b) > 0.0
}

// Clamp restricts x to the range [low, high]
func (mu *MathUtil) Clamp(x, low, high float64) float64 {
	if x < low {
		return low
	}
	if x > high {
		return high
	}
	return x
}

// Wrap wraps the given value into the inclusive-exclusive interval between min and max
func (mu *MathUtil) Wrap(n, min, max float64) float64 {
	if n >= min && n < max {
		return n
	}
	return mu.Mod(n-min, max-min) + min
}

// Mod returns the non-negative remainder of x / m
func (mu *MathUtil) Mod(x, m float64) float64 {
	return math.Mod(math.Mod(x, m)+m, m)
}

// Mercator returns mercator Y corresponding to latitude
func (mu *MathUtil) Mercator(lat float64) float64 {
	return math.Log(math.Tan(lat*0.5 + math.Pi/4))
}

// InverseMercator returns latitude from mercator Y
func (mu *MathUtil) InverseMercator(y float64) float64 {
	return 2*math.Atan(math.Exp(y)) - math.Pi/2
}

// Hav returns haversine(angle-in-radians)
func (mu *MathUtil) Hav(x float64) float64 {
	sinHalf := math.Sin(x * 0.5)
	return sinHalf * sinHalf
}

// ArcHav computes inverse haversine
func (mu *MathUtil) ArcHav(x float64) float64 {
	return 2 * math.Asin(math.Sqrt(x))
}

// SinFromHav given h==hav(x), returns sin(abs(x))
func (mu *MathUtil) SinFromHav(h float64) float64 {
	return 2 * math.Sqrt(h*(1-h))
}

// HavFromSin returns hav(asin(x))
func (mu *MathUtil) HavFromSin(x float64) float64 {
	x2 := x * x
	return x2 / (1 + math.Sqrt(1-x2)) * 0.5
}

// SinSumFromHav returns sin(arcHav(x) + arcHav(y))
func (mu *MathUtil) SinSumFromHav(x, y float64) float64 {
	a := math.Sqrt(x * (1 - x))
	b := math.Sqrt(y * (1 - y))
	return 2 * (a + b - 2*(a*y+b*x))
}

// HavDistance returns hav() of distance from (lat1, lng1) to (lat2, lng2) on the unit sphere
func (mu *MathUtil) HavDistance(lat1, lat2, dLng float64) float64 {
	return mu.Hav(lat1-lat2) + mu.Hav(dLng)*math.Cos(lat1)*math.Cos(lat2)
}
