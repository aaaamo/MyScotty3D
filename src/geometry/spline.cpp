
#include "../geometry/spline.h"

template <typename T>
T Spline<T>::at(float time) const
{

	// A4T1b: Evaluate a Catumull-Rom spline

	// Given a time, find the nearest positions & tangent values
	// defined by the control point map.

	// Transform them for use with cubic_unit_spline

	// Be wary of edge cases! What if time is before the first knot,
	// before the second knot, etc...

	if (knots.size() == 0)
	{
		return T();
	}

	auto it2 = knots.lower_bound(time);
	if (it2 == knots.begin())
	{
		return it2->second;
	}
	if (it2 == knots.end())
	{
		return std::prev(it2)->second;
	}

	float t2 = it2->first;
	T p2 = it2->second;

	auto it1 = std::prev(it2);
	float t1 = it1->first;
	T p1 = it1->second;

	float t0;
	T p0;
	if (it1 == knots.begin())
	{
		t0 = t1 - (t2 - t1);
		p0 = p1 - (p2 - p1);
	}
	else
	{
		auto it0 = std::prev(it1);
		t0 = it0->first;
		p0 = it0->second;
	}

	auto it3 = std::next(it2);
	float t3;
	T p3;
	if (it3 == knots.end())
	{
		t3 = t2 + (t2 - t1);
		p3 = p2 + (p2 - p1);
	}
	else
	{
		t3 = it3->first;
		p3 = it3->second;
	}

	const T m0 = (p2 - p0) / (t2 - t0);
	const T m1 = (p3 - p1) / (t3 - t1);

	const float norm_time = (time - t1) / (t2 - t1);

	return cubic_unit_spline(norm_time, p1, p2, m0, m1);
}

template <typename T>
T Spline<T>::cubic_unit_spline(float time, const T &position0, const T &position1,
							   const T &tangent0, const T &tangent1)
{

	// A4T1a: Hermite Curve over the unit interval

	// Given time in [0,1] compute the cubic spline coefficients and use them to compute
	// the interpolated value at time 'time' based on the positions & tangents

	// Note that Spline is parameterized on type T, which allows us to create splines over
	// any type that supports the * and + operators.

	// p(t) = a*t**3 + b*t**2 + c**t + d
	// p'(t) = 3a*t**2 + 2b*t + c

	float t3 = time * time * time;
	float t2 = time * time;

	float h00 = 2 * t3 - 3 * t2 + 1;
	float h10 = t3 - 2 * t2 + time;
	float h01 = -2 * t3 + 3 * t2;
	float h11 = t3 - t2;

	T interpolated = h00 * position0 + h10 * tangent0 + h01 * position1 + h11 * tangent1;

	return interpolated;
}

template class Spline<float>;
template class Spline<double>;
template class Spline<Vec4>;
template class Spline<Vec3>;
template class Spline<Vec2>;
template class Spline<Mat4>;
template class Spline<Spectrum>;
