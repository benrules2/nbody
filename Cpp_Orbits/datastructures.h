#pragma once
#include <string>
#include <vector>

struct point
{
	double x;
	double y;
	double z;

	point operator*(point a)
	{
		return point{ x*a.x, y*a.y, z*a.z };
	}

	point operator*(double a)
	{
		return point{ x*a, y*a, z*a };
	}

	point operator/(point a)
	{
		return point{ x/a.x, y/a.y, z/a.z };
	}

	point operator/(double a)
	{
		return point{ x / a, y / a, z / a };
	}

	point operator+(point a)
	{
		return point{ x +a.x, y + a.y, z + a.z };
	}

	point operator+=(const point& a)
	{
		x += a.x;
		y += a.y;
		z += a.z;
		return point{ x, y, z };
	}

	point operator-(point a)
	{
		return point{ x - a.x, y - a.y, z - a.z };
	}
};

struct body
{
	point location;
	double mass;
	point velocity;
	std::string name;
};
