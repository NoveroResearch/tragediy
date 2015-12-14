/*
 Copyright (c) 2015 "Novero GmbH" <http://novero.com>
 
 This file is part of tragediy <https://github.com/NoveroResearch/tragediy>.
 
 tragediy is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TRAGEDIY_UTIL_VECTOR2_H
#define TRAGEDIY_UTIL_VECTOR2_H

#include <cstddef>
#include <cmath>
#include <iostream>

class Vector2
{
private:
	double elements_[2]{0.0, 0.0};

public:
	typedef std::size_t size_type;

	Vector2();
	Vector2(double element0, double element1);

	double operator[](size_type element) const;

	Vector2 operator+(const Vector2 &x) const;
	Vector2 operator-(const Vector2 &x) const;
	Vector2 operator-() const;

	Vector2 operator*(double arg) const;
	double operator*(const Vector2 &x) const;

	Vector2 &operator+=(const Vector2 &x);
	Vector2 &operator-=(const Vector2 &x);

	bool operator==(const Vector2 &x) const;

	double getLength() const;
	Vector2 getPerpendicularVectorLeft() const;
	Vector2 getPerpendicularVectorRight() const;
};

inline Vector2::Vector2() : elements_{0.0, 0.0}
{
}

inline Vector2::Vector2(double element0, double element1) : elements_{element0, element1}
{
}

inline double Vector2::operator[](size_type element) const
{
	return elements_[element];
}

inline Vector2 Vector2::operator+(const Vector2 &arg) const
{
	return Vector2(elements_[0] + arg.elements_[0], elements_[1] + arg.elements_[1]);
}

inline Vector2 Vector2::operator-(const Vector2 &arg) const
{
	return Vector2(elements_[0] - arg.elements_[0], elements_[1] - arg.elements_[1]);
}

inline Vector2 Vector2::operator-() const
{
	return Vector2(-elements_[0], -elements_[1]);
}

inline Vector2 Vector2::operator*(double arg) const
{
	return Vector2(elements_[0] * arg, elements_[1] * arg);
}

inline double Vector2::operator*(const Vector2 &arg) const
{
	return elements_[0] * arg.elements_[0] + elements_[1] * arg.elements_[1];
}

inline Vector2 &Vector2::operator+=(const Vector2 &arg)
{
	elements_[0] += arg.elements_[0];
	elements_[1] += arg.elements_[1];

	return *this;
}

inline Vector2 &Vector2::operator-=(const Vector2 &arg)
{
	elements_[0] -= arg.elements_[0];
	elements_[1] -= arg.elements_[1];

	return *this;
}

inline bool Vector2::operator==(const Vector2 &arg) const
{
	return elements_[0] == arg.elements_[0] && elements_[1] == arg.elements_[1];
}

inline double Vector2::getLength() const
{
	return std::sqrt(elements_[0] * elements_[0] + elements_[1] * elements_[1]);
}

inline Vector2 operator*(double arg1, const Vector2 &arg2)
{
	return Vector2(arg1 * arg2[0], arg1 * arg2[1]);
}

inline Vector2 Vector2::getPerpendicularVectorRight() const
{
	return Vector2(-elements_[1], elements_[0]);
}

inline Vector2 Vector2::getPerpendicularVectorLeft() const
{
	return Vector2(elements_[1], -elements_[0]);
}

inline std::ostream &operator<<(std::ostream &out, const Vector2 &x)
{
	out << x[0] << " " << x[1];
	return out;
}

#endif
