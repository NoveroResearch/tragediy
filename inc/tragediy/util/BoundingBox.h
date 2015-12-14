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

#ifndef TRAGEDIY_UTIL_BOUNDINGBOX_H
#define TRAGEDIY_UTIL_BOUNDINGBOX_H

#include <tragediy/util/Vector2.h>
#include <algorithm>
#include <limits>

struct BoundingBox
{
	BoundingBox();
	BoundingBox(const Vector2 &p);
	BoundingBox(double xMin, double xMax, double yMin, double yMax);

	bool isEmpty() const;
	bool containsPoint(const Vector2 &p) const;

	double getWidth() const;
	double getHeight() const;

	void expand(double margin);
	BoundingBox expanded(double margin) const;

	double xMin_, xMax_, yMin_, yMax_;
};

std::ostream &operator<<(std::ostream &out, const BoundingBox &bb);

/**
 * Constructs an empty bounding box.
 */
inline BoundingBox::BoundingBox() : xMin_(std::numeric_limits<double>::infinity()), xMax_(-std::numeric_limits<double>::infinity()), yMin_(std::numeric_limits<double>::infinity()), yMax_(-std::numeric_limits<double>::infinity())
{
}

/**
 * Constructs a bounding-box including a single point.
 */
inline BoundingBox::BoundingBox(const Vector2 &p) : xMin_(p[0]), xMax_(p[0]), yMin_(p[1]), yMax_(p[1])
{
}

inline BoundingBox::BoundingBox(double xMin, double xMax, double yMin, double yMax) : xMin_(xMin), xMax_(xMax), yMin_(yMin), yMax_(yMax)
{
}

inline bool BoundingBox::isEmpty() const
{
	return xMin_ > xMax_ || yMin_ > yMax_;
}

inline bool BoundingBox::containsPoint(const Vector2 &p) const
{
	return p[0] >= xMin_ && p[1] >= yMin_ && p[0] <= xMax_ && p[1] <= yMax_;
}

inline double BoundingBox::getWidth() const
{
	return isEmpty() ? 0.0 : xMax_ - xMin_;
}

inline double BoundingBox::getHeight() const
{
	return isEmpty() ? 0.0 : yMax_ - yMin_;
}

inline void BoundingBox::expand(double margin)
{
	if (isEmpty())
		return;

	xMin_ -= margin;
	xMax_ += margin;
	yMin_ -= margin;
	yMax_ += margin;
}

inline BoundingBox BoundingBox::expanded(double margin) const
{
	BoundingBox bb(*this);
	bb.expand(margin);

	return bb;
}

/**
 * Calculates the bounding box for two points.
 */
inline BoundingBox getBoundingBox(const Vector2 &p, const Vector2 &q)
{
	return BoundingBox(std::min(p[0], q[0]), std::max(p[0], q[0]), std::min(p[1], q[1]), std::max(p[1], q[1]));
}

/**
 * Returns the bounding box of two bounding boxes.
 */
inline BoundingBox getBoundingBox(const BoundingBox &bb1, const BoundingBox &bb2)
{
	return BoundingBox(std::min(bb1.xMin_, bb2.xMin_), std::max(bb1.xMax_, bb2.xMax_), std::min(bb1.yMin_, bb2.yMin_), std::max(bb1.yMax_, bb2.yMax_));
}

inline std::ostream &operator<<(std::ostream &out, const BoundingBox &bb)
{
	out << "[" << bb.xMin_ << "; " << bb.xMax_ << "] x [" << bb.yMin_ << "; " << bb.yMax_ << "]";
	return out;
}

#endif
