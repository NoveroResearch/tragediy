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

#include <tragediy/track/LaneLineTile.h>
#include <tragediy/util/Constants.h>

LaneLineTile::~LaneLineTile()
{
}

void LaneLineTile::writeToStreamAsSvg(std::ostream &out, const BoundingBox &bb) const
{
	std::pair<double, double> last = std::make_pair(0.0, widthBaseThick);
	for (auto it = widthBase_.begin(); it != widthBase_.end(); ++it)
	{
		writeLine(out, last.first, it->first, 0.0, last.second);
		last = *it;
	}
	writeLine(out, last.first, getTotalLength(), 0.0, last.second);

	double current = 0.0;
	for (std::size_t i = 0; i < marks_.size(); ++i)
	{
		double length = std::get<2>(marks_[i]);

		if (current + length > getTotalLength() && (std::get<0>(marks_[i]) != MT_NONE || std::get<1>(marks_[i]) != MT_NONE))
		{
			std::cerr << "WARNING: Overfull tile." << std::endl;
			break;
		}

		switch (std::get<0>(marks_[i]))
		{
		case MT_NONE:
			break;
		case MT_THIN:
			writeLine(out, current, current + length, -distThin, widthThin);
			break;
		case MT_THICK:
			writeLine(out, current, current + length, -distThick, widthThick);
			break;
		case MT_THICKER:
			writeLine(out, current, current + length, -distThicker, widthThicker);
			break;
		case MT_THIN_DOUBLE:
			writeLine(out, current, current + length, -1.0 * distThinDouble, widthThinDouble);
			writeLine(out, current, current + length, -2.0 * distThinDouble, widthThinDouble);
			break;
		case MT_THICK_DOUBLE:
			writeLine(out, current, current + length, -1.0 * distThickDouble, widthThickDouble);
			writeLine(out, current, current + length, -2.0 * distThickDouble, widthThickDouble);
			break;
		case MT_SEPARATOR:
		{
			double tmp = 0.5 * (distBase - widthBaseThick);
			writeLine(out, current, current + length, -0.5 * (tmp + widthBaseThick), tmp * 1.1);
			writeLine(out, current, current + length, +0.5 * (tmp + widthBaseThick), tmp * 1.1);
			writeLine(out, current, current + length, 0.0, widthBaseThick, "white");

			break;
		}
		default:
			throwing_assert(false);
		}

		switch (std::get<1>(marks_[i]))
		{
		case MT_NONE:
			break;
		case MT_THIN:
			writeLine(out, current, current + length, +distThin, widthThin);
			break;
		case MT_THICK:
			writeLine(out, current, current + length, +distThick, widthThick);
			break;
		case MT_THICKER:
			writeLine(out, current, current + length, +distThicker, widthThicker);
			break;
		case MT_THIN_DOUBLE:
			writeLine(out, current, current + length, +1.0 * distThinDouble, widthThinDouble);
			writeLine(out, current, current + length, +2.0 * distThinDouble, widthThinDouble);
			break;
		case MT_THICK_DOUBLE:
			writeLine(out, current, current + length, +1.0 * distThickDouble, widthThickDouble);
			writeLine(out, current, current + length, +2.0 * distThickDouble, widthThickDouble);
			break;
		case MT_SEPARATOR:
			break;
		default:
			throwing_assert(false);
		}

		current += length;
	}
}

auto LaneLineTile::map(const Vector2 &coordinate) const -> std::tuple<double, double>
{
	Vector2 p = coordinate - startPoint_;
	double distance = p * startDirection_, error;

	if (distance < 0.0)
	{
		// start point is nearest to coordinate
		error = p.getLength();
		distance = 0.0;
	}
	else if (distance > length_)
	{
		// end point is nearest to coordinate
		error = (coordinate - getEndPoint()).getLength();
		distance = length_;
	}
	else
	{
		error = (p - distance * startDirection_).getLength();
	}

	return std::make_tuple(distance, error);
}

auto LaneLineTile::map(const Vector2 &coordinate, double lbound, double ubound) const -> std::tuple<double, double>
{
	if (lbound > ubound)
		return std::make_tuple(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());

	// Clip range to tile.
	lbound = std::max(0.0, lbound);
	ubound = std::min(length_, ubound);

	Vector2 p = coordinate - startPoint_;
	double distance = p * startDirection_, error;

	if (distance < lbound)
	{
		// start point is nearest to coordinate
		error = (coordinate - getPointOnLane(lbound)).getLength();
		distance = lbound;
	}
	else if (distance > ubound)
	{
		// end point is nearest to coordinate
		error = (coordinate - getPointOnLane(ubound)).getLength();
		distance = ubound;
	}
	else
	{
		error = (p - distance * startDirection_).getLength();
	}

	return std::make_tuple(distance, error);
}

auto LaneLineTile::getPointOnLane(double length) const -> Vector2
{
	return getStartPoint() + getStartDirection() * length;
}

auto LaneLineTile::getDirectionOnLane(double length) const -> Vector2
{
	return startDirection_;
}

void LaneLineTile::writeLine(std::ostream &out, double from, double to, double offset, double width, std::string color) const
{
	if (from >= to)
		return;

	Vector2 orthogonal = getStartDirection().getPerpendicularVectorRight();
	Vector2 p = getStartPoint() + getStartDirection() * from + orthogonal * offset;
	Vector2 q = getStartPoint() + getStartDirection() * to + orthogonal * offset;

	out << "<line x1=\"" << p[0] << "\" y1=\"" << p[1] << "\" x2=\"" << q[0] << "\" y2=\"" << q[1] << "\" style=\"stroke:" << color << "; stroke-width:" << width << ";\" />\n";
}
