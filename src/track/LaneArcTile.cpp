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

#include <tragediy/track/LaneArcTile.h>
#include <tragediy/util/Math.h>
#include <tragediy/util/Constants.h>

LaneArcTile::~LaneArcTile()
{
}

auto LaneArcTile::getEndPoint() const -> Vector2
{
	Vector2 midPoint = getMidPoint();

	Vector2 tmpStart = startPoint_ - midPoint;
	double angleStart = std::atan2(tmpStart[1], tmpStart[0]);
	double angleEnd = angleStart + length_ / radius_;

	return midPoint + Vector2(std::cos(angleEnd), std::sin(angleEnd)) * std::fabs(radius_);
}

auto LaneArcTile::getEndDirection() const -> Vector2
{
	Vector2 midPoint = getMidPoint();

	Vector2 tmpStart = startPoint_ - midPoint;
	double angleStart = std::atan2(tmpStart[1], tmpStart[0]);

	double angle = angleStart + length_ / radius_ + (radius_ > 0.0 ? 0.5 : -0.5) * pi<double>;

	return Vector2(std::cos(angle), std::sin(angle));
}

auto LaneArcTile::getBoundingBox() const -> BoundingBox
{
	Vector2 midPoint = getMidPoint();

	Vector2 tmpStart = startPoint_ - midPoint;
	double angleStart = std::atan2(tmpStart[1], tmpStart[0]);
	double angleEnd = angleStart + length_ / radius_;

	if (radius_ < 0.0)
		std::swap(angleStart, angleEnd);

	BoundingBox bb = ::getBoundingBox(startPoint_, getEndPoint());
	if (std::ceil((angleStart - 0.0) / (2.0 * pi<double>)) <= std::floor((angleEnd - 0.0) / (2.0 * pi<double>)))
		bb = ::getBoundingBox(bb, BoundingBox(midPoint + Vector2(1, 0) * std::fabs(radius_)));
	if (std::ceil((angleStart - 0.5 * pi<double>) / (2.0 * pi<double>)) <= std::floor((angleEnd - 0.5 * pi<double>) / (2.0 * pi<double>)))
		bb = ::getBoundingBox(bb, BoundingBox(midPoint + Vector2(0, 1) * std::fabs(radius_)));
	if (std::ceil((angleStart - pi<double>) / (2.0 * pi<double>)) <= std::floor((angleEnd - pi<double>) / (2.0 * pi<double>)))
		bb = ::getBoundingBox(bb, BoundingBox(midPoint + Vector2(-1, 0) * std::fabs(radius_)));
	if (std::ceil((angleStart - 1.5 * pi<double>) / (2.0 * pi<double>)) <= std::floor((angleEnd - 1.5 * pi<double>) / (2.0 * pi<double>)))
		bb = ::getBoundingBox(bb, BoundingBox(midPoint + Vector2(0, -1) * std::fabs(radius_)));

	return bb;
}

void LaneArcTile::writeToStreamAsSvg(std::ostream &out, const BoundingBox &bb) const
{
	Vector2 midPoint = getMidPoint();
	//Vector2 endPoint = getEndPoint();

	Vector2 tmpStart = startPoint_ - midPoint;
	double angleStart = std::atan2(tmpStart[1], tmpStart[0]);

	std::pair<double, double> last = std::make_pair(0.0, widthBaseThick);
	for (auto it = widthBase_.begin(); it != widthBase_.end(); ++it)
	{
		writeArc(out, midPoint, std::fabs(radius_), angleStart + last.first / radius_, angleStart + it->first / radius_, 0.0, last.second);
		last = *it;
	}
	writeArc(out, midPoint, std::fabs(radius_), angleStart + last.first / radius_, angleStart + length_ / radius_, 0.0, last.second);

	double current = 0.0;
	for (std::size_t i = 0; i < marks_.size(); ++i)
	{
		double length = std::get<2>(marks_[i]);

		if (current + length > getTotalLength() && (std::get<0>(marks_[i]) != MT_NONE || std::get<1>(marks_[i]) != MT_NONE))
		{
			std::cerr << "WARNING: Overfull tile." << std::endl;
			break;
		}

		double angleStartMark = angleStart + current / radius_;
		double angleEndMark = angleStart + (current + length) / radius_;

		switch (std::get<0>(marks_[i]))
		{
		case MT_NONE:
			break;
		case MT_THIN:
			writeArc(out, midPoint, radius_, angleStartMark, angleEndMark, -distThin, widthThin);
			break;
		case MT_THICK:
			writeArc(out, midPoint, radius_, angleStartMark, angleEndMark, -distThick, widthThick);
			break;
		case MT_THICKER:
			writeArc(out, midPoint, radius_, angleStartMark, angleEndMark, -distThicker, widthThicker);
			break;
		case MT_THIN_DOUBLE:
			writeArc(out, midPoint, radius_, angleStartMark, angleEndMark, -1.0 * distThinDouble, widthThinDouble);
			writeArc(out, midPoint, radius_, angleStartMark, angleEndMark, -2.0 * distThinDouble, widthThinDouble);
			break;
		case MT_THICK_DOUBLE:
			writeArc(out, midPoint, radius_, angleStartMark, angleEndMark, -1.0 * distThickDouble, widthThickDouble);
			writeArc(out, midPoint, radius_, angleStartMark, angleEndMark, -2.0 * distThickDouble, widthThickDouble);
			break;
		case MT_SEPARATOR:
		{
			double tmp = 0.5 * (distBase - widthBaseThick);
			writeArc(out, midPoint, radius_, angleStartMark, angleEndMark, -0.5 * (tmp + widthBaseThick), tmp);
			writeArc(out, midPoint, radius_, angleStartMark, angleEndMark, +0.5 * (tmp + widthBaseThick), tmp);
			writeArc(out, midPoint, radius_, angleStartMark, angleEndMark, 0.0, widthBaseThick, "white");
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
			writeArc(out, midPoint, radius_, angleStartMark, angleEndMark, +distThin, widthThin);
			break;
		case MT_THICK:
			writeArc(out, midPoint, radius_, angleStartMark, angleEndMark, +distThick, widthThick);
			break;
		case MT_THICKER:
			writeArc(out, midPoint, radius_, angleStartMark, angleEndMark, +distThicker, widthThicker);
			break;
		case MT_THIN_DOUBLE:
			writeArc(out, midPoint, radius_, angleStartMark, angleEndMark, +1.0 * distThinDouble, widthThinDouble);
			writeArc(out, midPoint, radius_, angleStartMark, angleEndMark, +2.0 * distThinDouble, widthThinDouble);
			break;
		case MT_THICK_DOUBLE:
			writeArc(out, midPoint, radius_, angleStartMark, angleEndMark, +1.0 * distThickDouble, widthThickDouble);
			writeArc(out, midPoint, radius_, angleStartMark, angleEndMark, +2.0 * distThickDouble, widthThickDouble);
			break;
		case MT_SEPARATOR:
			break;
		default:
			throwing_assert(false);
		}

		current += length;
	}
}

auto LaneArcTile::getPointOnLane(double length) const -> Vector2
{
	Vector2 midPoint = getMidPoint();

	Vector2 tmpStart = startPoint_ - midPoint;
	double angle = std::atan2(tmpStart[1], tmpStart[0]) + length / radius_;

	return midPoint + Vector2(std::cos(angle), std::sin(angle)) * std::fabs(radius_);
}

auto LaneArcTile::getDirectionOnLane(double length) const -> Vector2
{
	Vector2 midPoint = getMidPoint();

	Vector2 tmpStart = startPoint_ - midPoint;
	double angleStart = std::atan2(tmpStart[1], tmpStart[0]);

	double angle = angleStart + length / radius_ + (radius_ > 0.0 ? 0.5 : -0.5) * pi<double>;

	return Vector2(std::cos(angle), std::sin(angle));
}

void LaneArcTile::writeArc(std::ostream &out, const Vector2 &midPoint, double radius, double angleStart, double angleEnd, double offset, double width, std::string color) const
{
	if (angleEnd < angleStart)
		writeArc(out, midPoint, radius, angleEnd, angleStart, -offset, width);
	else if (angleEnd == angleStart)
		return;
	else
	{
		Vector2 startPoint = midPoint + Vector2(std::cos(angleStart), std::sin(angleStart)) * (std::fabs(radius) - offset);
		Vector2 endPoint = midPoint + Vector2(std::cos(angleEnd), std::sin(angleEnd)) * (std::fabs(radius) - offset);

		out << "<path d =\"M " << startPoint[0] << "," << startPoint[1] << " A " << std::fabs(radius) << "," << std::fabs(radius) << " 0 " << (angleEnd - angleStart > 0.5 * pi<double> ? 1 : 0) << ",1 " << endPoint[0] << "," << endPoint[1] << "\" style=\"fill:none; stroke:" << color << "; stroke-width:" << width << ";\" />\n";
	}
}
