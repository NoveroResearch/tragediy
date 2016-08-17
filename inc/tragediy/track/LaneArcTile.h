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

#ifndef TRAGEDIY_TRACK_LANEARCTILE_H
#define TRAGEDIY_TRACK_LANEARCTILE_H

#include <tragediy/track/LaneTileBase.h>

class LaneArcTile : public LaneTileBase
{
public:
	LaneArcTile(const Vector2 &startPoint, const Vector2 &startDirection, double radius, double length);
	LaneArcTile(const std::shared_ptr<LaneTileBase> prevTile, double radius, double length);
	virtual ~LaneArcTile();

	auto getMidPoint() const -> Vector2;
	auto getEndPoint() const -> Vector2;
	auto getEndDirection() const -> Vector2;

	auto getBoundingBox() const -> BoundingBox;

	void writeToStreamAsSvg(std::ostream &out, const BoundingBox &bb) const;

	auto map(const Vector2 &coordinate) const -> std::tuple<double, double>;
	auto map(const Vector2 &coordinate, double lbound, double ubound) const -> std::tuple<double, double>;

	auto getPointOnLane(double length) const -> Vector2;
	auto getDirectionOnLane(double length) const -> Vector2;

	auto getRadius() const -> double;

private:
	void writeArc(std::ostream &out, const Vector2 &midPoint, double radius, double angleStart, double angleEnd, double offset, double width, std::string color = "black") const;

	double radius_;
};

/**
 * Constructs a segment of a circle (arc). The arc starts at startPoint in the direction of startDirection. The circle's radius can be positive corresponding to a right turn or it can be negative in which case the arc corresponds to a left turn.
 */
inline LaneArcTile::LaneArcTile(const Vector2 &startPoint, const Vector2 &startDirection, double radius, double length) : LaneTileBase(startPoint, startDirection, length), radius_(radius)
{
}

/**
 * Constructs a segment of a circle (arc). The arc starts at the end point of the previous tile matching the direction of at the end point of the previous tile. The circle's radius can be positive corresponding to a right turn or it can be negative in which case the arc corresponds to a left turn.
 */
inline LaneArcTile::LaneArcTile(const std::shared_ptr<LaneTileBase> prevTile, double radius, double length) : LaneTileBase(prevTile, length), radius_(radius)
{
}

inline auto LaneArcTile::getMidPoint() const -> Vector2
{
	return startPoint_ + startDirection_.getPerpendicularVectorRight() * radius_;
}

inline auto LaneArcTile::getRadius() const -> double
{
	return radius_;
}

#endif
