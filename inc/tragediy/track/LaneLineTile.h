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

#ifndef TRAGEDIY_TRACK_LANELINETILE_H
#define TRAGEDIY_TRACK_LANELINETILE_H

#include <tragediy/track/LaneTileBase.h>

class LaneLineTile : public LaneTileBase
{
public:
	LaneLineTile(const Vector2 &startPoint, const Vector2 &startDirection, double length);
	LaneLineTile(const std::shared_ptr<LaneTileBase> prevTile, double length);
	virtual ~LaneLineTile();

	auto getEndPoint() const -> Vector2;
	auto getEndDirection() const -> Vector2;

	auto getBoundingBox() const -> BoundingBox;

	void writeToStreamAsSvg(std::ostream &out, const BoundingBox &bb) const;

	std::tuple<double, double> map(const Vector2 &coordinate) const;
	std::tuple<double, double> map(const Vector2 &coordinate, double lbound, double ubound) const;

	auto getPointOnLane(double length) const -> Vector2;
	auto getDirectionOnLane(double length) const -> Vector2;

private:
	void writeLine(std::ostream &out, double from, double to, double offset, double width, std::string color = "black") const;
};

inline LaneLineTile::LaneLineTile(const Vector2 &startPoint, const Vector2 &startDirection, double length) : LaneTileBase(startPoint, startDirection, length)
{
}

inline LaneLineTile::LaneLineTile(const std::shared_ptr<LaneTileBase> prevTile, double length) : LaneTileBase(prevTile, length)
{
}

inline auto LaneLineTile::getEndPoint() const -> Vector2
{
	return startPoint_ + startDirection_ * length_;
}

inline auto LaneLineTile::getEndDirection() const -> Vector2
{
	return startDirection_;
}

inline auto LaneLineTile::getBoundingBox() const -> BoundingBox
{
	return ::getBoundingBox(startPoint_, getEndPoint());
}

#endif
