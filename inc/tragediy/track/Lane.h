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

#ifndef TRAGEDIY_TRACK_LANE_H
#define TRAGEDIY_TRACK_LANE_H

#include <tragediy/track/LaneTileBase.h>
#include <tragediy/util/BoundingBox.h>
#include <tragediy/util/Constants.h>

#include <utility>
#include <vector>

class Lane
{
public:
	typedef std::vector<std::shared_ptr<LaneTileBase>>::iterator iterator;
	typedef std::vector<std::shared_ptr<LaneTileBase>>::const_iterator const_iterator;
	typedef std::size_t Identifier;

	Lane(Identifier laneNumber);

	auto find(double positionLon) -> iterator;
	auto find(double positionLon) const -> const_iterator;
	auto begin() -> iterator;
	auto end() -> iterator;
	auto begin() const -> const_iterator;
	auto end() const -> const_iterator;
	auto size() const -> std::size_t;

	auto getLaneNumber() const -> Identifier;
	auto getBoundingBox() const -> BoundingBox;

	void addTile(const std::shared_ptr<LaneTileBase> &tile);

	double getTotalLength() const;

	auto findEx(double positionLon) -> std::tuple<iterator, double>;
	auto findEx(double positionLon) const -> std::tuple<const_iterator, double>;

	auto getPointOnLane(double length) const -> Vector2;
	auto getDirectionOnLane(double length) const -> Vector2;

	double getPositionOfTile(LaneTileBase::Identifier id) const;

	std::tuple<double, double> map(const Vector2 &coordinate) const;

	void writeToStreamAsSvg(std::ostream &out, const BoundingBox &bb) const;
	void writeAnnotationToStreamAsSvg(std::ostream &out, const BoundingBox &bb) const;

	auto operator[](LaneTileBase::Identifier id) const -> std::shared_ptr<LaneTileBase>;

private:
	Identifier laneNumber_;
	std::vector<std::shared_ptr<LaneTileBase>> tiles_;
};

typedef Lane::Identifier LaneIdentifier;

std::ostream &operator<<(std::ostream &out, const Lane &lane);

inline Lane::Lane(Identifier laneNumber) : laneNumber_(laneNumber)
{
}

inline auto Lane::getLaneNumber() const -> Identifier
{
	return laneNumber_;
}

inline void Lane::addTile(const std::shared_ptr<LaneTileBase> &tile)
{
	tiles_.push_back(tile);
}

inline auto Lane::find(double distance) -> iterator
{
	return std::get<0>(findEx(distance));
}

inline auto Lane::find(double distance) const -> const_iterator
{
	return std::get<0>(findEx(distance));
}

inline auto Lane::begin() -> iterator
{
	return tiles_.begin();
}

inline auto Lane::end() -> iterator
{
	return tiles_.end();
}

inline auto Lane::begin() const -> const_iterator
{
	return tiles_.begin();
}

inline auto Lane::end() const -> const_iterator
{
	return tiles_.end();
}

inline auto Lane::size() const -> std::size_t
{
	return tiles_.size();
}

inline auto Lane::operator[](LaneTileBase::Identifier id) const -> std::shared_ptr<LaneTileBase>
{
	return *(tiles_.begin() + id);
}

inline double Lane::getPositionOfTile(LaneTileBase::Identifier id) const
{
	throwing_assert(id < size());

	double position = 0.0;
	for (LaneTileBase::Identifier i = 0; i < id; ++i)
		position += tiles_[i]->getTotalLength();

	return position;
}

#endif
