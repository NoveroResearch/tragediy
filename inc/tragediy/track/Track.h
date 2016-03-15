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

#ifndef TRAGEDIY_TRACK_TRACK_H
#define TRAGEDIY_TRACK_TRACK_H

#include <tragediy/track/Lane.h>
#include <tragediy/track/LocationTable.h>
#include <tragediy/util/BoundingBox.h>

#include <boost/filesystem.hpp>

#include <fstream>
#include <vector>

class Track
{
public:
	typedef std::vector<std::shared_ptr<Lane>>::iterator iterator;
	typedef std::vector<std::shared_ptr<Lane>>::const_iterator const_iterator;

	auto getBoundingBox() const -> const BoundingBox &;
	auto adaptCanvas() -> const BoundingBox &;

	auto find(Lane::Identifier laneId) const -> const_iterator;
	auto begin() const -> const_iterator;
	auto end() const -> const_iterator;
	auto size() const -> std::size_t;
	auto at(Lane::Identifier laneIdentifier) const -> const std::shared_ptr<Lane>&;

	void addLane(const Lane &lane);

	void readJsonFromStream(std::istream &in);

	void writeToStreamAsSvg(std::ostream &out, const BoundingBox &bb) const;
	void writeAnnotationToStreamAsSvg(std::ostream &out, const BoundingBox &bb) const;
	void writeToStreamAsJson(std::ostream &out, const BoundingBox &bb) const;

	auto map(const Vector2 &coordinate) const -> std::tuple<std::size_t, double, double>;

private:
	std::vector<std::shared_ptr<Lane>> lanes_;
	BoundingBox bb_;
};

void constructRingTrack(Track &track, Track &borders, double radius, double len, int n, bool withBorders = false);
void constructStarterTrack(Track &track);

inline auto Track::getBoundingBox() const -> const BoundingBox &
{
	return bb_;
}

inline void Track::addLane(const Lane &lane)
{
	// TODO check id etc. before adding
	lanes_.push_back(std::make_shared<Lane>(lane));
}

inline auto Track::at(Lane::Identifier laneIdentifier) const -> const std::shared_ptr<Lane>&
{
	auto it = find(laneIdentifier);
	if (it == end())
		throw std::out_of_range("Could not find lane identifier in track data structure.");

	return *it;
}

inline auto Track::find(Lane::Identifier laneId) const -> const_iterator
{
	return std::find_if(lanes_.begin(), lanes_.end(), [laneId](const std::shared_ptr<const Lane> &lane) {
		return lane->getLaneNumber() == laneId;
	});
}

inline auto Track::begin() const -> const_iterator
{
	return lanes_.begin();
}

inline auto Track::end() const -> const_iterator
{
	return lanes_.end();
}

inline auto Track::size() const -> std::size_t
{
	return lanes_.size();
}

#endif
