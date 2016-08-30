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

#ifndef TRAGEDIY_TRACK_LOCATIONTABLE_H
#define TRAGEDIY_TRACK_LOCATIONTABLE_H

#include <tragediy/track/Lane.h>
#include <tragediy/util/BoundingBox.h>
#include <tragediy/util/Vector2.h>

#include <iostream>
#include <map>
#include <tuple>

class Track;

class LocationTable
{
public:
	struct LocationData
	{
		Vector2 position;
		Vector2 direction;
		std::size_t lane;
		double distance;
		bool backward;
	};

	typedef std::map<std::tuple<bool, std::size_t, uint8_t, uint8_t>, LocationData>::const_iterator const_iterator;
	typedef std::map<std::tuple<bool, std::size_t, uint8_t, uint8_t>, LocationData>::iterator iterator;

	LocationTable();
	LocationTable(const Track &track);

	auto find(bool reverse, uint8_t segment, uint8_t block) const -> const_iterator;
	auto find(bool reverse, uint8_t segment, uint8_t block) -> iterator;
	auto find(bool reverse, std::size_t numBits, uint8_t segment, uint8_t block) const -> const_iterator;
	auto find(bool reverse, std::size_t numBits, uint8_t segment, uint8_t block) -> iterator;
	auto findNext(bool clockwise, uint8_t segment, uint8_t block, const Track &track) const -> const_iterator;
	auto findNext(bool clockwise, Lane::Identifier laneId, double minPosition, const Track &track) const -> const_iterator;
	auto begin() -> iterator;
	auto end() -> iterator;
	auto begin() const -> const_iterator;
	auto end() const -> const_iterator;
	auto size() const -> std::size_t;

	void readJsonFromStream(std::istream &in, const BoundingBox &bb);

	void writeToStreamAsSvg(std::ostream &out) const;
	void writeToStreamAsCsv(std::ostream &out) const;
	void writeToStreamAsJson(std::ostream &out, const BoundingBox &bb) const;

private:
	uint8_t reverseBits(uint8_t x, std::size_t numbits) const;
	std::map<std::tuple<bool, std::size_t, uint8_t, uint8_t>, LocationData> locationTable_;
};

inline auto LocationTable::find(bool reverse, uint8_t segment, uint8_t block) const -> const_iterator
{
	return locationTable_.find(std::make_tuple(reverse, bits, segment, block));
}

inline auto LocationTable::find(bool reverse, uint8_t segment, uint8_t block) -> iterator
{
	return locationTable_.find(std::make_tuple(reverse, bits, segment, block));
}

inline auto LocationTable::find(bool reverse, std::size_t numBits, uint8_t segment, uint8_t block) const -> const_iterator
{
	return locationTable_.find(std::make_tuple(reverse, numBits, segment, block));
}

inline auto LocationTable::find(bool reverse, std::size_t numBits, uint8_t segment, uint8_t block) -> iterator
{
	return locationTable_.find(std::make_tuple(reverse, numBits, segment, block));
}

inline auto LocationTable::begin() -> iterator
{
	return locationTable_.begin();
}

inline auto LocationTable::end() -> iterator
{
	return locationTable_.end();
}

inline auto LocationTable::begin() const -> const_iterator
{
	return locationTable_.begin();
}

inline auto LocationTable::end() const -> const_iterator
{
	return locationTable_.end();
}

inline auto LocationTable::size() const -> std::size_t
{
	return locationTable_.size();
}

inline uint8_t LocationTable::reverseBits(uint8_t x, std::size_t numbits) const
{
	if (numbits == 0)
		return x;

	uint8_t result = 0x00, result_mask = 0x01;
	for (uint8_t mask = static_cast<uint8_t>(1 << (numbits - 1)); mask != 0x00; mask >>= 1)
	{
		if (x & mask)
			result |= result_mask;
		result_mask <<= 1;
	}

	return result;
}

#endif
