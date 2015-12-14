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

#include <tragediy/track/LaneTileBase.h>
#include <tragediy/util/Constants.h>

LaneTileBase::~LaneTileBase()
{
}

uint8_t LaneTileBase::fillWithLocationMarkers(uint8_t segment, uint8_t firstBlockNumber, std::size_t maxBlocks, double lengthMark, double lengthGap, std::size_t numBits)
{
	for (std::size_t i = 0; i < maxBlocks; ++i)
	{
		if (firstBlockNumber == 0xff)
			return firstBlockNumber;

		if (!pushBlockOfMarks(firstBlockNumber++, segment, true, lengthMark, lengthGap, numBits))
			return firstBlockNumber - 1;

		if (!pushEndOfBlockMark(lengthMark))
			return firstBlockNumber;
	}

	return firstBlockNumber;
}

bool LaneTileBase::pushBlockOfMarks(uint8_t leftMarks, uint8_t rightMarks, bool trailingGap, double lengthMark, double lengthGap, std::size_t numBits)
{
	for (std::size_t i = 0; i < numBits; ++i)
	{
		if (!pushMark(leftMarks & (uint8_t)(1 << (numBits - 1 - i)) ? MT_THICK : MT_THIN, rightMarks & (uint8_t)(1 << (numBits - 1 - i)) ? MT_THICK : MT_THIN, (i != numBits - 1 || trailingGap), lengthMark, lengthGap))
		{
			for (std::size_t j = 0; j < i; ++j)
				popMark();

			return false;
		}
	}

	return true;
}

bool LaneTileBase::pushMark(MarkType leftMark, MarkType rightMark, bool gap, double lengthMark, double lengthGap)
{
	if (fill_ + lengthMark > length_)
		return false;

	marks_.push_back(std::make_tuple(leftMark, rightMark, lengthMark));
	fill_ += lengthMark;

	if (gap && !pushGap(lengthGap))
	{
		marks_.pop_back();
		fill_ -= lengthMark;
		return false;
	}

	return true;
}

void LaneTileBase::setWidthBase(double from, double thickness)
{
	auto it = widthBase_.begin();
	while (it != widthBase_.end() && it->first < from)
	{
		++it;
	}

	it = widthBase_.insert(it, std::make_pair(from, thickness));
	widthBase_.erase(it + 1, widthBase_.end());
}

void LaneTileBase::setWidthBase(double from, double to, double thickness)
{
	auto it = widthBase_.begin();
	double curWidth = widthBaseThick;
	while (it != widthBase_.end() && it->first < from)
	{
		curWidth = it->second;
		++it;
	}

	it = widthBase_.insert(it, std::make_pair(from, thickness));
	auto eraseFrom = ++it;
	while (it != widthBase_.end() && it->first < to)
		++it;
	it = widthBase_.erase(eraseFrom, it);

	if (it == widthBase_.end() || it->first > to)
	{
		widthBase_.insert(it, std::make_pair(to, curWidth));
	}
}
