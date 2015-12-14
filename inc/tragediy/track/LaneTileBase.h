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

#ifndef TRAGEDIY_TRACK_LANETILEBASE_H
#define TRAGEDIY_TRACK_LANETILEBASE_H

#include <tragediy/util/Vector2.h>
#include <tragediy/util/BoundingBox.h>
#include <tragediy/util/AssertionError.h>
#include <tragediy/util/Constants.h>

#include <vector>
#include <memory>

enum MarkType
{
	MT_NONE,
	MT_THIN,
	MT_THICK,
	MT_THICKER,
	MT_THIN_DOUBLE,
	MT_THICK_DOUBLE,
	MT_SEPARATOR
};

class LaneTileBase
{
public:
	typedef std::size_t Identifier;

	LaneTileBase(const Vector2 &startPoint, const Vector2 &startDirection, double length);
	LaneTileBase(const std::shared_ptr<LaneTileBase> prevTile, double length);
	virtual ~LaneTileBase();

	void fillWithEndOfBlockMarks();
	uint8_t fillWithLocationMarkers(uint8_t segment, uint8_t firstBlockNumber, std::size_t maxBlocks = 256, double lengthMark = lenMark, double lengthGap = lenGap, std::size_t numBits = bits);

	bool pushBlockOfMarks(uint8_t leftMarks, uint8_t rightMarks, bool trailingGap = true, double lengthMark = lenMark, double lengthGap = lenGap, std::size_t numBits = bits);
	void popBlockOfMarks();

	bool pushEndOfBlockMark(double lengthMark = lenMark);
	void popEndOfBlockMark();

	bool pushMark(MarkType leftMark, MarkType rightMark, bool gap = true, double lengthMark = lenMark, double lengthGap = lenGap);
	void popMark();

	bool pushGap(double length);
	void popGap();

	void setWidthBase(double from, double thickness);
	void setWidthBase(double from, double to, double thickness);

	auto getFillLength() const -> double;
	auto getTotalLength() const -> double;
	auto getStartPoint() const -> Vector2;
	auto getStartDirection() const -> Vector2;

	virtual void writeToStreamAsSvg(std::ostream &out, const BoundingBox &bb) const = 0;
	virtual Vector2 getPointOnLane(double length) const = 0;
	virtual Vector2 getDirectionOnLane(double length) const = 0;
	virtual Vector2 getEndPoint() const = 0;
	virtual Vector2 getEndDirection() const = 0;
	virtual BoundingBox getBoundingBox() const = 0;

	Vector2 startPoint_;
	Vector2 startDirection_;
	double length_;

	std::vector<std::tuple<MarkType, MarkType, double>> marks_;
	std::vector<std::pair<double, double>> widthBase_;

	double fill_ = 0.0;
};

inline std::ostream &operator<<(std::ostream &out, const LaneTileBase &tile)
{
	tile.writeToStreamAsSvg(out, BoundingBox(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()));
	return out;
}

inline LaneTileBase::LaneTileBase(const Vector2 &startPoint, const Vector2 &startDirection, double length) : startPoint_(startPoint), startDirection_(startDirection), length_(length)
{
	throwing_assert(length_ > 0.0);

	double len = startDirection_.getLength();
	throwing_assert(len > 0.0);

	startDirection_ = startDirection_ * (1.0 / len);
}

inline LaneTileBase::LaneTileBase(const std::shared_ptr<LaneTileBase> prevTile, double length) : startPoint_(prevTile->getEndPoint()), startDirection_(prevTile->getEndDirection()), length_(length)
{
	throwing_assert(length_ > 0.0);
}

inline void LaneTileBase::fillWithEndOfBlockMarks()
{
	while (pushEndOfBlockMark())
		;
}

inline void LaneTileBase::popBlockOfMarks()
{
	for (std::size_t i = 0; i < 8; ++i)
		popMark();
}

inline bool LaneTileBase::pushEndOfBlockMark(double lengthMark)
{
	return pushMark(MT_THICKER, MT_THIN, true, lengthMark, lengthMark);
}

inline void LaneTileBase::popEndOfBlockMark()
{
	popMark();
}

inline void LaneTileBase::popMark()
{
	fill_ -= std::get<2>(marks_.back());
	marks_.pop_back();
	popGap();
}

inline bool LaneTileBase::pushGap(double length)
{
	if (fill_ + length > length_)
		return false;

	marks_.push_back(std::make_tuple(MT_NONE, MT_NONE, length));
	fill_ += length;
	return true;
}

inline void LaneTileBase::popGap()
{
	fill_ -= std::get<2>(marks_.back());
	marks_.pop_back();
}

inline double LaneTileBase::getFillLength() const
{
	return fill_;
}

inline double LaneTileBase::getTotalLength() const
{
	return length_;
}

inline auto LaneTileBase::getStartPoint() const -> Vector2
{
	return startPoint_;
}

inline auto LaneTileBase::getStartDirection() const -> Vector2
{
	return startDirection_;
}

#endif
