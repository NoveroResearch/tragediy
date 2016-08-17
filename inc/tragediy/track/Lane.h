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

	struct Distance
	{
		Distance();
		Distance(const std::shared_ptr<const Lane> &lane, int laps, double distanceLon, double distanceLat);

		std::shared_ptr<const Lane> lane_;
		int laps_{0};
		double distanceLon_{0.0};
		double distanceLat_{0.0};

		double getDistanceLon() const;
		double getDistanceLat() const;

		double getMinimumForwardDistance(bool clockwise) const;
		double getMinimumSignedDistance() const;

		int getLaps() const;
	};

	struct Position
	{
		Position();
		Position(const std::shared_ptr<const Lane> &lane, double positionLon);

		int lap_{0};
		std::shared_ptr<const Lane> lane_;
		double positionLon_{0.0 / 0.0};
		double positionLat_{0.0};

		std::shared_ptr<const Lane> getReferenceLane() const;

		double getLongitudinalPosition() const;
		double getNonperiodicLongitudinalPosition() const;

		double getLateralPositionRelative() const;
		double getLateralPositionAbsolute() const;

		Vector2 getPoint() const;
		Vector2 getPointOnLane() const;
		Vector2 getDirection(bool clockwise) const;
		double getDirectionAngle(bool clockwise) const;

		Lane::const_iterator findTile() const;
		LaneTileBase::Identifier getTileIdentifier() const;

		void forward(bool clockwise, double distance);
		Position forwarded(bool clockwise, double distance) const;

		void shiftLaterallyBy(double offset);
		Position shiftedLaterallyBy(double offset) const;

		void shiftLaterallyTo(double offset);
		Position shiftedLaterallyTo(double offset) const;

		void normalize();
		Position normalized() const;

		Position remapped(const std::shared_ptr<const Lane> &lane) const;
		void remap(const std::shared_ptr<const Lane> &lane);
		std::tuple<std::shared_ptr<LaneTileBase>, std::shared_ptr<LaneTileBase>> remapEx(const std::shared_ptr<const Lane> &lane);

		Distance operator-(Position position) const;

		Position relapped(const Lane::Position &position, bool clockwise, bool behind) const;
		void relap(const Position &position, bool clockwise, bool behind);
	};

	Lane(Identifier laneNumber);

	auto find(double positionLon) -> iterator;
	auto find(double positionLon) const -> const_iterator;
	auto begin() -> iterator;
	auto end() -> iterator;
	auto begin() const -> const_iterator;
	auto end() const -> const_iterator;
	auto size() const -> std::size_t;

	auto getLaneNumber() const -> Identifier;
	auto getLateralPositionAbsolute() const -> double;
	auto getBoundingBox() const -> BoundingBox;

	void addTile(const std::shared_ptr<LaneTileBase> &tile);

	double getTotalLength() const;

	auto findEx(double positionLon) -> std::tuple<iterator, double>;
	auto findEx(double positionLon) const -> std::tuple<const_iterator, double>;

	auto getPointOnLane(double length) const -> Vector2;
	auto getDirectionOnLane(double length) const -> Vector2;

	double getPositionOfTile(LaneTileBase::Identifier id) const;

	std::tuple<double, double> map(const Vector2 &coordinate) const;
	std::tuple<double, double> map(const Vector2 &coordinate, double lbound, double ubound) const;

	std::tuple<double, double> mapConstrained(const Vector2 &coordinate, const Vector2 &direction) const;

	std::tuple<double, double, double> mapSigned(const Vector2 &coordinate) const;
	std::tuple<double, double, double> mapSigned(const Vector2 &coordinate, double lbound, double ubound) const;

	auto unmapSigned(double distance, double offset) const -> Vector2;

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

inline auto Lane::getLateralPositionAbsolute() const -> double
{
	return laneNumber_ * distBase;
}

inline auto Lane::unmapSigned(double distance, double offset) const -> Vector2
{
	return getPointOnLane(distance) + getDirectionOnLane(distance).getPerpendicularVectorRight() * offset;
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

inline Lane::Position::Position()
{
}

inline Lane::Position::Position(const std::shared_ptr<const Lane> &lane, double positionLon) : lane_(lane), positionLon_(positionLon)
{
	positionLon_ = std::fmod(positionLon_, lane_->getTotalLength());
	if (positionLon_ < 0.0)
		positionLon_ += lane_->getTotalLength();
}

inline std::shared_ptr<const Lane> Lane::Position::getReferenceLane() const
{
	return lane_;
}

inline double Lane::Position::getLongitudinalPosition() const
{
	return positionLon_;
}

inline double Lane::Position::getNonperiodicLongitudinalPosition() const
{
	return positionLon_ + lap_ * lane_->getTotalLength();
}

inline double Lane::Position::getLateralPositionRelative() const
{
	return positionLat_;
}

inline double Lane::Position::getLateralPositionAbsolute() const
{
	return positionLat_ + lane_->getLaneNumber() * distBase;
}

inline auto Lane::Position::getPoint() const -> Vector2
{
	return lane_->unmapSigned(positionLon_, positionLat_);
}

inline auto Lane::Position::getPointOnLane() const -> Vector2
{
	return lane_->getPointOnLane(positionLon_);
}

inline auto Lane::Position::getDirection(bool clockwise) const -> Vector2
{
	return clockwise ? -lane_->getDirectionOnLane(positionLon_) : lane_->getDirectionOnLane(positionLon_);
}

inline auto Lane::Position::getDirectionAngle(bool clockwise) const -> double
{
	auto direction = getDirection(clockwise);
	return std::atan2(direction[1], direction[0]);
}

inline void Lane::Position::normalize()
{
	double laneLength = lane_->getTotalLength();

	double laps = std::floor(positionLon_ / laneLength);
	lap_ += static_cast<int>(laps);

	positionLon_ -= laps * laneLength;

	// Numerically the position can still be larger than the length of the lane.
	if (positionLon_ >= laneLength)
	{
		++lap_;
		positionLon_ -= laneLength;
	}
	else if (positionLon_ < 0.0)
	{
		--lap_;
		positionLon_ += laneLength;
	}

	throwing_assert(positionLon_ >= 0.0 && positionLon_ < laneLength);
}

inline auto Lane::Position::normalized() const -> Position
{
	Lane::Position position(*this);
	position.normalize();
	return position;
}

inline Lane::const_iterator Lane::Position::findTile() const
{
	return std::get<0>(lane_->findEx(positionLon_));
}

inline auto Lane::Position::getTileIdentifier() const -> LaneTileBase::Identifier
{
	return findTile() - lane_->begin();
}

inline void Lane::Position::forward(bool clockwise, double distance)
{
	if (clockwise)
		positionLon_ -= distance;
	else
		positionLon_ += distance;

	normalize();
}

inline auto Lane::Position::forwarded(bool clockwise, double distance) const -> Position
{
	Lane::Position position(*this);
	position.forward(clockwise, distance);
	return position;
}

inline void Lane::Position::shiftLaterallyBy(double offset)
{
	positionLat_ += offset;
}

inline auto Lane::Position::shiftedLaterallyBy(double offset) const -> Position
{
	Lane::Position position(*this);
	position.shiftLaterallyBy(offset);
	return position;
}

inline void Lane::Position::shiftLaterallyTo(double position)
{
	positionLat_ = position - lane_->getLateralPositionAbsolute();
}

inline auto Lane::Position::shiftedLaterallyTo(double lateralPosition) const -> Position
{
	Lane::Position position(*this);
	position.shiftLaterallyTo(lateralPosition);
	return position;
}

inline auto Lane::Position::remapped(const std::shared_ptr<const Lane> &lane) const -> Position
{
	Lane::Position position(*this);
	position.remap(lane);
	return position;
}

inline void Lane::Position::remap(const std::shared_ptr<const Lane> &lane)
{
	if (lane_->getLaneNumber() == lane->getLaneNumber())
		return;

	throwing_assert(lane_->size() == lane->size());

	const_iterator tileIt;
	double tilePosition;
	std::tie(tileIt, tilePosition) = lane_->findEx(positionLon_);

	LaneTileBase::Identifier tileId = tileIt - lane_->begin();
	std::shared_ptr<LaneTileBase> tileB = (*lane)[tileId];

	positionLon_ = lane->getPositionOfTile(tileId) + std::get<0>(tileB->map(lane_->getPointOnLane(positionLon_)));
	positionLat_ += (static_cast<double>(lane_->getLaneNumber()) - lane->getLaneNumber()) * distBase;
	lane_ = lane;
}

inline std::tuple<std::shared_ptr<LaneTileBase>, std::shared_ptr<LaneTileBase>> Lane::Position::remapEx(const std::shared_ptr<const Lane> &lane)
{
	throwing_assert(lane_->size() == lane->size());

	const_iterator tileIt;
	double tilePosition;
	std::tie(tileIt, tilePosition) = lane_->findEx(positionLon_);

	if (lane_->getLaneNumber() == lane->getLaneNumber())
		return std::make_tuple(*tileIt, *tileIt);

	LaneTileBase::Identifier tileId = tileIt - lane_->begin();
	std::shared_ptr<LaneTileBase> tileB = (*lane)[tileId];

	positionLon_ = lane->getPositionOfTile(tileId) + std::get<0>(tileB->map(lane_->getPointOnLane(positionLon_)));
	positionLat_ += (static_cast<double>(lane_->getLaneNumber()) - lane->getLaneNumber()) * distBase;
	lane_ = lane;

	return std::make_tuple(*tileIt, tileB);
}

inline auto Lane::Position::operator-(Lane::Position position) const -> Distance
{
	position.remap(lane_);
	return Lane::Distance(lane_, lap_ - position.lap_, positionLon_ - position.positionLon_, positionLat_ - position.positionLat_);
}

inline auto Lane::Position::relapped(const Lane::Position &position, bool clockwise, bool behind) const -> Position
{
	Lane::Position copy(*this);
	copy.relap(position, clockwise, behind);
	return copy;
}

inline void Lane::Position::relap(const Lane::Position &position, bool clockwise, bool behind)
{
	lap_ = position.lap_;

	if (behind)
	{
		if (clockwise)
		{
			if (((*this) - position).getDistanceLon() < 0.0)
				++lap_;
		}
		else
		{
			if ((position - (*this)).getDistanceLon() < 0.0)
				--lap_;
		}
	}
	else
	{
		if (clockwise)
		{
			if (((*this) - position).getDistanceLon() > 0.0)
				--lap_;
		}
		else
		{
			if ((position - (*this)).getDistanceLon() > 0.0)
				++lap_;
		}
	}
}

inline Lane::Distance::Distance()
{
}

inline Lane::Distance::Distance(const std::shared_ptr<const Lane> &lane, int laps, double distanceLon, double distanceLat) : lane_(lane), laps_(laps), distanceLon_(distanceLon), distanceLat_(distanceLat)
{
}

inline double Lane::Distance::getDistanceLon() const
{
	return laps_ * lane_->getTotalLength() + distanceLon_;
}

inline double Lane::Distance::getDistanceLat() const
{
	return distanceLat_;
}

/**
 * Assuming the lane distance is the difference between a position B and a
 * position A on a periodic lane, then this function calculates the
 * non-negative distance point A would have to move along the lane in the given
 * direction in order to reach position B.
 *
 * @param clockwise Specifies the forward direction.
 * @return The minimum non-negative distance.
 */
inline double Lane::Distance::getMinimumForwardDistance(bool clockwise) const
{
	double distance = clockwise ? -getDistanceLon() : getDistanceLon();

	double laneLength = lane_->getTotalLength();
	distance = std::fmod(distance, laneLength);
	if (distance < 0.0)
		distance += laneLength;

	return distance;
}

/**
 * Assuming the lane distance is the difference between a position B and a
 * position A on a periodic lane, then this function calculates the
 * signed distance with the smallest magnitude point A has to move along
 * the lane in order to reach position B. The direction in which point A
 * would have to move is given by the sign. A negative sign indicates a
 * clockwise direction.
 *
 * @return The minimum signed distance.
 */
inline double Lane::Distance::getMinimumSignedDistance() const
{
	double laneLength = lane_->getTotalLength();
	double distance = std::fmod(getDistanceLon(), laneLength);
	if (distance < 0.0)
		distance += laneLength;

	if (laneLength - distance < distance)
		return -(laneLength - distance);
	else
		return distance;
}

inline int Lane::Distance::getLaps() const
{
	return laps_;
}

#endif
