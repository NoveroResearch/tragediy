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

#include <tragediy/track/Lane.h>
#include <tragediy/track/LaneArcTile.h>

auto Lane::getBoundingBox() const -> BoundingBox
{
	BoundingBox bb;

	for (auto &tile : tiles_)
		bb = ::getBoundingBox(bb, tile->getBoundingBox());

	return bb;
}

auto Lane::getTotalLength() const -> double
{
	double length = 0.0;
	for (auto &i : tiles_)
	{
		length += i->getTotalLength();
	}

	return length;
}

auto Lane::findEx(double distance) -> std::tuple<Lane::iterator, double>
{
	double laneTotalLength = getTotalLength();

	distance = std::fmod(distance, laneTotalLength);
	if (distance < 0.0)
		distance += laneTotalLength;

	for (std::size_t tileNumber = 0; tileNumber < size(); ++tileNumber)
	{
		if (distance < tiles_[tileNumber]->getTotalLength())
			return std::make_tuple(tiles_.begin() + tileNumber, distance);

		distance = distance - tiles_[tileNumber]->getTotalLength();
	}

	return std::make_tuple(tiles_.begin() + size() - 1, tiles_[size() - 1]->getTotalLength());
}

auto Lane::findEx(double distance) const -> std::tuple<Lane::const_iterator, double>
{
	double laneTotalLength = getTotalLength();

	distance = std::fmod(distance, laneTotalLength);
	if (distance < 0.0)
		distance += laneTotalLength;

	for (std::size_t tileNumber = 0; tileNumber < size(); ++tileNumber)
	{
		if (distance < tiles_[tileNumber]->getTotalLength())
			return std::make_tuple(tiles_.begin() + tileNumber, distance);

		distance = distance - tiles_[tileNumber]->getTotalLength();
	}

	return std::make_tuple(tiles_.begin() + size() - 1, tiles_[size() - 1]->getTotalLength());
}

auto Lane::getPointOnLane(double distance) const -> Vector2
{
	double laneTotalLength = getTotalLength();

	distance = std::fmod(distance, laneTotalLength);
	if (distance < 0.0)
		distance += laneTotalLength;

	std::size_t tileNumber = 0;
	while (distance >= tiles_[tileNumber]->getTotalLength())
	{
		distance = distance - tiles_[tileNumber]->getTotalLength();
		tileNumber = (tileNumber + 1) % tiles_.size();
	}

	return tiles_[tileNumber]->getPointOnLane(distance);
}

auto Lane::getDirectionOnLane(double distance) const -> Vector2
{
	double laneTotalLength = getTotalLength();

	distance = std::fmod(distance, laneTotalLength);
	if (distance < 0.0)
		distance += laneTotalLength;

	std::size_t tileNumber = 0;
	while (distance >= tiles_[tileNumber]->getTotalLength())
	{
		distance = distance - tiles_[tileNumber]->getTotalLength();
		tileNumber = (tileNumber + 1) % tiles_.size();
	}

	return tiles_[tileNumber]->getDirectionOnLane(distance);
}

auto Lane::map(const Vector2 &coordinate) const -> std::tuple<double, double>
{
	double distance = std::numeric_limits<double>::infinity();
	double error = std::numeric_limits<double>::infinity();
	double total = 0.0;

	for (auto &i : tiles_)
	{
		auto ret = i->map(coordinate);

		if (std::get<1>(ret) < error)
		{
			error = std::get<1>(ret);
			distance = std::get<0>(ret) + total;
		}

		total += i->getTotalLength();
	}

	return std::make_tuple(distance, error);
}

auto Lane::mapConstrained(const Vector2 &coordinate, const Vector2 &direction) const -> std::tuple<double, double>
{
	double distance = std::numeric_limits<double>::infinity();
	double error = std::numeric_limits<double>::infinity();
	double total = 0.0;

	for (auto &i : tiles_)
	{
		auto ret = i->map(coordinate);

		if (std::get<1>(ret) < error && std::abs(i->getDirectionOnLane(std::get<0>(ret)) * direction) >= 1.0 / std::sqrt(2.0))
		{
			error = std::get<1>(ret);
			distance = std::get<0>(ret) + total;
		}

		total += i->getTotalLength();
	}

	return std::make_tuple(distance, error);
}

auto Lane::map(const Vector2 &coordinate, double lbound, double ubound) const -> std::tuple<double, double>
{
	if (lbound > ubound)
		return std::make_tuple(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());

	double distance = std::numeric_limits<double>::infinity();
	double error = std::numeric_limits<double>::infinity();
	double total = 0.0;
	double laneLength = getTotalLength();

	if (ubound - lbound >= laneLength)
	{
		lbound = 0.0;
		ubound = laneLength;
	}
	else
	{
		// Determine lbound and ubound \in [0; laneLength)
		lbound = std::fmod(lbound, laneLength);
		ubound = std::fmod(ubound, laneLength);
		if (lbound < 0.0)
			lbound += laneLength;
		if (ubound < 0.0)
			ubound += laneLength;
	}

	for (auto &i : tiles_)
	{
		std::tuple<double, double> ret;

		if (lbound <= ubound)
		{
			double lboundTile = std::max(total, lbound);
			double uboundTile = std::min(total + i->getTotalLength(), ubound);

			ret = i->map(coordinate, lboundTile - total, uboundTile - total);
		}
		else
		{
			double lboundTile1 = std::max(total, lbound);
			double uboundTile1 = std::min(total + i->getTotalLength(), laneLength);

			double lboundTile2 = std::max(total, 0.0);
			double uboundTile2 = std::min(total + i->getTotalLength(), ubound);

			auto ret1 = i->map(coordinate, lboundTile1 - total, uboundTile1 - total);
			auto ret2 = i->map(coordinate, lboundTile2 - total, uboundTile2 - total);

			if (std::get<1>(ret1) < std::get<1>(ret2))
				ret = ret1;
			else
				ret = ret2;
		}

		if (std::get<1>(ret) < error)
		{
			error = std::get<1>(ret);
			distance = std::get<0>(ret) + total;
		}

		total += i->getTotalLength();
	}

	return std::make_tuple(distance, error);
}

auto Lane::mapSigned(const Vector2 &coordinate) const -> std::tuple<double, double, double>
{
	double distance, offset, error;
	std::tie(distance, error) = map(coordinate);

	if (error == std::numeric_limits<double>::infinity())
		return std::make_tuple(distance, error, error);

	offset = (coordinate - getPointOnLane(distance)) * getDirectionOnLane(distance).getPerpendicularVectorRight();
	error = std::abs((coordinate - getPointOnLane(distance)) * getDirectionOnLane(distance));
	return std::make_tuple(distance, offset, error);
}

auto Lane::mapSigned(const Vector2 &coordinate, double lbound, double ubound) const -> std::tuple<double, double, double>
{
	double distance, offset, error;
	std::tie(distance, error) = map(coordinate, lbound, ubound);

	if (error == std::numeric_limits<double>::infinity())
		return std::make_tuple(distance, error, error);

	offset = (coordinate - getPointOnLane(distance)) * getDirectionOnLane(distance).getPerpendicularVectorRight();
	error = std::abs((coordinate - getPointOnLane(distance)) * getDirectionOnLane(distance));
	return std::make_tuple(distance, offset, error);
}

void Lane::writeToStreamAsSvg(std::ostream &out, const BoundingBox &bb) const
{
	for (auto &tile : tiles_)
		tile->writeToStreamAsSvg(out, bb);
}

void Lane::writeAnnotationToStreamAsSvg(std::ostream &out, const BoundingBox &bb) const
{
	for (auto &tile : tiles_)
		out << "<text x=\"" << tile->getStartPoint()[0] << "\" y=\"" << tile->getStartPoint()[1] << "\" style=\"font-size:" << 5.0 << "; fill:red\">" << laneNumber_ << "</text>\n";
}

std::ostream &operator<<(std::ostream &out, const Lane &lane)
{
	double p = 0.0;
	LaneTileBase::Identifier no = 0;
	out << "Lane has identifier " << lane.getLaneNumber() << " and contains " << lane.size() << " tiles.\n";
	for (auto &tile : lane)
	{
		out << "#" << no++ << " [" << p << " mm] ";

		auto arcTile = std::dynamic_pointer_cast<LaneArcTile, LaneTileBase>(tile);
		if (arcTile)
			out << "arc of length " << tile->getTotalLength() << " mm with radius " << arcTile->getRadius() << " mm";
		else
			out << "line of length " << tile->getTotalLength();
		out << "\n";

		p += tile->getTotalLength();
	}

	out << "#" << 0 << " [" << p << " mm]\n";

	return out;
}
