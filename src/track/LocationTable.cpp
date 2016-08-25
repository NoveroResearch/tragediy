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

#include <tragediy/track/LocationTable.h>
#include <tragediy/track/Track.h>
#include <tragediy/util/Constants.h>

#include <boost/lexical_cast.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <limits>

LocationTable::LocationTable()
{
}

LocationTable::LocationTable(const Track &track)
{
	for (auto &lane : track)
	{
		double posAlongLane = 0.0;

		std::size_t numBits = 0;
		uint8_t leftByte = 0;
		uint8_t rightByte = 0;

		bool first = true;
		double firstPosition = std::numeric_limits<double>::infinity();

		for (auto it = lane->begin(); it != lane->end(); ++it)
		{
			auto &tile = *it;

			double posAlongTile = 0.0;

			for (auto &mark : tile->marks_)
			{
				MarkType leftMark = std::get<0>(mark);
				MarkType rightMark = std::get<1>(mark);
				double len = std::get<2>(mark);

				if ((leftMark == MT_THICKER && rightMark == MT_THIN) || (leftMark == MT_SEPARATOR && rightMark == MT_SEPARATOR))
				{
					// counter clockwise: end block or segment boundary
					if (numBits == bits)
					{
						LocationData locationData;
						locationData.position = tile->getPointOnLane(posAlongTile + len);
						locationData.direction = tile->getDirectionOnLane(posAlongTile + len);
						locationData.lane = lane->getLaneNumber();
						locationData.distance = posAlongLane + posAlongTile + len;

						auto key = std::make_tuple(false, rightByte, leftByte);

						if (locationTable_.find(key) != locationTable_.end())
							std::cout << "Duplicate data block." << std::endl;

						locationTable_[key] = locationData;
					}
					else if (numBits == 0)
					{
						if (first)
							firstPosition = posAlongLane + posAlongTile + len;
					}
					else
					{
						std::cout << "Incomplete data block." << std::endl;
					}

					first = false;
					numBits = 0;
					leftByte = rightByte = 0;
				}
				else if ((leftMark == MT_THIN || leftMark == MT_THICK) && (rightMark == MT_THIN || rightMark == MT_THICK))
				{
					// data bit

					leftByte <<= 1;
					if (leftMark == MT_THICK)
						leftByte |= 1;
					rightByte <<= 1;
					if (rightMark == MT_THICK)
						rightByte |= 1;
					++numBits;
				}
				else if (leftMark == MT_THIN_DOUBLE && rightMark == MT_THIN_DOUBLE)
				{
				}
				else if (rightMark == MT_THICK_DOUBLE && rightMark == MT_THICK_DOUBLE)
				{
				}
				else if (leftMark == MT_NONE && rightMark == MT_NONE)
				{
				}
				else
				{
					std::cout << "Unknown encoding." << std::endl;
				}

				posAlongTile += len;
			}

			posAlongLane += tile->length_;
		}

		if (firstPosition != std::numeric_limits<double>::infinity() && numBits == bits)
		{
			LocationData locationData;
			locationData.position = lane->getPointOnLane(firstPosition);
			locationData.direction = lane->getDirectionOnLane(firstPosition);
			locationData.lane = lane->getLaneNumber();
			locationData.distance = firstPosition;

			auto key = std::make_tuple(false, rightByte, leftByte);

			if (locationTable_.find(key) != locationTable_.end())
				std::cout << "Duplicate data block." << std::endl;

			locationTable_[key] = locationData;
		}

		numBits = 0;
		leftByte = 0;
		rightByte = 0;

		first = true;
		firstPosition = std::numeric_limits<double>::infinity();

		for (auto it = lane->end(); it != lane->begin();)
		{
			--it;
			auto &tile = *it;

			posAlongLane -= tile->length_;

			double posAlongTile = 0.0;
			for (auto &mark : tile->marks_)
			{
				posAlongTile += std::get<2>(mark);
			}

			for (auto it2 = tile->marks_.end(); it2 != tile->marks_.begin();)
			{
				--it2;
				auto &mark = *it2;

				MarkType leftMark = std::get<0>(mark);
				MarkType rightMark = std::get<1>(mark);
				double len = std::get<2>(mark);

				if ((leftMark == MT_THICKER && rightMark == MT_THIN) || (leftMark == MT_SEPARATOR && rightMark == MT_SEPARATOR))
				{
					// clockwise: end block or segment boundary
					if (numBits == 8)
					{
						LocationData locationData;
						locationData.position = tile->getPointOnLane(posAlongTile - len);
						locationData.direction = -tile->getDirectionOnLane(posAlongTile - len);
						locationData.lane = lane->getLaneNumber();
						locationData.distance = posAlongLane + posAlongTile - len;

						auto key = std::make_tuple(true, rightByte, leftByte);

						if (locationTable_.find(key) != locationTable_.end())
							std::cout << "Duplicate data block." << std::endl;

						locationTable_[key] = locationData;
					}
					else if (numBits == 0)
					{
						if (first)
							firstPosition = posAlongLane + posAlongTile - len;
					}
					else
					{
						std::cout << "Incomplete data block." << std::endl;
					}

					first = false;
					numBits = 0;
					leftByte = rightByte = 0;
				}
				else if ((leftMark == MT_THIN || leftMark == MT_THICK) && (rightMark == MT_THIN || rightMark == MT_THICK))
				{
					// data bit

					leftByte >>= 1;
					if (leftMark == MT_THICK)
						leftByte |= 128;
					rightByte >>= 1;
					if (rightMark == MT_THICK)
						rightByte |= 128;
					++numBits;
				}
				else if (leftMark == MT_THIN_DOUBLE && rightMark == MT_THIN_DOUBLE)
				{
				}
				else if (rightMark == MT_THICK_DOUBLE && rightMark == MT_THICK_DOUBLE)
				{
				}
				else if (leftMark == MT_NONE && rightMark == MT_NONE)
				{
				}
				else
				{
					std::cout << "Unknown encoding." << std::endl;
				}

				posAlongTile -= len;
			}
		}

		if (firstPosition != std::numeric_limits<double>::infinity() && numBits == bits)
		{
			LocationData locationData;
			locationData.position = lane->getPointOnLane(firstPosition);
			locationData.direction = -lane->getDirectionOnLane(firstPosition);
			locationData.lane = lane->getLaneNumber();
			locationData.distance = firstPosition;

			auto key = std::make_tuple(true, rightByte, leftByte);

			if (locationTable_.find(key) != locationTable_.end())
				std::cout << "Duplicate data block." << std::endl;

			locationTable_[key] = locationData;
		}
	}
}

auto LocationTable::findNext(bool clockwise, uint8_t segment, uint8_t block, const Track &track) const -> const_iterator
{

	auto current = find(clockwise, segment, block);
	if (current == end())
	{
		std::cout << "Could not find current segment/block combination." << std::endl;
		return end();
	}

	auto lane = track.find(current->second.lane);
	if (lane == track.end())
	{
		std::cout << "Could not find lane of current segment/block combination." << std::endl;
		return end();
	}

	double laneLength = (*lane)->getTotalLength();
	double minDistance = laneLength;
	auto next = current;

	for (auto it = locationTable_.begin(); it != locationTable_.end(); ++it)
	{
		if (std::get<0>(it->first) == clockwise && it->second.lane == current->second.lane)
		{
			double position = it->second.distance;

			if (clockwise)
			{
				// position decreases in driving direction
				if (position >= current->second.distance)
					position -= laneLength;

				if (current->second.distance - position <= minDistance)
				{
					minDistance = current->second.distance - position;
					next = it;
				}
			}
			else
			{
				// position increases in driving direction
				if (position <= current->second.distance)
					position += laneLength;

				if (position - current->second.distance <= minDistance)
				{
					minDistance = position - current->second.distance;
					next = it;
				}
			}
		}
	}

	assert(next != end());
	return next;
}

auto LocationTable::findNext(bool clockwise, Lane::Identifier laneId, double minPosition, const Track &track) const -> const_iterator
{
	auto lane = track.find(laneId);
	if (lane == track.end())
	{
		std::cout << "Could not find lane in track data structure." << std::endl;
		return end();
	}

	double laneLength = (*lane)->getTotalLength();

	// Ensure that the given position is in the interval [0; laneLength).
	minPosition = std::fmod(minPosition, laneLength);
	if (minPosition < 0.0)
		minPosition += laneLength;

	double minDistance = std::numeric_limits<double>::infinity();
	auto next = end();

	for (auto it = locationTable_.begin(); it != locationTable_.end(); ++it)
	{
		if (std::get<0>(it->first) == clockwise && it->second.lane == laneId)
		{
			double position = it->second.distance;

			if (clockwise)
			{
				// position decreases in driving direction
				if (position >= minPosition)
					position -= laneLength;

				if (minPosition - position <= minDistance)
				{
					minDistance = minPosition - position;
					next = it;
				}
			}
			else
			{
				// position increases in driving direction
				if (position <= minPosition)
					position += laneLength;

				if (position - minPosition <= minDistance)
				{
					minDistance = position - minPosition;
					next = it;
				}
			}
		}
	}

	throwing_assert(next != end());
	return next;
}

void LocationTable::readJsonFromStream(std::istream &in, const BoundingBox &bb)
{
	boost::property_tree::ptree pt;

	try
	{
		boost::property_tree::json_parser::read_json(in, pt);
	}
	catch (boost::property_tree::ptree_error &err)
	{
		throw std::runtime_error(std::string("Error parsing location table from JSON input.") + err.what());
	}

	locationTable_.clear();

	for (auto &i : pt)
	{
		bool clockwise = (i.first == "cw");

		if (i.first != "ccw" && i.first != "cw")
			throw std::runtime_error("Error parsing location table from JSON input: Invalid entry on first level.");

		for (auto &j : i.second)
		{
			uint8_t segment = boost::numeric_cast<uint8_t>(boost::lexical_cast<int>(j.first));

			for (auto &k : j.second)
			{
				uint8_t block = boost::numeric_cast<uint8_t>(boost::lexical_cast<int>(k.first));

				LocationData locationData;
				locationData.position = Vector2(bb.xMin_ + k.second.get<double>("position_x") * (bb.xMax_ - bb.xMin_), bb.yMin_ + k.second.get<double>("position_y") * (bb.yMax_ - bb.yMin_));
				locationData.direction = Vector2(std::cos(k.second.get<double>("phi")), std::sin(k.second.get<double>("phi")));
				locationData.distance = k.second.get<double>("distance");
				locationData.lane = k.second.get<std::size_t>("lane");

				locationTable_[std::make_tuple(clockwise, segment, block)] = locationData;
			}
		}
	}
}

void LocationTable::writeToStreamAsSvg(std::ostream &out) const
{
	for (auto &entry : locationTable_)
	{
		std::string color;
		if (std::get<0>(entry.first))
			color = "green";
		else
			color = "red";

		out << "<circle cx=\"" << entry.second.position[0] << "\" cy=\"" << entry.second.position[1] << "\" r=\"2.0\" stroke=\"black\" stroke-width=\"0.1\" fill=\"" << color << "\" />\n";
	}
}

void LocationTable::writeToStreamAsCsv(std::ostream &out) const
{
	for (auto &entry : locationTable_)
		// cw/ccw, segment, block, position, direction, lane, distance
		out << std::get<0>(entry.first) << " " << static_cast<std::size_t>(std::get<1>(entry.first)) << " " << static_cast<std::size_t>(std::get<2>(entry.first)) << " " << entry.second.position[0] << " " << entry.second.position[1] << " " << entry.second.lane << " " << entry.second.distance << "\n";
}

void LocationTable::writeToStreamAsJson(std::ostream &out, const BoundingBox &bb) const
{
	bool firstBlockInSegment = true;

	std::pair<bool, bool> sectionOpen = std::make_pair(false, false);
	std::pair<bool, uint8_t> sectionLastValue = std::make_pair(false, static_cast<uint8_t>(0));

	out << "{\n";
	for (auto &entry : locationTable_)
	{
		// prepare ccw/cw section
		if (!sectionOpen.first)
		{
			sectionOpen.first = true;
			sectionLastValue.first = std::get<0>(entry.first);
			out << "\t\"" << (sectionLastValue.first ? "cw" : "ccw") << "\": {\n";
		}
		else if (sectionLastValue.first != std::get<0>(entry.first))
		{
			if (sectionOpen.second)
			{
				sectionOpen.second = false;
				out << "\n\t\t}\n";
			}
			out << "\t},\n";
			sectionLastValue.first = std::get<0>(entry.first);
			out << "\t\"" << (sectionLastValue.first ? "cw" : "ccw") << "\": {\n";
		}

		// prepare segment section
		if (!sectionOpen.second)
		{
			sectionOpen.second = true;
			sectionLastValue.second = std::get<1>(entry.first);
			out << "\t\t\"" << static_cast<std::size_t>(sectionLastValue.second) << "\": {\n";
			firstBlockInSegment = true;
		}
		else if (sectionLastValue.second != std::get<1>(entry.first))
		{
			out << "\n\t\t},\n";
			sectionLastValue.second = std::get<1>(entry.first);
			out << "\t\t\"" << static_cast<std::size_t>(sectionLastValue.second) << "\": {\n";
			firstBlockInSegment = true;
		}

		// print block location data
		if (firstBlockInSegment)
			firstBlockInSegment = false;
		else
			out << ",\n";

		out << "\t\t\t\"" << static_cast<std::size_t>(std::get<2>(entry.first)) << "\": {\n";
		const Vector2 &x = entry.second.position;
		out << "\t\t\t\t\"position_x\": " << (x[0] - bb.xMin_) / (bb.xMax_ - bb.xMin_) << ",\n";
		out << "\t\t\t\t\"position_y\": " << (x[1] - bb.yMin_) / (bb.yMax_ - bb.yMin_) << ",\n";
		out << "\t\t\t\t\"phi\": " << std::atan2(entry.second.direction[1], entry.second.direction[0]) << ",\n";
		out << "\t\t\t\t\"distance\":   " << entry.second.distance << ",\n";
		out << "\t\t\t\t\"lane\":       " << entry.second.lane << "\n";
		out << "\t\t\t}";
	}

	if (sectionOpen.second)
		out << "\n\t\t}\n";
	if (sectionOpen.first)
		out << "\t}\n";
	out << "}\n";
}
