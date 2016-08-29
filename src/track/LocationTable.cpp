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
#include <set>

LocationTable::LocationTable()
{
}

LocationTable::LocationTable(const Track &track)
{
	std::set<uint8_t> ambiguousRoadPieces;

	for (auto &lane : track)
	{
		double posAlongLane = 0.0;

		std::size_t numBits = 0;
		uint8_t leftByte = 0;
		uint8_t rightByte = 0;

		bool first = true;
		double firstPosition = std::numeric_limits<double>::infinity();

		bool reverse = false;

		for (auto it = lane->begin(); it != lane->end(); ++it)
		{
			auto &tile = *it;

			double posAlongTile = 0.0;

			for (auto &mark : tile->marks_)
			{
				MarkType leftMark = std::get<0>(mark);
				MarkType rightMark = std::get<1>(mark);
				double len = std::get<2>(mark);

				if ((leftMark == MT_THICKER && rightMark == MT_THIN) || (leftMark == MT_THIN && rightMark == MT_THICKER) || (leftMark == MT_SEPARATOR && rightMark == MT_SEPARATOR))
				{
					if (leftMark == MT_THICKER && rightMark == MT_THIN)
						reverse = false;
					else if (leftMark == MT_THIN && rightMark == MT_THICKER)
						reverse = true;

					// counter clockwise: end block or segment boundary
					if (numBits == 0)
					{
						if (first)
							firstPosition = posAlongLane + posAlongTile + len;
					}
					else
					{
						LocationData locationData;
						locationData.position = tile->getPointOnLane(posAlongTile + len);
						locationData.direction = tile->getDirectionOnLane(posAlongTile + len);
						locationData.lane = lane->getLaneNumber();
						locationData.distance = posAlongLane + posAlongTile + len;
						locationData.backward = false;

						if (reverse)
						{
							std::swap(rightByte, leftByte);

							leftByte = reverseBits(leftByte, numBits);
							rightByte = reverseBits(rightByte, numBits);
						}

						auto key = std::make_tuple(reverse, numBits, rightByte, leftByte);

						if (locationTable_.find(key) != locationTable_.end())
							ambiguousRoadPieces.insert(rightByte);

						locationTable_[key] = locationData;
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

		if (firstPosition != std::numeric_limits<double>::infinity() && numBits > 0)
		{
			LocationData locationData;
			locationData.position = lane->getPointOnLane(firstPosition);
			locationData.direction = lane->getDirectionOnLane(firstPosition);
			locationData.lane = lane->getLaneNumber();
			locationData.distance = firstPosition;
			locationData.backward = false;

			if (reverse)
			{
				std::swap(rightByte, leftByte);

				leftByte = reverseBits(leftByte, numBits);
				rightByte = reverseBits(rightByte, numBits);
			}

			auto key = std::make_tuple(reverse, numBits, rightByte, leftByte);

			if (locationTable_.find(key) != locationTable_.end())
				ambiguousRoadPieces.insert(rightByte);

			locationTable_[key] = locationData;
		}

		numBits = 0;
		leftByte = 0;
		rightByte = 0;

		first = true;
		firstPosition = std::numeric_limits<double>::infinity();

		reverse = true;

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

				MarkType leftMark = std::get<1>(mark);
				MarkType rightMark = std::get<0>(mark);
				double len = std::get<2>(mark);

				if ((leftMark == MT_THICKER && rightMark == MT_THIN) || (leftMark == MT_THIN && rightMark == MT_THICKER) || (leftMark == MT_SEPARATOR && rightMark == MT_SEPARATOR))
				{
					if (leftMark == MT_THICKER && rightMark == MT_THIN)
						reverse = false;
					else if (leftMark == MT_THIN && rightMark == MT_THICKER)
						reverse = true;

					// clockwise: end block or segment boundary
					if (numBits == 0)
					{
						if (first)
							firstPosition = posAlongLane + posAlongTile - len;
					}
					else
					{
						LocationData locationData;
						locationData.position = tile->getPointOnLane(posAlongTile - len);
						locationData.direction = -tile->getDirectionOnLane(posAlongTile - len);
						locationData.lane = lane->getLaneNumber();
						locationData.distance = posAlongLane + posAlongTile - len;
						locationData.backward = true;

						if (reverse)
						{
							std::swap(rightByte, leftByte);

							leftByte = reverseBits(leftByte, numBits);
							rightByte = reverseBits(rightByte, numBits);
						}

						auto key = std::make_tuple(reverse, numBits, rightByte, leftByte);

						if (locationTable_.find(key) != locationTable_.end())
							ambiguousRoadPieces.insert(rightByte);

						locationTable_[key] = locationData;
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

				posAlongTile -= len;
			}
		}

		if (firstPosition != std::numeric_limits<double>::infinity() && numBits > 0)
		{
			LocationData locationData;
			locationData.position = lane->getPointOnLane(firstPosition);
			locationData.direction = -lane->getDirectionOnLane(firstPosition);
			locationData.lane = lane->getLaneNumber();
			locationData.distance = firstPosition;
			locationData.backward = true;

			if (reverse)
			{
				std::swap(rightByte, leftByte);

				leftByte = reverseBits(leftByte, numBits);
				rightByte = reverseBits(rightByte, numBits);
			}

			auto key = std::make_tuple(reverse, numBits, rightByte, leftByte);

			if (locationTable_.find(key) != locationTable_.end())
				ambiguousRoadPieces.insert(rightByte);

			locationTable_[key] = locationData;
		}
	}

	if (!ambiguousRoadPieces.empty())
	{
		std::cout << "WARNING: Encountered ambiguous location barcodes with the following road piece identifiers:";
		for (auto &roadPieceIdentifier : ambiguousRoadPieces)
			std::cout << " " << static_cast<std::size_t>(roadPieceIdentifier);
		std::cout << std::endl;
	}
}

auto LocationTable::findNext(bool reverse, uint8_t segment, uint8_t block, const Track &track) const -> const_iterator
{

	auto current = find(reverse, segment, block);
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
		if (std::get<0>(it->first) == reverse && it->second.lane == current->second.lane)
		{
			double position = it->second.distance;

			if (reverse)
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

auto LocationTable::findNext(bool reverse, Lane::Identifier laneId, double minPosition, const Track &track) const -> const_iterator
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
		if (std::get<0>(it->first) == reverse && it->second.lane == laneId)
		{
			double position = it->second.distance;

			if (reverse)
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
		bool reverse = (i.first == "reverse");

		if (i.first != "non-reverse" && i.first != "reverse")
			throw std::runtime_error("Error parsing location table from JSON input: Invalid entry on first level.");

		for (auto &j : i.second)
		{
			std::size_t numbits = boost::lexical_cast<std::size_t>(j.first);

			for (auto &k : j.second)
			{
				uint8_t segment = boost::numeric_cast<uint8_t>(boost::lexical_cast<int>(k.first));

				for (auto &l : k.second)
				{
					uint8_t block = boost::numeric_cast<uint8_t>(boost::lexical_cast<int>(l.first));

					LocationData locationData;
					locationData.position = Vector2(bb.xMin_ + l.second.get<double>("position_x") * (bb.xMax_ - bb.xMin_), bb.yMin_ + l.second.get<double>("position_y") * (bb.yMax_ - bb.yMin_));
					locationData.direction = Vector2(std::cos(l.second.get<double>("phi")), std::sin(l.second.get<double>("phi")));
					locationData.distance = l.second.get<double>("distance");
					locationData.lane = l.second.get<std::size_t>("lane");
					locationData.backward = l.second.get<bool>("backward");

					locationTable_[std::make_tuple(reverse, numbits, segment, block)] = locationData;
				}
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
		// reverse, numbits, segment, block, position, direction, lane, distance, cw/ccw
		out << std::get<0>(entry.first) << " " << std::get<1>(entry.first) << " " << static_cast<std::size_t>(std::get<2>(entry.first)) << " " << static_cast<std::size_t>(std::get<3>(entry.first)) << " " << entry.second.position[0] << " " << entry.second.position[1] << " " << entry.second.lane << " " << entry.second.distance << " " << entry.second.backward << "\n";
}

void LocationTable::writeToStreamAsJson(std::ostream &out, const BoundingBox &bb) const
{
	bool firstBlockInSegment = true;

	std::tuple<bool, bool, bool> sectionOpen = std::make_tuple(false, false, false);
	std::tuple<bool, std::size_t, uint8_t> sectionLastValue = std::make_tuple(false, static_cast<std::size_t>(0), static_cast<uint8_t>(0));

	out << "{\n";
	for (auto &entry : locationTable_)
	{
		// prepare (non-)reverse section
		if (!std::get<0>(sectionOpen))
		{
			std::get<0>(sectionOpen) = true;
			std::get<0>(sectionLastValue) = std::get<0>(entry.first);
			out << "\t\"" << (std::get<0>(sectionLastValue) ? "reverse" : "non-reverse") << "\": {\n";
		}
		else if (std::get<0>(sectionLastValue) != std::get<0>(entry.first))
		{
			if (std::get<2>(sectionOpen))
			{
				std::get<2>(sectionOpen) = false;
				out << "\n\t\t\t}\n";
			}
			if (std::get<1>(sectionOpen))
			{
				std::get<1>(sectionOpen) = false;
				out << "\t\t}\n";
			}
			out << "\t},\n";
			std::get<0>(sectionLastValue) = std::get<0>(entry.first);
			out << "\t\"" << (std::get<0>(sectionLastValue) ? "reverse" : "non-reverse") << "\": {\n";
		}

		// prepare number of bits section
		if (!std::get<1>(sectionOpen))
		{
			std::get<1>(sectionOpen) = true;
			std::get<1>(sectionLastValue) = std::get<1>(entry.first);
			out << "\t\t\"" << std::get<1>(sectionLastValue) << "\": {\n";
		}
		else if (std::get<1>(sectionLastValue) != std::get<1>(entry.first))
		{
			if (std::get<2>(sectionOpen))
			{
				std::get<2>(sectionOpen) = false;
				out << "\n\t\t\t}\n";
			}
			out << "\t\t},\n";
			std::get<1>(sectionLastValue) = std::get<1>(entry.first);
			out << "\t\t\"" << std::get<1>(sectionLastValue) << "\": {\n";
		}

		// prepare segment section
		if (!std::get<2>(sectionOpen))
		{
			std::get<2>(sectionOpen) = true;
			std::get<2>(sectionLastValue) = std::get<2>(entry.first);
			out << "\t\t\t\"" << static_cast<std::size_t>(std::get<2>(sectionLastValue)) << "\": {\n";
			firstBlockInSegment = true;
		}
		else if (std::get<2>(sectionLastValue) != std::get<2>(entry.first))
		{
			out << "\n\t\t\t},\n";
			std::get<2>(sectionLastValue) = std::get<2>(entry.first);
			out << "\t\t\t\"" << static_cast<std::size_t>(std::get<2>(sectionLastValue)) << "\": {\n";
			firstBlockInSegment = true;
		}

		// print block location data
		if (firstBlockInSegment)
			firstBlockInSegment = false;
		else
			out << ",\n";

		out << "\t\t\t\t\"" << static_cast<std::size_t>(std::get<3>(entry.first)) << "\": {\n";
		const Vector2 &x = entry.second.position;
		out << "\t\t\t\t\t\"position_x\": " << (x[0] - bb.xMin_) / (bb.xMax_ - bb.xMin_) << ",\n";
		out << "\t\t\t\t\t\"position_y\": " << (x[1] - bb.yMin_) / (bb.yMax_ - bb.yMin_) << ",\n";
		out << "\t\t\t\t\t\"phi\": " << std::atan2(entry.second.direction[1], entry.second.direction[0]) << ",\n";
		out << "\t\t\t\t\t\"distance\":   " << entry.second.distance << ",\n";
		out << "\t\t\t\t\t\"lane\":       " << entry.second.lane << ",\n";
		out << "\t\t\t\t\t\"backward\":   " << entry.second.backward << "\n";
		out << "\t\t\t\t}";
	}

	if (std::get<2>(sectionOpen))
		out << "\n\t\t\t}\n";
	if (std::get<1>(sectionOpen))
		out << "\t\t}\n";
	if (std::get<0>(sectionOpen))
		out << "\t}\n";
	out << "}\n";
}
