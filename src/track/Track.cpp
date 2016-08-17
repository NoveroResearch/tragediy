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

#include <tragediy/track/LaneArcTile.h>
#include <tragediy/track/LaneLineTile.h>
#include <tragediy/track/Track.h>
#include <tragediy/util/Constants.h>
#include <tragediy/util/Math.h>

#include <boost/lexical_cast.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <limits>
#include <map>

void constructRingTrack(Track &track, Track &borders, double radius, double len, int n, bool withBorders)
{
	std::map<uint8_t, std::size_t> numberOfBlocksInSegment;
	std::shared_ptr<LaneTileBase> tile;
	uint8_t segment;
	double r = 0;
	double lineDistance = distBase;
	double lineWidth = widthBaseThick;
	int borderRing;

	// track
	for (int i = 0; i < n; ++i)
	{
		Lane lane(i);

		double r = radius + distBase * i;
		segment = 0x00;
		tile = std::make_shared<LaneLineTile>(Vector2(0, -distBase * i), Vector2(-1, 0), len);
		tile->pushMark(MT_SEPARATOR, MT_SEPARATOR, true); // Transition
		numberOfBlocksInSegment[segment] = tile->fillWithLocationMarkers(segment, numberOfBlocksInSegment[segment], 256, lenMark / 2, lenGap / 2);
		tile->pushMark(MT_SEPARATOR, MT_SEPARATOR, true);
		lane.addTile(tile);

		segment = 0x01;
		tile = std::make_shared<LaneArcTile>(tile, -r, r * pi<double>);
		numberOfBlocksInSegment[segment] = tile->fillWithLocationMarkers(segment, numberOfBlocksInSegment[segment]);
		tile->fillWithEndOfBlockMarks();
		lane.addTile(tile);

		segment = 0x02;
		tile = std::make_shared<LaneLineTile>(tile, len);
		tile->pushMark(MT_SEPARATOR, MT_SEPARATOR, true);
		numberOfBlocksInSegment[segment] = tile->fillWithLocationMarkers(segment, numberOfBlocksInSegment[segment], 256, lenMark / 2, lenGap / 2);
		tile->pushMark(MT_SEPARATOR, MT_SEPARATOR, true);
		lane.addTile(tile);

		segment = 0x03;
		tile = std::make_shared<LaneArcTile>(tile, -r, r * pi<double>);
		numberOfBlocksInSegment[segment] = tile->fillWithLocationMarkers(segment, numberOfBlocksInSegment[segment]);
		tile->fillWithEndOfBlockMarks();
		lane.addTile(tile);

		track.addLane(lane);
	}

	if (withBorders)
	{
		// inner borders
		borderRing = 2; // start with outer ring
		for (int i = -6; i < 0; ++i)
		{
			Lane lane(i);

			if (i >= -3)
				borderRing = 1; // inner ring

			// fat line closer to thin line
			if (i == -3 || i == -6)
			{
				lineDistance = borderRing * distBase + (2 * widthThin + widthThicker / 2);
				r = radius - lineDistance;
				lineWidth = widthBaseThicker;
			}
			// thin line
			else if (i == -2 || i == -5)
			{
				lineDistance = borderRing * distBase;
				r = radius - lineDistance;
				lineWidth = widthBaseThin;
			}
			// fat line further away from thin line
			else if (i == -1 || i == -4)
			{
				lineDistance = borderRing * distBase - (widthThin + widthThick + widthThicker / 2);
				r = radius - lineDistance;
				lineWidth = widthBaseThicker;
			}

			tile = std::make_shared<LaneLineTile>(Vector2(0, lineDistance), Vector2(-1, 0), len);
			tile->setWidthBase(0, lineWidth);
			lane.addTile(tile);

			tile = std::make_shared<LaneArcTile>(tile, -r, r * pi<double>);
			tile->setWidthBase(0, lineWidth);
			lane.addTile(tile);

			tile = std::make_shared<LaneLineTile>(tile, len);
			tile->setWidthBase(0, lineWidth);
			lane.addTile(tile);

			tile = std::make_shared<LaneArcTile>(tile, -r, r * pi<double>);
			tile->setWidthBase(0, lineWidth);
			lane.addTile(tile);

			borders.addLane(lane);
		}

		// outer borders
		borderRing = 1; // start with inner ring
		for (int i = n; i < n + 6; ++i)
		{
			Lane lane(i);

			if (i >= n + 3)
				borderRing = 2; // outer ring

			// fat line closer to thin line
			if (i == n || i == n + 3)
			{
				lineDistance = -distBase * (n - 1 + borderRing) - (2 * widthThin + widthThicker / 2);
				r = radius - lineDistance;
				lineWidth = widthBaseThicker;
			}
			// thin line
			else if (i == n + 1 || i == n + 4)
			{
				lineDistance = -distBase * (n - 1 + borderRing);
				r = radius - lineDistance;
				lineWidth = widthBaseThin;
			}
			// fat line further away from thin line
			else if (i == n + 2 || i == n + 5)
			{
				lineDistance = -distBase * (n - 1 + borderRing) + (widthThin + widthThick + widthThicker / 2);
				r = radius - lineDistance;
				lineWidth = widthBaseThicker;
			}

			tile = std::make_shared<LaneLineTile>(Vector2(0, lineDistance), Vector2(-1, 0), len);
			tile->setWidthBase(0, lineWidth);
			lane.addTile(tile);

			tile = std::make_shared<LaneArcTile>(tile, -r, r * pi<double>);
			tile->setWidthBase(0, lineWidth);
			lane.addTile(tile);

			tile = std::make_shared<LaneLineTile>(tile, len);
			tile->setWidthBase(0, lineWidth);
			lane.addTile(tile);

			tile = std::make_shared<LaneArcTile>(tile, -r, r * pi<double>);
			tile->setWidthBase(0, lineWidth);
			lane.addTile(tile);

			borders.addLane(lane);
		}
	}
}

void constructStarterTrack(Track &track)
{
	std::map<uint8_t, std::size_t> numberOfBlocksInSegment;
	std::shared_ptr<LaneTileBase> tile;
	uint8_t segment;

	for (int i = 0; i < 32; ++i)
	{
		Lane lane(i);
		double r = 150.0 + i * distBase;

		segment = 0x4c;
		tile = std::make_shared<LaneLineTile>(Vector2(0, -i * distBase), Vector2(-1, 0), 250.0);
		tile->pushMark(MT_SEPARATOR, MT_SEPARATOR);
		numberOfBlocksInSegment[segment] = tile->fillWithLocationMarkers(segment, numberOfBlocksInSegment[segment]);
		tile->fillWithEndOfBlockMarks();
		lane.addTile(tile);

		segment = 0x4d;
		tile = std::make_shared<LaneLineTile>(tile, 980.0);
		tile->pushMark(MT_SEPARATOR, MT_SEPARATOR);
		numberOfBlocksInSegment[segment] = tile->fillWithLocationMarkers(segment, numberOfBlocksInSegment[segment]);
		tile->fillWithEndOfBlockMarks();
		lane.addTile(tile);

		segment = 0x46;
		tile = std::make_shared<LaneLineTile>(tile, 250.0);
		tile->pushMark(MT_SEPARATOR, MT_SEPARATOR);
		numberOfBlocksInSegment[segment] = tile->fillWithLocationMarkers(segment, numberOfBlocksInSegment[segment]);
		tile->fillWithEndOfBlockMarks();
		lane.addTile(tile);

		segment = 0x47;
		auto arctile = std::make_shared<LaneArcTile>(tile, -r, r * 1.103 * pi<double>);
		tile = arctile;
		tile->pushMark(MT_SEPARATOR, MT_SEPARATOR);
		numberOfBlocksInSegment[segment] = tile->fillWithLocationMarkers(segment, numberOfBlocksInSegment[segment]);
		tile->fillWithEndOfBlockMarks();
		lane.addTile(tile);

		double r_large = (-arctile->getMidPoint()[0]) / (2.0 * std::sin((1.103 - 1.0) * pi<double>)) - r;
		double angle_side = 0.04;
		double angle_mid = 2.0 * (1.103 - 1.0) - 2.0 * angle_side;

		segment = 0x48;
		tile = std::make_shared<LaneArcTile>(tile, r_large, r_large * angle_side * pi<double>);
		tile->pushMark(MT_SEPARATOR, MT_SEPARATOR);
		numberOfBlocksInSegment[segment] = tile->fillWithLocationMarkers(segment, numberOfBlocksInSegment[segment]);
		tile->fillWithEndOfBlockMarks();
		lane.addTile(tile);

		segment = 0x49;
		tile = std::make_shared<LaneArcTile>(tile, r_large, r_large * angle_mid * pi<double>);
		tile->pushMark(MT_SEPARATOR, MT_SEPARATOR);
		numberOfBlocksInSegment[segment] = tile->fillWithLocationMarkers(segment, numberOfBlocksInSegment[segment]);
		tile->fillWithEndOfBlockMarks();
		lane.addTile(tile);

		segment = 0x4a;
		tile = std::make_shared<LaneArcTile>(tile, r_large, r_large * angle_side * pi<double>);
		tile->pushMark(MT_SEPARATOR, MT_SEPARATOR);
		numberOfBlocksInSegment[segment] = tile->fillWithLocationMarkers(segment, numberOfBlocksInSegment[segment]);
		tile->fillWithEndOfBlockMarks();
		lane.addTile(tile);

		segment = 0x4b;
		tile = std::make_shared<LaneArcTile>(tile, -r, r * 1.103 * pi<double>);
		tile->pushMark(MT_SEPARATOR, MT_SEPARATOR);
		numberOfBlocksInSegment[segment] = tile->fillWithLocationMarkers(segment, numberOfBlocksInSegment[segment]);
		if (i == 24)
		{
			// Squeeze in additional block of marks.
			tile->pushBlockOfMarks(numberOfBlocksInSegment[segment]++, segment, false);
		}
		tile->fillWithEndOfBlockMarks();
		lane.addTile(tile);

		track.addLane(lane);
	}
}

const BoundingBox &Track::adaptCanvas()
{
	bb_ = BoundingBox();

	for (auto &lane : lanes_)
		bb_ = ::getBoundingBox(bb_, lane->getBoundingBox());

	return bb_;
}

void Track::readJsonFromStream(std::istream &in)
{
	boost::property_tree::ptree pt;
	std::shared_ptr<LaneTileBase> tile;

	try
	{
		boost::property_tree::json_parser::read_json(in, pt);
	}
	catch (boost::property_tree::ptree_error &err)
	{
		throw std::runtime_error(std::string("Error parsing track from JSON input.") + err.what());
	}

	lanes_.clear();

	if (pt.find("left") == pt.not_found() || pt.find("top") == pt.not_found() || pt.find("width") == pt.not_found() || pt.find("height") == pt.not_found())
	{
		throw std::runtime_error("Error parsing track from JSON input: No bounding box description found.");
	}
	else
	{
		bb_.xMin_ = pt.get<double>("left");
		bb_.yMin_ = pt.get<double>("top");
		bb_.xMax_ = bb_.xMin_ + pt.get<double>("width");
		bb_.yMax_ = bb_.yMin_ + pt.get<double>("height");
	}

	for (auto &i : pt)
	{
		if (i.first == "left" || i.first == "top" || i.first == "width" || i.first == "height")
			continue;

		Lane lane(boost::lexical_cast<std::size_t>(i.first));

		for (auto &j : i.second)
		{
			std::string type = j.second.get<std::string>("type");

			double len = j.second.get<double>("length");

			auto &ptStartPoint = j.second.get_child("start_point");
			if (ptStartPoint.size() != 2)
				throw std::runtime_error("Error parsing track from JSON input: Start point is not a vector with two components.");
			Vector2 startPoint(boost::lexical_cast<double>(ptStartPoint.front().second.data()), boost::lexical_cast<double>(ptStartPoint.back().second.data()));

			auto &ptStartDirection = j.second.get_child("start_direction");
			if (ptStartDirection.size() != 2)
				throw std::runtime_error("Error parsing track from JSON input: Start direction is not a vector with two components.");
			Vector2 startDirection(boost::lexical_cast<double>(ptStartDirection.front().second.data()), boost::lexical_cast<double>(ptStartDirection.back().second.data()));

			if (type == "line")
			{
				tile = std::make_shared<LaneLineTile>(startPoint, startDirection, len);
			}
			else if (type == "arc")
			{
				double r = j.second.get<double>("radius");
				tile = std::make_shared<LaneArcTile>(startPoint, startDirection, r, len);
			}
			else
			{
				throw std::runtime_error("Error parsing track from JSON input: Unknown tile type found.");
			}

			lane.addTile(tile);
		}

		addLane(lane);
	}
}

void Track::writeToStreamAsSvg(std::ostream &out, const BoundingBox &bb) const
{
	for (auto &lane : lanes_)
		lane->writeToStreamAsSvg(out, bb);
}

void Track::writeAnnotationToStreamAsSvg(std::ostream &out, const BoundingBox &bb) const
{
	for (auto &lane : lanes_)
		lane->writeAnnotationToStreamAsSvg(out, bb);
}

void Track::writeToStreamAsJson(std::ostream &out, const BoundingBox &bb) const
{
	out << "{\n";
	out << "\t\"left\":   " << bb.xMin_ << ",\n";
	out << "\t\"top\":    " << bb.yMin_ << ",\n";
	out << "\t\"width\":  " << bb.xMax_ - bb.xMin_ << ",\n";
	out << "\t\"height\": " << bb.yMax_ - bb.yMin_ << ",\n";
	for (std::size_t i = 0; i < lanes_.size(); ++i)
	{
		auto &lane = lanes_[i];

		out << "\t\"" << lane->getLaneNumber() << "\": [\n";

		for (std::size_t j = 0; j < lane->size(); ++j)
		{
			auto &tile = *(lane->begin() + j);
			auto arctile = std::dynamic_pointer_cast<LaneArcTile, LaneTileBase>(tile);

			out << "\t\t{\n";
			out << "\t\t\t\"type\":            \"" << (arctile ? "arc" : "line") << "\",\n";
			out << "\t\t\t\"length\":          " << tile->getTotalLength() << ",\n";
			out << "\t\t\t\"start_point\":     [" << tile->getStartPoint()[0] << ", " << tile->getStartPoint()[1] << "],\n";
			out << "\t\t\t\"start_direction\": [" << tile->getStartDirection()[0] << ", " << tile->getStartDirection()[1] << "]";
			if (arctile)
				out << ",\n\t\t\t\"radius\":          " << arctile->getRadius();
			out << "\n";
			out << "\t\t}";
			if (j != lane->size() - 1)
				out << ",";
			out << "\n";
		}
		out << "\t]";
		if (i != lanes_.size() - 1)
			out << ",";
		out << "\n";
	}
	out << "}\n";
}

auto Track::map(const Vector2 &coordinate) const -> std::tuple<std::size_t, double, double>
{
	std::tuple<std::size_t, double, double> best = std::make_tuple(std::numeric_limits<std::size_t>::max(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());

	for (auto &lane : lanes_)
	{
		double distance, error;
		std::tie(distance, error) = lane->map(coordinate);

		if (error < std::get<2>(best))
			best = std::make_tuple(lane->getLaneNumber(), distance, error);
	}

	return best;
}

auto Track::mapSigned(const Vector2 &coordinate) const -> std::tuple<std::size_t, double, double, double>
{
	std::tuple<std::size_t, double, double, double> best = std::make_tuple(std::numeric_limits<std::size_t>::max(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());

	for (auto &lane : lanes_)
	{
		double distance, offset, error;
		std::tie(distance, offset, error) = lane->mapSigned(coordinate);

		if (error < std::get<3>(best))
			best = std::make_tuple(lane->getLaneNumber(), distance, offset, error);
	}

	return best;
}

auto Track::mapConstrained(const Vector2 &coordinate, const Vector2 &direction) const -> std::tuple<std::size_t, double, double>
{
	std::tuple<std::size_t, double, double> best = std::make_tuple(std::numeric_limits<std::size_t>::max(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());

	for (auto &lane : lanes_)
	{
		double distance, error;
		std::tie(distance, error) = lane->mapConstrained(coordinate, direction);

		if (error < std::get<2>(best))
			best = std::make_tuple(lane->getLaneNumber(), distance, error);
	}

	return best;
}
