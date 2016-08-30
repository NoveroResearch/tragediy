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

#include <tragediy/track/AnkiDriveMap.h>
#include <tragediy/track/LaneArcTile.h>
#include <tragediy/track/LaneLineTile.h>
#include <tragediy/track/Track.h>
#include <tragediy/util/AssertionError.h>

#include <boost/lexical_cast.hpp>

#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

std::istream &operator>>(std::istream &in, AnkiDriveRoadPieceLocation &location)
{
	skipAhead(in) >> location.x_;
	skipAhead(in) >> location.y_;
	skipAhead(in) >> location.z_;
	skipAhead(in) >> location.a_;
	skipAhead(in) >> location.b_;

	return in;
}

std::size_t AnkiDriveRoadPieceDescription::getPairedConnectorId(std::size_t connectorIdentifier) const
{
	// If vehicle enters a road piece on socket identifier currentSocketId it exits the road piece on socket identifier currentSocketId + 1.
	// TODO: This can probably solved for more general cases.
	return connectorIdentifier + 1;
}

std::istream &operator>>(std::istream &in, AnkiDriveRoadPieceDescription &pieceDescription)
{
	skipAhead(in) >> pieceDescription.length_;
	skipAhead(in) >> pieceDescription.radius_;
	skipAhead(in) >> pieceDescription.barcodeLength_;
	skipAhead(in) >> pieceDescription.gapLength_;
	skipAhead(in) >> pieceDescription.transitionLength_;
	skipAhead(in) >> pieceDescription.intersectionCodeLength_;
	skipAhead(in) >> pieceDescription.numLanes_;

	std::size_t numLocations;
	skipAhead(in) >> numLocations;

	pieceDescription.locations_.clear();
	pieceDescription.locations_.resize(numLocations);

	for (std::size_t i = 0; i < numLocations; ++i)
		in >> pieceDescription.locations_[i];

	skipAhead(in) >> pieceDescription.numConnections_;

	// TODO remaining stuff

	return in;
}

AnkiDriveRoadPiece::FullIdentifier AnkiDriveRoadPiece::getFullIdentifier() const
{
	return identifier_;
}

std::istream &operator>>(std::istream &in, AnkiDriveRoadPiece &piece)
{
	std::size_t tmp;
	skipAhead(in) >> piece.identifier_;
	skipAhead(in) >> tmp;
	skipAhead(in) >> piece.speedLimit_;

	if (tmp != piece.identifier_)
		std::cout << "WARNING: Piece has non-matching identifiers." << std::endl;

	return in;
}

bool AnkiDriveMap::isValid() const
{
	bool valid = true;

	for (std::size_t i = 0; i < connections_.size(); ++i)
	{
		if (connections_[i].identifierX_ >= roadPieces_.size() || connections_[i].identifierY_ >= roadPieces_.size())
		{
			std::cout << "ERROR: Invalid piece identifier in connection." << std::endl;
			valid = false;
		}

		if (connections_[i].connectionX_ >= connections_.size() || connections_[i].connectionY_ >= connections_.size())
		{
			std::cout << "ERROR: Invalid connection identifier in connection." << std::endl;
			valid = false;
		}
	}

	return valid;
}

std::istream &operator>>(std::istream &in, AnkiDriveMap &track)
{
	std::size_t numPieces;

	skipAhead(in) >> numPieces;

	track.roadPieces_.clear();
	track.roadPieces_.resize(numPieces);

	for (std::size_t i = 0; i < numPieces; ++i)
		in >> track.roadPieces_[i];

	std::size_t numConnections;
	skipAhead(in) >> numConnections;

	track.connections_.clear();
	track.connections_.resize(numConnections);

	for (std::size_t i = 0; i < numConnections; ++i)
		in >> track.connections_[i];

	in >> track.finishLine_;

	if (!in)
		track.finishLine_ = AnkiFinishLine();

	skipAhead(in) >> track.offsetX_;
	skipAhead(in) >> track.offsetY_;
	skipAhead(in) >> track.theta_;

	if (!in)
	{
		track.offsetX_ = 0.0;
		track.offsetY_ = 0.0;
		track.theta_ = 0.0;
	}

	throwing_assert(track.isValid());

	return in;
}

void AnkiDriveMap::loadRacingMap(boost::filesystem::path &pathToAppData, const char *name)
{
	boost::filesystem::path file = name;

	std::ifstream fin(file.c_str());
	if (!fin)
	{
		file = pathToAppData / "files/expansion/assets/resources/basestation/config/mapFiles/racing" / name;

		fin = std::ifstream(file.c_str());
		if (!fin)
			throw std::runtime_error(std::string("Cannot open Anki map file ") + file.string() + ".");
	}

	fin >> (*this);

	loadRoadPieceDefinitions(pathToAppData);
}

void AnkiDriveMap::convert(Track &track, double rotationAngle)
{
	throwing_assert(roadPieces_.size() > 0);

	std::size_t numLanes;
	{
		auto &p = roadPieceDescriptions_[roadPieces_[0].getFullIdentifier()];
		numLanes = p.numLanes_;
	}

	throwing_assert(numLanes >= 1);

	Track prototype;

	for (std::size_t i = 0; i < numLanes; ++i)
	{
		Lane lane(i);

		std::shared_ptr<LaneTileBase> tile;

		std::size_t pieceId0 = 0;
		std::size_t socketId0 = 1;

		std::size_t currentPieceId = pieceId0;
		std::size_t currentSocketId = socketId0;

		Vector2 offset0(offsetX_, offsetY_);
		Vector2 direction0(std::cos(theta_ + rotationAngle), std::sin(theta_ + rotationAngle));
		offset0 += direction0.getPerpendicularVectorRight() * ((i - 0.5 * (numLanes - 1)) * 9.0);

		do
		{
			throwing_assert(currentPieceId < roadPieces_.size());

			auto &p = roadPieceDescriptions_[roadPieces_[currentPieceId].getFullIdentifier()];

			throwing_assert(p.numLanes_ == numLanes);
			throwing_assert(currentSocketId < p.numConnections_);

			if (p.radius_ == 0.0 || p.numConnections_ == 4)
			{
				// straight or intersection
				if (!tile)
					tile = std::make_shared<LaneLineTile>(offset0, direction0, p.length_ * 1000.0);
				else
					tile = std::make_shared<LaneLineTile>(tile, p.length_ * 1000.0);
			}
			else
			{
				double r_mid = -p.radius_ * 1000.0;
				double l_mid = p.length_ * 1000.0;

				double r_lane = r_mid - (i - 0.5 * (numLanes - 1)) * 9.0;
				double l_lane = r_lane / r_mid * l_mid;

				// arc tile
				if (!tile)
					tile = std::make_shared<LaneArcTile>(offset0, direction0, r_lane, l_lane);
				else
					tile = std::make_shared<LaneArcTile>(tile, r_lane, l_lane);
			}

			lane.addTile(tile);

			// Look for connection end point starting at piece currentPieceId and socket currentSocketId.
			bool invalid = true;
			for (std::size_t j = 0; j < connections_.size(); ++j)
			{
				if (connections_[j].identifierX_ == currentPieceId && connections_[j].connectionX_ == currentSocketId)
				{
					currentPieceId = connections_[j].identifierY_;
					currentSocketId = connections_[j].connectionY_;

					currentSocketId = roadPieceDescriptions_[roadPieces_[currentPieceId].getFullIdentifier()].getPairedConnectorId(currentSocketId);

					invalid = false;
					break;
				}
			}

			throwing_assert(!invalid);
		} while (currentPieceId != pieceId0 || currentSocketId != socketId0);

		prototype.addLane(lane);
	}

	// Construct middle lane and translate locations into longitudinal/lateral coordinate system.
	std::map<Lane::Identifier, std::map<std::size_t, std::map<double, std::pair<std::size_t, std::size_t>>>> locations; // Maps (lane id, tile index, longitudinal position) to the encoded numbers in the barcode (segment/piece identifier, location id)
	{
		Lane lane(0);

		std::shared_ptr<LaneTileBase> tile;

		std::size_t pieceId0 = 0;
		std::size_t socketId0 = 1;

		std::size_t currentPieceId = pieceId0;
		std::size_t currentSocketId = socketId0;

		Vector2 offset0(offsetX_, offsetY_);
		Vector2 direction0(std::cos(theta_ + rotationAngle), std::sin(theta_ + rotationAngle));

		do
		{
			throwing_assert(currentPieceId < roadPieces_.size());

			auto &p = roadPieceDescriptions_[roadPieces_[currentPieceId].getFullIdentifier()];

			throwing_assert(p.numLanes_ == numLanes);
			throwing_assert(currentSocketId < p.numConnections_);

			if (p.radius_ == 0.0 || p.numConnections_ == 4)
			{
				// straight or intersection
				if (!tile)
					tile = std::make_shared<LaneLineTile>(offset0, direction0, p.length_ * 1000.0);
				else
					tile = std::make_shared<LaneLineTile>(tile, p.length_ * 1000.0);
			}
			else
			{
				double r_mid = -p.radius_ * 1000.0;
				double l_mid = p.length_ * 1000.0;

				// arc tile
				if (!tile)
					tile = std::make_shared<LaneArcTile>(offset0, direction0, r_mid, l_mid);
				else
					tile = std::make_shared<LaneArcTile>(tile, r_mid, l_mid);
			}

			// Translate all locations of current piece into longitudinal positions.
			for (std::size_t i = 0; i < p.locations_.size(); ++i)
			{
				Vector2 q = tile->getStartPoint() + tile->getStartDirection() * (p.locations_[i].x_ * 1000.0) + tile->getStartDirection().getPerpendicularVectorLeft() * (p.locations_[i].y_ * 1000.0);

				double distance, error;
				Lane::Identifier laneIdentifier;
				std::tie(laneIdentifier, distance, error) = prototype.map(q);

				if (error >= 0.5 * 9.0)
					std::cout << "WARNING: Could not reliably map cartesian location to longitudinal location." << std::endl;

				locations[laneIdentifier][lane.size()][distance] = std::make_pair(roadPieces_[currentPieceId].identifier_, i);
			}

			lane.addTile(tile);

			// Look for connection end point starting at piece currentPieceId and socket currentSocketId.
			bool invalid = true;
			for (std::size_t j = 0; j < connections_.size(); ++j)
			{
				if (connections_[j].identifierX_ == currentPieceId && connections_[j].connectionX_ == currentSocketId)
				{
					currentPieceId = connections_[j].identifierY_;
					currentSocketId = connections_[j].connectionY_;

					currentSocketId = roadPieceDescriptions_[roadPieces_[currentPieceId].getFullIdentifier()].getPairedConnectorId(currentSocketId);

					invalid = false;
					break;
				}
			}

			throwing_assert(!invalid);
		} while (currentPieceId != pieceId0 || currentSocketId != socketId0);
	}

	// Add marks to track.
	double threshold = 1.0e-10;
	for (std::size_t i = 0; i < numLanes; ++i)
	{
		Lane lane = *prototype.at(i);

		for (std::size_t j = 0; j < lane.size(); ++j)
		{
			auto &locationsInTile = locations[i][j];
			auto tile = lane[j];

			double fillState0 = lane.getPositionOfTile(j);
			double fillState = fillState0;

			for (auto &p : locationsInTile)
			{
				// p.first: longitudinal position
				// p.second: pair of segment and block
				double markerPositionEnd = p.first;
				double markerPositionStart = markerPositionEnd - bits * (lenMark + lenGap);

				double gap = markerPositionStart - fillState + lenGap;
				throwing_assert(gap >= 0.0);

				tile->pushGap(gap);
				tile->pushBlockOfMarks(p.second.second, p.second.first, false);

				fillState = fillState0 + tile->getFillLength();

				if (std::abs(fillState - markerPositionEnd) > threshold)
					std::cout << "WARNING: Fillstate deviates by " << fillState - markerPositionEnd << " mm." << std::endl;

				if (fillState0 + tile->getTotalLength() - fillState >= 2.0 * (lenGap + lenMark))
				{
					tile->pushGap(lenGap);
					tile->pushMark(MT_THICKER, MT_THIN, false);

					fillState = fillState0 + tile->getFillLength();
				}
			}

			throwing_assert(fillState0 + tile->getTotalLength() - fillState >= lenGap + lenMark);

			if (!tile->pushGap(fillState0 + tile->getTotalLength() - fillState - lenMark - threshold))
				std::cout << "WARNING: Cannot push final gap." << std::endl;
			if (!tile->pushMark(MT_SEPARATOR, MT_SEPARATOR, false))
				std::cout << "WARNING: Cannot push separator." << std::endl;
		}

		track.addLane(lane);
	}
}

void AnkiDriveMap::loadRoadPieceDefinitions(boost::filesystem::path &pathToAppData)
{
	roadPieceDescriptions_.clear();

	for (std::size_t i = 0; i < roadPieces_.size(); ++i)
	{
		if (roadPieceDescriptions_.find(roadPieces_[i].getFullIdentifier()) == roadPieceDescriptions_.end())
		{
			auto file = pathToAppData / "files/expansion/assets/resources/basestation/config/roadPieceDefinitionFiles/racing" / (boost::lexical_cast<std::string>(roadPieces_[i].identifier_) + ".txt");

			std::ifstream fin(file.c_str());
			if (!fin)
				throw std::runtime_error(std::string("Cannot open Anki Drive road piece definition file ") + file.string() + ".");

			AnkiDriveRoadPieceDescription pieceDescription;
			fin >> pieceDescription;

			roadPieceDescriptions_[roadPieces_[i].getFullIdentifier()] = pieceDescription;
		}
	}
}
