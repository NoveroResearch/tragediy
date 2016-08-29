#include <tragediy/track/AnkiOverdriveMap.h>
#include <tragediy/track/LaneArcTile.h>
#include <tragediy/track/LaneLineTile.h>
#include <tragediy/util/Math.h>

#include <cmath>

#include <boost/lexical_cast.hpp>

std::istream &operator>>(std::istream &in, AnkiOverdriveLocalizableSection &section)
{
	skipAhead(in) >> section.poseStart_;
	skipAhead(in) >> section.poseEnd_;
	for (std::size_t i = 0; i < 6; ++i)
		skipAhead(in) >> section.laneInfo_[i];
	skipAhead(in) >> section.specialCodeInfo1_;
	skipAhead(in) >> section.specialCodeInfo2_;

	return in;
}

std::istream &operator>>(std::istream &in, AnkiOverdriveRoadPieceConnection &connection)
{
	for (std::size_t i = 0; i < 4; ++i)
		skipAhead(in) >> connection.unknown_[i];

	return in;
}

std::istream &operator>>(std::istream &in, AnkiOverdriveRoadPieceLaneEntry &laneEntry)
{
	skipAhead(in) >> laneEntry.poseStart_;
	skipAhead(in) >> laneEntry.poseStartValid_;
	skipAhead(in) >> laneEntry.poseEnd_;
	skipAhead(in) >> laneEntry.poseEndValid_;
	skipAhead(in) >> laneEntry.laneNumber_;
	skipAhead(in) >> laneEntry.sectionIdentifier_;

	return in;
}

std::istream &operator>>(std::istream &in, AnkiOverdriveRoadPieceBarcodeEntry &location)
{
	skipAhead(in) >> location.pose_;
	skipAhead(in) >> location.locationIndex_;
	skipAhead(in) >> location.bit_;
	skipAhead(in) >> location.lane_;
	skipAhead(in) >> location.section_;

	return in;
}

std::size_t AnkiOverdriveRoadPieceDescription::getPairedConnectorId(std::size_t connectorIdentifier) const
{
	// TODO: There is probably no necessity to hardcode the following.
	std::size_t ret;
	if (connectorIdentifier == 0)
		ret = 1;
	else if (connectorIdentifier == 1)
		ret = 0;
	else if (connectorIdentifier == 2)
		ret = 3;
	else if (connectorIdentifier == 3)
		ret = 2;
	else
		throwing_assert(false);

	return ret;
}

std::istream &operator>>(std::istream &in, AnkiOverdriveRoadPieceDescription &pieceDescription)
{
	skipAhead(in) >> pieceDescription.fileVersion_;
	skipAhead(in) >> pieceDescription.type_;
	skipAhead(in) >> pieceDescription.invertedColor_;
	skipAhead(in) >> pieceDescription.numBits_;
	skipAhead(in) >> pieceDescription.identifier_;
	skipAhead(in) >> pieceDescription.numLanes_;
	skipAhead(in) >> pieceDescription.numBorderLanes_;
	skipAhead(in) >> pieceDescription.transitionLengthStart_;
	skipAhead(in) >> pieceDescription.transitionLengthEnd_;
	skipAhead(in) >> pieceDescription.gapLength_;
	skipAhead(in) >> pieceDescription.barcodeLength_;
	skipAhead(in) >> pieceDescription.intersectionCodeLength_;
	skipAhead(in) >> pieceDescription.distanceLanes_;
	skipAhead(in) >> pieceDescription.numSections_;

	// Read localizable sections.
	{
		std::size_t numLocalizableSections;
		skipAhead(in) >> numLocalizableSections;

		pieceDescription.localizableSections_.clear();
		pieceDescription.localizableSections_.resize(numLocalizableSections);

		for (std::size_t i = 0; i < numLocalizableSections; ++i)
			in >> pieceDescription.localizableSections_[i];
	}

	// Read connectors.
	{
		std::size_t numConnectors;
		skipAhead(in) >> numConnectors;

		pieceDescription.connectors_.clear();
		pieceDescription.connectors_.resize(numConnectors);

		for (std::size_t i = 0; i < numConnectors; ++i)
			in >> pieceDescription.connectors_[i];
	}

	// Read connections.
	{
		std::size_t numConnections;
		skipAhead(in) >> numConnections;

		pieceDescription.connections_.clear();
		pieceDescription.connections_.resize(numConnections);

		for (std::size_t i = 0; i < numConnections; ++i)
			in >> pieceDescription.connections_[i];
	}

	// Read lane entries.
	{
		std::size_t numLaneEntries;
		skipAhead(in) >> numLaneEntries;

		pieceDescription.laneEntries_.clear();
		pieceDescription.laneEntries_.resize(numLaneEntries);

		for (std::size_t i = 0; i < numLaneEntries; ++i)
			in >> pieceDescription.laneEntries_[i];
	}

	// Read locations.
	{
		std::size_t numLocations;
		skipAhead(in) >> numLocations;

		pieceDescription.barcodeEntries_.clear();
		pieceDescription.barcodeEntries_.resize(numLocations);

		for (std::size_t i = 0; i < numLocations; ++i)
			in >> pieceDescription.barcodeEntries_[i];
	}

	return in;
}

std::istream &operator>>(std::istream &in, AnkiOverdriveRoadPiece &piece)
{
	skipAhead(in) >> piece.inverted_;
	skipAhead(in) >> piece.numBits_;
	skipAhead(in) >> piece.identifier_;
	skipAhead(in) >> piece.speedLimit_;
	skipAhead(in) >> piece.reverse_;

	return in;
}

AnkiOverdriveRoadPiece::FullIdentifier AnkiOverdriveRoadPiece::getFullIdentifier() const
{
	return std::make_tuple(inverted_, numBits_, identifier_);
}

bool AnkiOverdriveMap::isValid() const
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

std::istream &operator>>(std::istream &in, AnkiOverdriveMap &track)
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

void AnkiOverdriveMap::loadRacingMap(boost::filesystem::path &pathToAppData, const char *name)
{
	boost::filesystem::path file = name;

	std::ifstream fin(file.c_str());
	if (!fin)
	{
		file = pathToAppData / "files/expansion/assets/resources/basestation/config/mapFiles/racing" / name;

		fin = std::ifstream(file.c_str());
		if (!fin)
			throw std::runtime_error(std::string("Cannot open Anki Overdrive map file ") + file.string() + ".");
	}

	fin >> (*this);

	loadRoadPieceDefinitions(pathToAppData);
}

void AnkiOverdriveMap::convert(Track &track, double rotationAngle)
{
	throwing_assert(roadPieces_.size() > 0);

	std::size_t numLanes;
	double distanceLanes;
	{
		auto &p = roadPieceDescriptions_[roadPieces_[0].getFullIdentifier()];
		numLanes = p.numLanes_;
		distanceLanes = p.distanceLanes_ * 1000.0;
	}

	throwing_assert(numLanes >= 1);

	Track prototype;

	for (std::size_t i = 0; i < numLanes; ++i)
	{
		Lane lane(i);

		std::shared_ptr<LaneTileBase> tile;

		std::size_t pieceId0 = 0;
		std::size_t socketId0 = roadPieces_[pieceId0].reverse_ ? 0 : 1;

		std::size_t currentPieceId = pieceId0;
		std::size_t currentSocketId = socketId0;

		Vector2 offset0(offsetX_, offsetY_);
		Vector2 direction0(std::cos(theta_ + rotationAngle), std::sin(theta_ + rotationAngle));
		offset0 += direction0.getPerpendicularVectorRight() * ((i - 0.5 * (numLanes - 1)) * distanceLanes);

		do
		{
			throwing_assert(currentPieceId < roadPieces_.size());

			auto &p = roadPieceDescriptions_[roadPieces_[currentPieceId].getFullIdentifier()];

			throwing_assert(p.numLanes_ == numLanes);
			throwing_assert(currentSocketId < p.connectors_.size());

			std::size_t exitSocketId = p.getPairedConnectorId(currentSocketId);

			if (p.type_ == 0 || p.type_ == 4)
			{
				// straight or intersection
				Vector2 pointStart(p.connectors_[currentSocketId].dx_, p.connectors_[currentSocketId].dy_);
				Vector2 pointEnd(p.connectors_[exitSocketId].dx_, p.connectors_[exitSocketId].dy_);

				double length = (pointEnd - pointStart).getLength();

				if (!tile)
					tile = std::make_shared<LaneLineTile>(offset0, direction0, length * 1000.0);
				else
					tile = std::make_shared<LaneLineTile>(tile, length * 1000.0);
			}
			else if (p.type_ == 1)
			{
				// curve
				throwing_assert(p.localizableSections_.size() == 1);

				Vector2 pointStart(p.localizableSections_[0].poseStart_.dx_, p.localizableSections_[0].poseStart_.dy_);
				Vector2 directionStart(std::cos(p.localizableSections_[0].poseStart_.dphi_), std::sin(p.localizableSections_[0].poseStart_.dphi_));

				Vector2 pointEnd(p.localizableSections_[0].poseEnd_.dx_, p.localizableSections_[0].poseEnd_.dy_);
				Vector2 directionEnd(std::cos(p.localizableSections_[0].poseEnd_.dphi_), std::sin(p.localizableSections_[0].poseEnd_.dphi_));

				Vector2 A(directionStart.getPerpendicularVectorLeft() - directionEnd.getPerpendicularVectorLeft());
				Vector2 b(pointEnd - pointStart);

				throwing_assert(A * A >= 1.0e-6);

				// Solve A^T*A*x = A^T*b
				double r_mid = (A * b) / (A * A);

				if (roadPieces_[currentPieceId].reverse_)
					r_mid = -r_mid;

				r_mid = r_mid * 1000.0;

				double phiStart = std::fmod(p.localizableSections_[0].poseStart_.dphi_, 2.0 * pi<double>);
				if (phiStart < 0.0)
					phiStart += 2.0 * pi<double>;

				double phiEnd = std::fmod(p.localizableSections_[0].poseEnd_.dphi_, 2.0 * pi<double>);
				if (phiEnd < 0.0)
					phiEnd += 2.0 * pi<double>;

				if (phiEnd < phiStart)
					phiEnd += 2.0 * pi<double>;

				double dphi = phiEnd - phiStart;

				throwing_assert(dphi >= 0.0);

				double l_mid = std::abs(r_mid) * dphi;

				double r_lane = r_mid - (i - 0.5 * (numLanes - 1)) * distanceLanes;
				double l_lane = r_lane / r_mid * l_mid;

				// arc tile
				if (!tile)
					tile = std::make_shared<LaneArcTile>(offset0, direction0, r_lane, l_lane);
				else
					tile = std::make_shared<LaneArcTile>(tile, r_lane, l_lane);
			}
			else
			{
				throwing_assert(false);
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
	std::map<Lane::Identifier, std::map<std::size_t, std::map<double, std::pair<std::size_t, std::size_t>>>> barcodeEntries; // Maps (lane id, tile index, longitudinal position) to (piece index, barcode entry index).
	std::vector<std::size_t> tileToPieceIndex; // Maps (tile index) to (piece index).
	{
		Lane lane(0);

		std::shared_ptr<LaneTileBase> tile;

		std::size_t pieceId0 = 0;
		std::size_t socketId0 = roadPieces_[pieceId0].reverse_ ? 0 : 1;

		std::size_t currentPieceId = pieceId0;
		std::size_t currentSocketId = socketId0;

		Vector2 offset0(offsetX_, offsetY_);
		Vector2 direction0(std::cos(theta_ + rotationAngle), std::sin(theta_ + rotationAngle));

		do
		{
			throwing_assert(currentPieceId < roadPieces_.size());

			tileToPieceIndex.push_back(currentPieceId);

			auto &p = roadPieceDescriptions_[roadPieces_[currentPieceId].getFullIdentifier()];

			throwing_assert(p.numLanes_ == numLanes);
			throwing_assert(currentSocketId < p.connectors_.size());

			std::size_t exitSocketId = p.getPairedConnectorId(currentSocketId);

			if (p.type_ == 0 || p.type_ == 4)
			{
				// straight or intersection
				Vector2 pointStart(p.connectors_[currentSocketId].dx_, p.connectors_[currentSocketId].dy_);
				Vector2 pointEnd(p.connectors_[exitSocketId].dx_, p.connectors_[exitSocketId].dy_);

				double length = (pointEnd - pointStart).getLength();

				if (!tile)
					tile = std::make_shared<LaneLineTile>(offset0, direction0, length * 1000.0);
				else
					tile = std::make_shared<LaneLineTile>(tile, length * 1000.0);
			}
			else if (p.type_ == 1)
			{
				// curve
				throwing_assert(p.localizableSections_.size() == 1);

				Vector2 pointStart(p.localizableSections_[0].poseStart_.dx_, p.localizableSections_[0].poseStart_.dy_);
				Vector2 directionStart(std::cos(p.localizableSections_[0].poseStart_.dphi_), std::sin(p.localizableSections_[0].poseStart_.dphi_));

				Vector2 pointEnd(p.localizableSections_[0].poseEnd_.dx_, p.localizableSections_[0].poseEnd_.dy_);
				Vector2 directionEnd(std::cos(p.localizableSections_[0].poseEnd_.dphi_), std::sin(p.localizableSections_[0].poseEnd_.dphi_));

				Vector2 A(directionStart.getPerpendicularVectorLeft() - directionEnd.getPerpendicularVectorLeft());
				Vector2 b(pointEnd - pointStart);

				throwing_assert(A * A >= 1.0e-6);

				// Solve A^T*A*x = A^T*b
				double r_mid = (A * b) / (A * A);

				if (roadPieces_[currentPieceId].reverse_)
					r_mid = -r_mid;

				r_mid = r_mid * 1000.0;

				double phiStart = std::fmod(p.localizableSections_[0].poseStart_.dphi_, 2.0 * pi<double>);
				if (phiStart < 0.0)
					phiStart += 2.0 * pi<double>;

				double phiEnd = std::fmod(p.localizableSections_[0].poseEnd_.dphi_, 2.0 * pi<double>);
				if (phiEnd < 0.0)
					phiEnd += 2.0 * pi<double>;

				if (phiEnd < phiStart)
					phiEnd += 2.0 * pi<double>;

				double dphi = phiEnd - phiStart;

				throwing_assert(dphi >= 0.0);

				double l_mid = std::abs(r_mid) * dphi;

				// arc tile
				if (!tile)
					tile = std::make_shared<LaneArcTile>(offset0, direction0, r_mid, l_mid);
				else
					tile = std::make_shared<LaneArcTile>(tile, r_mid, l_mid);
			}
			else
			{
				throwing_assert(false);
			}

			// Determine connection.
			std::size_t section1 = std::numeric_limits<std::size_t>::max(), section2 = std::numeric_limits<std::size_t>::max();
			for (std::size_t j = 0; j < p.connections_.size(); ++j)
			{
				if (p.connections_[j].unknown_[1] == currentSocketId || p.connections_[j].unknown_[3] == currentSocketId)
				{
					section1 = p.connections_[j].unknown_[0];
					section2 = p.connections_[j].unknown_[2];
					break;
				}
			}

			throwing_assert(section1 != std::numeric_limits<std::size_t>::max() && section2 != std::numeric_limits<std::size_t>::max());

			// Translate all locations of current piece into longitudinal positions.
			for (std::size_t i = 0; i < p.barcodeEntries_.size(); ++i)
			{
				if (p.barcodeEntries_[i].section_ != section1 && p.barcodeEntries_[i].section_ != section2)
					continue;

				if (p.type_ == 4)
					continue;

				Vector2 x(p.barcodeEntries_[i].pose_.dx_, p.barcodeEntries_[i].pose_.dy_);
				if (roadPieces_[currentPieceId].reverse_)
				{
					if (p.type_ == 0)
					{
						// WARNING: This assumes that the modular straight tiles originate at (0, 0) and point along the positive x-axis.
						x = Vector2(tile->getTotalLength() * 0.001 - x[0], -x[1]);
					}
					else if (p.type_ == 1)
					{
						auto arcTile = std::dynamic_pointer_cast<LaneArcTile>(tile);
						double radius = -arcTile->getRadius() * 0.001;
						Vector2 ep(std::abs(radius), -radius);

						if (radius > 0.0)
						{
							// Right turn.
							x = Vector2(-ep[1] + x[1], ep[0] - x[0]);
						}
						else
						{
							// Left turn.
							x = Vector2(ep[1] - x[1], -ep[0] + x[0]);
						}
					}
					else
					{
						throwing_assert(false);
					}
				}

				// TODO rotate and shift intersection!!!

				Vector2 q = tile->getStartPoint() + tile->getStartDirection() * (x[0] * 1000.0) + tile->getStartDirection().getPerpendicularVectorLeft() * (x[1] * 1000.0);

				double distance, error;
				Lane::Identifier laneIdentifier;
				std::tie(laneIdentifier, distance, error) = prototype.mapConstrained(q, lane.size());

				if (error >= 0.5 * distanceLanes)
					std::cout << "WARNING: Could not reliably map cartesian location to longitudinal location." << std::endl;

				barcodeEntries[laneIdentifier][lane.size()][distance] = std::make_pair(currentPieceId, i);
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
			auto &barcodeEntriesInTile = barcodeEntries[i][j];
			auto tile = lane[j];

			double fillState0 = lane.getPositionOfTile(j);
			double fillState = fillState0;

			auto &piece = roadPieces_[tileToPieceIndex.at(j)];
			auto &pieceDescription = roadPieceDescriptions_.at(piece.getFullIdentifier());

			double pieceLenGap = pieceDescription.gapLength_ * 1000.0;
			double pieceLenMark = pieceDescription.barcodeLength_ * 1000.0;

			tile->pushMark(MT_SEPARATOR, MT_SEPARATOR, false, pieceLenMark, pieceLenGap);
			fillState += pieceLenMark;

			for (auto &p : barcodeEntriesInTile)
			{
				// p.first: longitudinal position
				// p.second: (piece index, barcode entry index)

				throwing_assert(tileToPieceIndex.at(j) == p.second.first);

				double markerPositionEnd = p.first + 0.5 * pieceLenMark;
				double markerPositionStart = p.first - 0.5 * pieceLenMark;

				double gap = markerPositionStart - fillState;
				throwing_assert(gap >= 0.0);

				tile->pushGap(gap);

				auto &barcodeEntry = pieceDescription.barcodeEntries_.at(p.second.second);
				if (barcodeEntry.locationIndex_ != -1)
				{
					std::size_t mask = (1 << ((piece.numBits_ - 1) - barcodeEntry.bit_));
					MarkType leftMark = (barcodeEntry.locationIndex_ & mask) ? MT_THICK : MT_THIN;
					MarkType rightMark = (piece.identifier_ & mask) ? MT_THICK : MT_THIN;

					if (piece.reverse_ || barcodeEntry.section_ == 2 || barcodeEntry.section_ == 3)
						std::swap(leftMark, rightMark);

					tile->pushMark(leftMark, rightMark, false, pieceLenMark, pieceLenGap);
				}
				else
				{
					MarkType leftMark = MT_THICKER;
					MarkType rightMark = MT_THIN;

					if (piece.reverse_ || barcodeEntry.section_ == 2 || barcodeEntry.section_ == 3)
						std::swap(leftMark, rightMark);

					tile->pushMark(leftMark, rightMark, false, pieceLenMark, pieceLenGap);
				}

				fillState = fillState0 + tile->getFillLength();

				if (std::abs(fillState - markerPositionEnd) > threshold)
					std::cout << "WARNING: Fillstate deviates by " << fillState - markerPositionEnd << " mm." << std::endl;
			}

			throwing_assert(fillState0 + tile->getTotalLength() - fillState - pieceLenMark + threshold >= 0.0);

			if (!tile->pushGap(fillState0 + tile->getTotalLength() - fillState - pieceLenMark - threshold))
				std::cout << "WARNING: Cannot push final gap." << std::endl;
			if (!tile->pushMark(MT_SEPARATOR, MT_SEPARATOR, false, pieceLenMark, pieceLenGap))
				std::cout << "WARNING: Cannot push separator." << std::endl;
		}

		track.addLane(lane);
	}
}

void AnkiOverdriveMap::loadRoadPieceDefinitions(boost::filesystem::path &pathToAppData)
{
	roadPieceDescriptions_.clear();

	for (std::size_t i = 0; i < roadPieces_.size(); ++i)
	{
		if (roadPieceDescriptions_.find(roadPieces_[i].getFullIdentifier()) == roadPieceDescriptions_.end())
		{
			auto file = pathToAppData / "files/expansion/assets/resources/basestation/config/modularRoadPieceDefinitionFiles/racing" / (boost::lexical_cast<std::string>(roadPieces_[i].inverted_) + "_" + boost::lexical_cast<std::string>(roadPieces_[i].numBits_) + "_" + boost::lexical_cast<std::string>(roadPieces_[i].identifier_) + ".txt");

			std::ifstream fin(file.c_str());
			if (!fin)
				throw std::runtime_error(std::string("Cannot open Anki Overdrive road piece definition file ") + file.string() + ".");

			AnkiOverdriveRoadPieceDescription pieceDescription;
			fin >> pieceDescription;

			roadPieceDescriptions_[roadPieces_[i].getFullIdentifier()] = pieceDescription;
		}
	}
}
