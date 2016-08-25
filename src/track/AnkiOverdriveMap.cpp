#include <tragediy/track/AnkiOverdriveMap.h>

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

std::istream &operator>>(std::istream &in, AnkiOverdriveRoadPieceLocation &location)
{
	skipAhead(in) >> location.pose_;
	skipAhead(in) >> location.locationIndex_;
	skipAhead(in) >> location.bit_;
	skipAhead(in) >> location.lane_;
	skipAhead(in) >> location.section_;

	std::cout << location.pose_.dx_ << " " << location.pose_.dy_ << " " << location.pose_.dphi_ << std::endl;

	return in;
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

	std::cout << "road piece " << pieceDescription.identifier_ << std::endl;

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

		pieceDescription.locations_.clear();
		pieceDescription.locations_.resize(numLocations);

		for (std::size_t i = 0; i < numLocations; ++i)
			in >> pieceDescription.locations_[i];
	}

	return in;
}

std::istream &operator>>(std::istream &in, AnkiOverdriveRoadPiece &piece)
{
	skipAhead(in) >> piece.unknown_;
	skipAhead(in) >> piece.numBits_;
	skipAhead(in) >> piece.identifier_;
	skipAhead(in) >> piece.speedLimit_;
	skipAhead(in) >> piece.reverse_;

	return in;
}

AnkiOverdriveRoadPiece::FullIdentifier AnkiOverdriveRoadPiece::getFullIdentifier() const
{
	return std::make_tuple(unknown_, numBits_, identifier_);
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
	auto file = pathToAppData / "files/expansion/assets/resources/basestation/config/mapFiles/racing" / name;

	std::ifstream fin(file.c_str());
	if (!fin)
		throw std::runtime_error(std::string("Cannot open Anki Overdrive map file ") + file.string() + ".");

	fin >> (*this);

	loadRoadPieceDefinitions(pathToAppData);
}

void AnkiOverdriveMap::loadRoadPieceDefinitions(boost::filesystem::path &pathToAppData)
{
	pieceDescriptions_.clear();

	for (std::size_t i = 0; i < roadPieces_.size(); ++i)
	{
		if (pieceDescriptions_.find(roadPieces_[i].getFullIdentifier()) == pieceDescriptions_.end())
		{
			auto file = pathToAppData / "files/expansion/assets/resources/basestation/config/modularRoadPieceDefinitionFiles/racing" / (boost::lexical_cast<std::string>(roadPieces_[i].unknown_) + "_" + boost::lexical_cast<std::string>(roadPieces_[i].numBits_) + "_" + boost::lexical_cast<std::string>(roadPieces_[i].identifier_) + ".txt");

			std::ifstream fin(file.c_str());
			if (!fin)
				throw std::runtime_error(std::string("Cannot open Anki Overdrive road piece definition file ") + file.string() + ".");

			AnkiOverdriveRoadPieceDescription pieceDescription;
			fin >> pieceDescription;

			pieceDescriptions_[roadPieces_[i].getFullIdentifier()] = pieceDescription;
		}
	}
}
