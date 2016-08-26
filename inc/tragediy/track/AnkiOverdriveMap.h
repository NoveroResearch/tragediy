#ifndef TRAGEDIY_TRACK_ANKIOVERDRIVEMAP_H
#define TRAGEDIY_TRACK_ANKIOVERDRIVEMAP_H

#include <tragediy/track/Track.h>
#include <tragediy/track/AnkiCommon.h>

#include <iostream>
#include <map>
#include <vector>


struct AnkiOverdriveLocalizableSection
{
	AnkiPose poseStart_, poseEnd_;
	std::size_t laneInfo_[6];
	std::size_t specialCodeInfo1_;
	double specialCodeInfo2_;
};

std::istream &operator>>(std::istream &in, AnkiOverdriveLocalizableSection &section);

struct AnkiOverdriveRoadPieceConnection
{
	std::size_t unknown_[4];
};

std::istream &operator>>(std::istream &in, AnkiOverdriveRoadPieceConnection &connection);

struct AnkiOverdriveRoadPieceLaneEntry
{
	AnkiPose poseStart_, poseStartValid_;
	AnkiPose poseEnd_, poseEndValid_;
	std::size_t laneNumber_, sectionIdentifier_;
};

std::istream &operator>>(std::istream &in, AnkiOverdriveRoadPieceLaneEntry &laneEntry);

struct AnkiOverdriveRoadPieceBarcodeEntry
{
	AnkiPose pose_;
	int locationIndex_;
	std::size_t bit_;
	std::size_t lane_;
	std::size_t section_;
};

std::istream &operator>>(std::istream &in, AnkiOverdriveRoadPieceBarcodeEntry &location);

struct AnkiOverdriveRoadPieceDescription
{
	std::size_t fileVersion_;
	std::size_t type_;
	std::size_t invertedColor_;
	std::size_t numBits_;
	std::size_t identifier_;
	std::size_t numLanes_;
	std::size_t numBorderLanes_;
	double transitionLengthStart_, transitionLengthEnd_;
	double gapLength_;
	double barcodeLength_;
	double intersectionCodeLength_;
	double distanceLanes_;
	std::size_t numSections_;

	std::vector<AnkiOverdriveLocalizableSection> localizableSections_;
	std::vector<AnkiPose> connectors_;
	std::vector<AnkiOverdriveRoadPieceConnection> connections_;
	std::vector<AnkiOverdriveRoadPieceLaneEntry> laneEntries_;
	std::vector<AnkiOverdriveRoadPieceBarcodeEntry> barcodeEntries_;

	std::size_t getPairedConnectorId(std::size_t connectorIdentifier) const;
};

std::istream &operator>>(std::istream &in, AnkiOverdriveRoadPieceDescription &pieceDescription);

struct AnkiOverdriveRoadPiece
{
	typedef std::tuple<std::size_t, std::size_t, std::size_t> FullIdentifier;

	std::size_t inverted_;
	std::size_t numBits_;
	std::size_t identifier_;
	std::size_t speedLimit_;
	std::size_t reverse_;

	FullIdentifier getFullIdentifier() const;
};

std::istream &operator>>(std::istream &in, AnkiOverdriveRoadPiece &piece);

struct AnkiOverdriveMap
{
	std::vector<AnkiOverdriveRoadPiece> roadPieces_;
	std::vector<AnkiConnection> connections_;
	std::map<AnkiOverdriveRoadPiece::FullIdentifier, AnkiOverdriveRoadPieceDescription> roadPieceDescriptions_;
	AnkiFinishLine finishLine_;
	double offsetX_, offsetY_, theta_;
	double thetaExtra_{0.0};

	bool isValid() const;

	void loadRacingMap(boost::filesystem::path &pathToAppData, const char *name);
	void convert(Track &track, double rotationAngle = 0.0);

private:
	void loadRoadPieceDefinitions(boost::filesystem::path &pathToAppData);
};

std::istream &operator>>(std::istream &in, AnkiOverdriveMap &track);

#endif
