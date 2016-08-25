#ifndef TRAGEDIY_TRACK_ANKIDRIVEMAP_H
#define TRAGEDIY_TRACK_ANKIDRIVEMAP_H

#include <tragediy/track/Track.h>
#include <tragediy/track/AnkiCommon.h>

#include <iostream>
#include <map>
#include <vector>

struct AnkiDriveRoadPieceLocation
{
	double x_, y_, z_;
	std::size_t a_, b_;
};

std::istream &operator>>(std::istream &in, AnkiDriveRoadPieceLocation &location);

struct AnkiDriveRoadPieceDescription
{
	double length_;
	double radius_;
	double barcodeLength_;
	double gapLength_;
	double transitionLength_;
	double intersectionCodeLength_;
	std::size_t numLanes_;

	std::vector<AnkiDriveRoadPieceLocation> locations_;

	std::size_t numConnections_;
};

std::istream &operator>>(std::istream &in, AnkiDriveRoadPieceDescription &pieceDescription);

struct AnkiDriveRoadPiece
{
	std::size_t identifier_;
	std::size_t speedLimit_;
};

std::istream &operator>>(std::istream &in, AnkiDriveRoadPiece &piece);

struct AnkiDriveMap
{
	std::vector<AnkiDriveRoadPiece> roadPieces_;
	std::vector<AnkiConnection> connections_;
	std::map<std::size_t, AnkiDriveRoadPieceDescription> roadPieceDescriptions_;
	AnkiFinishLine finishLine_;
	double offsetX_, offsetY_, theta_;
	double thetaExtra_{0.0};

	bool isValid() const;

	void loadRacingMap(boost::filesystem::path &pathToAppData, const char *name);
	void convert(Track &track, double rotationAngle = 0.0);

private:
	void loadRoadPieceDefinitions(boost::filesystem::path &pathToAppData);
};

std::istream &operator>>(std::istream &in, AnkiDriveMap &track);

#endif
