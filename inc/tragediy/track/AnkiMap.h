#ifndef TRAGEDIY_TRACK_ANKIMAP_H
#define TRAGEDIY_TRACK_ANKIMAP_H

#include <iostream>
#include <map>
#include <vector>
#include <tragediy/track/Track.h>

struct AnkiPose
{
	double dx_, dy_, dphi_;
};

std::istream &operator>>(std::istream &in, AnkiPose &pose);

struct AnkiLocation
{
	double x_, y_, z_;
	std::size_t a_, b_;
};

std::istream &operator>>(std::istream &in, AnkiLocation &location);

struct AnkiPieceDescription
{
	double length_;
	double radius_;
	double barcodeLength_;
	double gapLength_;
	double transitionLength_;
	double intersectionCodeLength_;
	std::size_t numLanes_;

	std::vector<AnkiLocation> locations_;

	std::size_t numConnections_;
};

std::istream &operator>>(std::istream &in, AnkiPieceDescription &pieceDescription);

struct AnkiPiece
{
	std::size_t identifier_;
	std::size_t speedLimit_;
};

std::istream &operator>>(std::istream &in, AnkiPiece &piece);

struct AnkiConnection
{
	std::size_t identifierX_, connectionX_, identifierY_, connectionY_;
};

std::istream &operator>>(std::istream &in, AnkiConnection &connection);

struct AnkiFinishLine
{
	std::size_t pieceId_, drivableSectionId_;
	double start_, end_;
};

std::istream &operator>>(std::istream &in, AnkiFinishLine &finishLine);

struct AnkiMap
{
	std::vector<AnkiPiece> pieces_;
	std::vector<AnkiConnection> connections_;
	std::map<std::size_t, AnkiPieceDescription> pieceDescriptions_;
	AnkiFinishLine finishLine_;
	double offsetX_, offsetY_, theta_;
	double thetaExtra_{0.0};

	bool isValid() const;

	void loadRacingMap(boost::filesystem::path &pathToAppData, const char *name);
	void convert(Track &track, double rotationAngle = 0.0);

private:
	void loadRoadPieceDefinitions(boost::filesystem::path &pathToAppData);
};

std::istream &operator>>(std::istream &in, AnkiMap &track);

#endif
