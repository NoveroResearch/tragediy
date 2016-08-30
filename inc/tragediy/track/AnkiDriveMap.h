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

#ifndef TRAGEDIY_TRACK_ANKIDRIVEMAP_H
#define TRAGEDIY_TRACK_ANKIDRIVEMAP_H

#include <tragediy/track/AnkiCommon.h>
#include <tragediy/track/Track.h>

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

	std::size_t getPairedConnectorId(std::size_t connectorIdentifier) const;
};

std::istream &operator>>(std::istream &in, AnkiDriveRoadPieceDescription &pieceDescription);

struct AnkiDriveRoadPiece
{
	typedef std::size_t FullIdentifier;

	std::size_t identifier_;
	std::size_t speedLimit_;

	FullIdentifier getFullIdentifier() const;
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
