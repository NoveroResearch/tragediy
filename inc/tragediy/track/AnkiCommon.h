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

#ifndef TRAGEDIY_TRACK_ANKICOMMON_H
#define TRAGEDIY_TRACK_ANKICOMMON_H

#include <iostream>
#include <cctype>

std::istream &skipAhead(std::istream &in);

struct AnkiPose
{
	double dx_, dy_, dphi_;
};

std::istream &operator>>(std::istream &in, AnkiPose &pose);

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

#endif
