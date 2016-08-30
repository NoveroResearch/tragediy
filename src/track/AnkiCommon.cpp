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

#include <tragediy/track/AnkiCommon.h>

std::istream &skipAhead(std::istream &in)
{
	while (isspace(in.peek()))
		in.get();

	std::string line;
	if (in.peek() == '#')
	{
		std::getline(in, line);
		return skipAhead(in);
	}

	return in;
}

std::istream &operator>>(std::istream &in, AnkiPose &pose)
{
	skipAhead(in) >> pose.dx_;
	skipAhead(in) >> pose.dy_;
	skipAhead(in) >> pose.dphi_;

	return in;
}

std::istream &operator>>(std::istream &in, AnkiConnection &connection)
{
	skipAhead(in) >> connection.identifierX_;
	skipAhead(in) >> connection.connectionX_;
	skipAhead(in) >> connection.identifierY_;
	skipAhead(in) >> connection.connectionY_;

	return in;
}

std::istream &operator>>(std::istream &in, AnkiFinishLine &finishLine)
{
	skipAhead(in) >> finishLine.pieceId_;
	skipAhead(in) >> finishLine.drivableSectionId_;
	skipAhead(in) >> finishLine.start_;
	skipAhead(in) >> finishLine.end_;

	return in;
}
