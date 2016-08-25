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
