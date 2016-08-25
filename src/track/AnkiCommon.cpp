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
