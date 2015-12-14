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

#ifndef ANKIWATCH_UTIL_ASSERTIONERROR_H
#define ANKIWATCH_UTIL_ASSERTIONERROR_H

#include <stdexcept>
#include <string>

class AssertionError : public std::logic_error
{
	std::string file_;
	int lineNumber_;
	std::string expression_;

public:
	explicit AssertionError(const std::string &file, int lineNumber, const std::string &expression, const std::string &what_arg);
	virtual ~AssertionError() throw();

	const std::string &getFileName() const;
	int getLineNumber() const;
	const std::string &getExpression() const;
};

inline AssertionError::AssertionError(const std::string &file, int lineNumber, const std::string &expression, const std::string &what_arg) : std::logic_error(what_arg), file_(file), lineNumber_(lineNumber), expression_(expression)
{
}

inline const std::string &AssertionError::getFileName() const
{
	return file_;
}

inline int AssertionError::getLineNumber() const
{
	return lineNumber_;
}

inline const std::string &AssertionError::getExpression() const
{
	return expression_;
}

#define stringize(x) stringize2(x)
#define stringize2(x) #x
#define throwing_assert(expression)                                                                                                                                                      \
	{                                                                                                                                                                                    \
		if (!(expression))                                                                                                                                                               \
		{                                                                                                                                                                                \
			throw AssertionError(__FILE__, __LINE__, #expression, std::string("(" #expression "), function ") + __PRETTY_FUNCTION__ + ", file " __FILE__ ", line " stringize(__LINE__)); \
		}                                                                                                                                                                                \
	}

#endif
