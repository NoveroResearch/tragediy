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

#ifndef TRAGEDIY_UTIL_CONSTANTS_H
#define TRAGEDIY_UTIL_CONSTANTS_H

const double widthThin = 0.42;
const double widthThick = 1.1;
const double widthThicker = 1.7;
const double widthThinDouble = widthThin;
const double widthThickDouble = widthThick;
const double widthBaseThick = widthThick;
const double widthBaseThicker = widthThicker;
const double widthBaseThin = widthThin;
const double distThin = 0.5 * widthBaseThick + 0.5 * widthThin + 0.91;
const double distThick = 0.5 * widthBaseThick + 0.5 * widthThick + 0.56;
const double distThicker = 0.5 * widthBaseThick + 0.5 * widthThicker + 0.56;
const double distThinDouble = 1.62;
const double distThickDouble = 1.62;
const double distBase = 8.9;
const double lenMark = 10.0;
const double lenGap = 10.0;

const std::size_t bits = 8;

#endif
