//
// Created by Justin Hoogestraat on 10/29/18.
//
#ifndef UTIL_STRING_OPS
#define UTIL_STRING_OPS

#include <string>
#include <cstdio>

/**
 *  Copyright (C) 2018 Surreal Development LLC
 *  
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * Functions to help out with strings.
 * Mainly base64 things
 */
std::string toBase64(uint8_t * buf, uint32_t length)
{
    std::string s;
    for (int i = 0; i < length; i++)
    {
        char b[4];
        sprintf(b, "%02x", (unsigned char)buf[i]);
        s += b;
    }

    return s;
}


#endif