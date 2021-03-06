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

#ifndef FDOMESP_UAC_H
#define FDOMESP_UAC_H

#include <string>
#include "util/stringops.cpp"

/**
 * Manages user access control to specific features
 */
class uac{

public:
    /**
     * Simply function to quickly check if a token is valid or not.
     * True implies the token is on record and not invalid for any reason.
     * @param token
     * @return validity, true is valid
     */
    bool tokenIsValid(std::string token);

    /**
     * Generate a new token to be shared.
     * Token is base 64 coded.
     * @return token in string format
     */
    std::string generateNewToken();



};


#endif //FDOMESP_UAC_H
