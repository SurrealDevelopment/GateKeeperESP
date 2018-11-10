//
// Created by Justin Hoogestraat on 11/1/18.
//

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
#include <can/IsoTpMessage.h>

/**
 * Retreives SID message of an IsoTpMessage
 * @param msg
 * @return - SID , 0 for invalid
 */
uint32_t getMessageSid(IsoTpMessage msg)
{
    if (msg.dataLength == 0)
    {
        return 0;
    }
    else
    {
        return (uint32_t) msg.data[0];
    }
}