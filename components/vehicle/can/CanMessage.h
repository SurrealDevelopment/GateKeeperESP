//
// Created by Justin Hoogestraat on 10/31/18.
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
#ifndef FDOMESP_CANMESSAGE_H
#define FDOMESP_CANMESSAGE_H

#include <stdint.h>
#include <functional>


/**
 * Low level CAN Message object.
 */
class CanMessage{
public:

    uint32_t addresss;

    uint8_t data[64]; // now storing data locally for easier management

    uint32_t dataLength;

    bool flexibleDataRate; // if is CAN-FD

    bool extended;

    /**
     * Copies data argument to our data.
     * @param data
     */
    void copyData(uint8_t * data);

    CanMessage();

};


#endif //FDOMESP_CANMESSAGE_H
