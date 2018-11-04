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
#ifndef FDOMESP_ISOTPMESSAGE_H
#define FDOMESP_ISOTPMESSAGE_H


#include <stdint.h>

#define MAX_ISO_TP_DATA_LENGTH 4095


/**
 * Higher level Iso-TP message
 */
class IsoTpMessage{

protected:
    /**
    * Types of message enumerable
    */
    enum Type {
        SINGLE_FRAME,
        FIRST_FRAME,
        CONSECUTIVE_FRAME,
        FLOW_CONTROL
    };
public:


    /**
     * address message will transmit or
     * address message recieved from
     */
    uint32_t address;

    /**
     * Pointer to high level data.
     * This is aggregated data. Meaning it can be anywhere
     * from 0 bytes to 4095 bytes
     *
     * Note because of our limited memory this data is likely going to be overwritten after
     * this message is processed.
     */
    uint8_t * data;

    /**
     * Length of data
     */
    uint32_t dataLength = 0;

    /**
     * Quickly tests if the message is valid (not too many bytes)
     * @return validity of message, true is valid
     */
    bool isValid();
};


#endif //FDOMESP_ISOTPMESSAGE_H
