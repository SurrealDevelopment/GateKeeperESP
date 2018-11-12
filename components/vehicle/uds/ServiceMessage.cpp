//
// Created by Justin Hoogestraat on 11/10/18.
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
#include "ServiceMessage.h"
#include "uds.cpp"


ServiceMessage::ServiceMessage(IsoTpMessage  msg) {

    this->transmitAddress = msg.transmitAddress;


    uint32_t serviceId = getMessageSid(msg);

    if (serviceId == 0)
        this->validity = Validity ::INVALID;
    else
        this->validity = Validity ::VALID;

    this->serviceId = serviceId;

}

ServiceMessage::ServiceMessage(uint32_t addr, uint32_t sid) {
    this->transmitAddress = addr;

    this->validity = Validity ::VALID;
    this->serviceId = sid;


}


void ServiceMessage::writeToBuffer(uint8_t *buffer, uint32_t bufferSize, uint32_t frameNumber) {

    // only write if first frame
    if(frameNumber == 0)
    {
        this->data = buffer;
        this->dataLength = 1;
        this->data[0] = (uint8_t) (this->serviceId & 0xff);
    }
}
