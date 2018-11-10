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
#include "NegativeResponse.h"
#include "services.h"

NegativeResponse::NegativeResponse(IsoTpMessage msg): ServiceMessage(msg) {

    if (msg.dataLength < 3 || msg.data[1] != 0x7f)
    {
        this->validity = Validity ::INVALID;
        return;
    }

    if (msg.dataLength >= 3)
    {
        this->reqServiceId = (uint32_t )msg.data[1];
        this->returnCode = (uint32_t)msg.data[2];
    }
    // this is optional
    if (msg.dataLength >= 4)
    {
        this->deviceControlLimitExceeded = (uint32_t)msg.data[3] << 8;
        this->deviceControlLimitExceeded |= (uint32_t)msg.data[4];

    }

}

void NegativeResponse::writeToBuffer(uint8_t *buffer, uint32_t bufferSize, uint32_t frameNumber) {
    ServiceMessage::writeToBuffer(buffer, bufferSize, frameNumber);

    buffer[1] = (uint8_t )(reqServiceId & 0xff);
    buffer[2] = (uint8_t )(returnCode & 0xff);
    if (deviceControlLimitExceeded != 0)
    {
        buffer[3] = (uint8_t )(deviceControlLimitExceeded & 0xff00) >> 8;
        buffer[4] = (uint8_t )(deviceControlLimitExceeded & 0xff);
    }

}

NegativeResponse::NegativeResponse(uint32_t addr, uint32_t reqServiceId, uint32_t returnCode, uint32_t extra)
        : ServiceMessage(addr, 0x7f)
{
    this->deviceControlLimitExceeded = extra;
    this->returnCode = returnCode;
    this->reqServiceId = reqServiceId;

    if (extra == 0)
    {
        this->dataLength = 3;
    }
    else
    {
        this->dataLength = 5;
    }


}