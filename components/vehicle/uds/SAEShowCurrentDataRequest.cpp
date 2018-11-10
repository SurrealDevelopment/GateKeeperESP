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
#include "SAEShowCurrentDataRequest.h"
SAEShowCurrentData::SAEShowCurrentData (uint32_t addr, uint32_t pid)
    : ServiceMessage(addr, SAE_SHOW_CURRENT_DATA)
{
    this->pid = pid;
    this->dataLength = 2; // 1 for sid, 1 for pid
}

void SAEShowCurrentData::writeToBuffer(uint8_t *buffer, uint32_t bufferSize, uint32_t frameNumber)
{
    // always call super
    ServiceMessage::writeToBuffer(buffer, bufferSize, frameNumber);
    // just write, this will never be bigger than
    buffer[1] = (uint8_t)pid;
}
