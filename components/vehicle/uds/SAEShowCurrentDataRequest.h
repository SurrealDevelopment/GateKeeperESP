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
#ifndef GATEKEEPERESP_SAESHOWCURRENTDATA_H
#define GATEKEEPERESP_SAESHOWCURRENTDATA_H

#include "services.h"
#include "ServiceMessage.h"

// J1979 mode 1
class SAEShowCurrentData: public ServiceMessage {


public:
    uint32_t pid;
    // builds a request
    SAEShowCurrentData (uint32_t addr, uint32_t pid);

    void writeToBuffer(uint8_t *buffer, uint32_t bufferSize, uint32_t frameNumber) override;


};


#endif //GATEKEEPERESP_SAESHOWCURRENTDATA_H
