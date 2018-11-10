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
#ifndef GATEKEEPERESP_SAESHOWCURRENTDATARESPONSE_H
#define GATEKEEPERESP_SAESHOWCURRENTDATARESPONSE_H

#include "ServiceMessage.h"


class SAESHowCurrentDataResponse: public ServiceMessage {

    uint32_t pid;
    uint8_t dataBytes[4];
    uint32_t dataBytesLength = 0;
    SAESHowCurrentDataResponse(IsoTpMessage msg);
};


#endif //GATEKEEPERESP_SAESHOWCURRENTDATARESPONSE_H
