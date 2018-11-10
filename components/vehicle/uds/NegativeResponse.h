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
#ifndef GATEKEEPERESP_NEGATIVERESPONSE_H
#define GATEKEEPERESP_NEGATIVERESPONSE_H

#include "ServiceMessage.h"

/**
 * Negative 7F response
 */
class NegativeResponse: public ServiceMessage  {


public:
    uint32_t reqServiceId = 0;
    uint32_t returnCode = 0;
    uint32_t deviceControlLimitExceeded = 0;
    /**
     * Tries to build negative response from raw message
     * @param msg
     */
    explicit NegativeResponse(IsoTpMessage msg);

    /**
     * Build a negative response into the buffer
     * @param buffer - buffer to write into
     * @param reqServiceId - service id
     * @param returnCode - return code
     * @param extra - optional extra, 0 won't be used
     */
    NegativeResponse(uint32_t addr, uint32_t reqServiceId, uint32_t returnCode, uint32_t extra = 0);

    void writeToBuffer(uint8_t *buffer, uint32_t bufferSize, uint32_t frameNumber) override;


};


#endif //GATEKEEPERESP_NEGATIVERESPONSE_H
