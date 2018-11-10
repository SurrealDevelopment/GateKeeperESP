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
#ifndef GATEKEEPERESP_SERVICEMESSAGE_H
#define GATEKEEPERESP_SERVICEMESSAGE_H

#include <stdint.h>
#include <can/IsoTpMessage.h>

/**
 * Higher level object of a service message
 */
class ServiceMessage: public IsoTpMessage {
public:
    enum class Validity {
        VALID,
        INVALID
    };

    /**
     * Validity of message.
     */
    Validity validity = Validity ::INVALID;

    uint32_t serviceId;

    /**
     * From a raw iso tp message. Tries to read service id of message.
     */
    explicit ServiceMessage(IsoTpMessage);

    /**
     * Build new message
     * @param sid
     */
    ServiceMessage(uint32_t addr, uint32_t sid);



    void writeToBuffer(uint8_t *buffer, uint32_t bufferSize, uint32_t frameNumber) override;

};


#endif //GATEKEEPERESP_SERVICEMESSAGE_H
