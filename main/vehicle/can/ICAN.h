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
#ifndef FDOMESP_CANINTERFACE_H
#define FDOMESP_CANINTERFACE_H

#include "ICANListener.h"

/**
 * ICAN is the general interface for a CAN Device. CAN Device will fullfill
 * the data link layer of the OSI 7 model for CAN.
 */
class ICAN {

public:


    enum class Priority {
        High,
        Med,
        Low
    };

    enum class MessageWriteResult {
        OKAY,
        FAIL
    };


    virtual void setCanListener(ICANListener * listener) = 0;


    /**
     * Write a message to the CAN Device
     *
     * @param message - message to write.
     * @param priority - priority of message
     * @param onFinish - higher order function called when on finish from ICAN thread
     */
    virtual void writeMessage(CanMessage message, Priority priority, std::function<void(MessageWriteResult)> onFinish) = 0;




};

#endif //FDOMESP_CANINTERFACE_H
