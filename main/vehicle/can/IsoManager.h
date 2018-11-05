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
#ifndef FDOMESP_ISOMANAGER_H
#define FDOMESP_ISOMANAGER_H

#include "ICAN.h"
#include "IsoTpMessage.h"
#include <vector>
#include <utility>


#define MAX_FRAME_BUFFER 512 // max number of aggregated bytes

#define MAX_MULTIFRAME_SIZE 4095 // largest size of multiframe message
#define MAX_MULTIFRAME_FD_SIZE 0xffffffff // largest size of multiframe messages in FD mode. Yes that is 4 GB.

/**
 * Like its predecesor in Gretio. ISO Manager will do the logic for
 * single and multi frame messages. It will also do logic for repeating messages.
 *
 * In the OSI Model this will handle most of the transport layer (layer 4)
 * and n
 * https://en.wikipedia.org/wiki/ISO_15765-2
 *
 * ISO Manager also manages what is currently being listened to.
 */
class IsoManager : ICANListener {
private:

    /**
     * State of what IsoManager is doing
     */
    enum class state {
        NONE, // No or unknown state
        SENDING_MULTIFRAME,
        RECIEVING_MULTIFRAME,
    };

    ICAN * mIcan = nullptr;
    /**
     *  Buffer for frames ie multi frame
     */
     uint8_t buffer[MAX_FRAME_BUFFER];
public:

    virtual void onCANMessager(CanMessage message);


    /**
     * Constructor which injects which ICAN the IsoManager will use.
     * @param CAN - the device the manager will use
     */
    IsoManager(ICAN * CAN);

    /**
     * Alternative setter dependency injection
     * @param CAN - the device the manager will use
     */
    void setICAN(ICAN * CAN);
};


#endif //FDOMESP_ISOMANAGER_H