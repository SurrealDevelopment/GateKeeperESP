//
// Created by Justin Hoogestraat on 11/4/18.
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
#ifndef FDOMESP_ISOFRAGMENT_H
#define FDOMESP_ISOFRAGMENT_H

#include "CanMessage.h"


/**
 * Converts Standard can message to an ISO-TP Message
 * and aids in creation of converting ISO TP frames to raw can messages.
 *
 * Works with both ISO-TP for CAN and ISO-TYP for CAN-FD
 */
class IsoFragment{
public:
    enum class Type{
        INVALID, // not a valid message
                SINGLE_FRAME,
                FIRST_FRAME,
                CONSEC_FRAME,
                FLOW_CONTROL
    };


    enum class FlowControlFlag
    {
        ContinueToSend,
        Wait,
        OverflowAbort
    };
private:
    Type mType = Type::INVALID;


public:
    // used for all frames
    bool mFlexRequired = false; // flexible data rate is required

    uint32_t addr;
    uint8_t * rawData; // pointer to raw data
    uint32_t rawDataLength; // raw data length


    // used for first, consec, and single frame
    uint8_t * data; // pointer to data of fragment. Handle immediately
    uint32_t dataLength =0; //length of data or data just stored in fragment
    uint32_t totalDataLength =0; //length of total data for first frame

    // used for consec frames only
    uint32_t cseq =0;

    // Used for flow control only
    FlowControlFlag fcFlag;
    uint32_t blockSize=0;
    uint32_t separationTime=0; // seperation time in microseconds

    Type getType();


    /**
     * Create and parse iso fragment from CanMessage
     * @param message - message to parse
     */
    IsoFragment(CanMessage message);


    /**
     * Create single frame fragment
     * @param addr address to send
     * @param data data to send
     * @param buffer buffer to store compelte message
     * @param dataLength length of data
     * @return
     */
    static IsoFragment makeSingleFrame(bool flex,
            uint32_t addr, uint8_t * data, uint8_t * buffer, uint32_t dataLength);


    /**
     * Create a flow control fragment
     * @param addr address to send
     * @param buffer buffer to store complete message
     * @param flag flag of flow contorol
     * @param blockSize block size of flow control
     * @param st seperation time of flow control
     * @return
     */
    static IsoFragment makeFlowControl(
            uint32_t addr,
            uint8_t *buffer,
            FlowControlFlag flag,
            uint32_t blockSize = 0,
            uint32_t st = 0);

    /**
     * Makes a first frame
     * @param addr - address to send to
     * @param data - pointer to data (must be at least 6 long)
     * @param buffer - buffer to put raw message into
     * @param totalDataLength - total dat alength
     * @return
     */
    static IsoFragment makeFirstFrame(bool flex, uint32_t addr, uint8_t * data, uint8_t * buffer, uint32_t totalDataLength);



    /**
     * Make a consec frame
     * @param addr - address to send to
     * @param data - pointer to data
     * @param buffer - pointer to buffer for raw message
     * @param dataLength - how much data (max 7). This is not the same as total length from first frame
     * @param cseq - cseq
     * @return IsoFragment
     */
    static IsoFragment makeConsecFrame( uint32_t addr, uint8_t * data, uint8_t * buffer, uint32_t dataLength, uint32_t cseq);


    /**
     * Convert this frame to a can message.
     * Reuses buffers.
     * @param flex - whether to be a CAN FD Message or not
     * @return
     */
    CanMessage toCanMessage(bool flex = false);



};


#endif //FDOMESP_ISOFRAGMENT_H
