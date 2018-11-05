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
#include "IsoFragment.h"

IsoFragment::Type IsoFragment::getType() {
    return this->mType;
}

IsoFragment::IsoFragment(CanMessage message) {


    // common features
    this->rawData = message.data;
    this->rawDataLength = message.dataLength;
    this->addr = message.addresss;

    if (message.dataLength < 1)
    {
        this->mType = Type::INVALID; // invalid message
        return;
    }

    if (message.flexibleDataRate)
        this->mFlexRequired = true;

    uint32_t type = message.data[0] >> 4;
    if (type == 0 && message.dataLength >= 2)
    {
        this->mType = Type::SINGLE_FRAME; // single frame
        this->dataLength = message.data[0] & 0x0fu;

        // special case for CAN-FD
        if (this->dataLength == 0)
        {
            this->dataLength = message.data[1];
            if (this->dataLength + 2 > message.dataLength)
            {
                this->mType = Type::INVALID; // invalid message
                return;
            }
            this->data = &(message.data[2]); // everything after second byte is data

        }
        else
        {
            if (this->dataLength + 1 > message.dataLength)
            {
                this->mType = Type::INVALID; // invalid message
                return;
            }
            this->data = &(message.data[1]); // everything after first byte is data
        }

    }
    else if (type == 1 && message.dataLength >= 8) // need CAN Message for first frame
    {
        this->mType = Type::FIRST_FRAME; // First frame

        // half byte 1..3 are length
        this->totalDataLength = message.data[0] & 0x0fu;
        this->totalDataLength = this->totalDataLength << 8;
        this->totalDataLength |= message.data[1];

        if (this->totalDataLength == 0) // tell tale sign of a flexible frame
        {
            this->totalDataLength = message.data[2] << 8;
            this->totalDataLength |= message.data[3];
            if (64 > message.dataLength) // must be full
            {
                this->mType = Type::INVALID; // invalid message
                return;
            }
            this->data = &message.data[4]; // data starts after 4th byte
            this->dataLength = 60;
        }
        else
        {
            this->data = &message.data[2]; // data starts after 2nd byte
            this->dataLength = 6;
        }


    }
    else if (type == 2)
    {
        this->mType = Type::CONSEC_FRAME; // consec frame message
        this->cseq= message.data[0] & 0x0fu;
        this->dataLength = message.dataLength-1; // this is just general rule, always base on first frame
        this->data = &message.data[1];
    }
    else if (type == 3 && message.dataLength >= 3)
    {
        this->mType = Type::FLOW_CONTROL; // flow control
        uint32_t fcFlag = message.data[0] & 0b11u;
        if (fcFlag == 0)
            this->fcFlag = FlowControlFlag::ContinueToSend;
        else if (fcFlag == 1)
            this->fcFlag = FlowControlFlag::Wait;
        else if (fcFlag == 2)
            this->fcFlag = FlowControlFlag::OverflowAbort;
        else
            this->mType = Type::INVALID; // invalid message

        this->blockSize = message.data[1];
        uint32_t st = message.data[2];


        if (st <= 127)
        {
            this->separationTime = st * 1000;
        }
        else if (st >= 0xf1 && st <= 0xf9)
        {
            st = st - 0xf0;
            this->separationTime = st * 100;
        }
        else
            this->mType = Type::INVALID; // invalid message



    }
    else
    {
        this->mType = Type::INVALID; // invalid message
    }
}

IsoFragment IsoFragment::makeSingleFrame(bool flex,uint32_t addr, uint8_t *data, uint8_t *buffer, uint32_t dataLength) {

    IsoFragment frag;

    frag.addr = addr;
    frag.dataLength=dataLength;
    frag.rawData = buffer;
    frag.mType = Type::INVALID;

    if(flex)
    {
        if (dataLength > 7 ) return frag; // cant send more than 7 bytes
    }
    else
    {
        if (dataLength > 62) return frag; // 2 bytes are used for can-fd
    }


    frag.mType = Type::SINGLE_FRAME;



    if (flex)
    {
        buffer[0] = 0;
        buffer[1] = (uint8_t)dataLength;
        // copy buffer to buffer
        for (int i = 0; i < dataLength; i++)
        {
            buffer[i+2] = data[i];
        }
        frag.data = &buffer[2];
        frag.rawDataLength = 2+dataLength;


    }
    else
    {
        buffer[0] = (uint8_t)(dataLength &0x0f);
        // copy data to buffer
        for (int i = 0; i < dataLength; i++)
        {
            buffer[i+1] = data[i];
        }
        frag.data = &buffer[1];
        frag.rawDataLength = 1+dataLength;

    }

    return frag;

}

IsoFragment IsoFragment::makeFlowControl(uint32_t addr, uint8_t *buffer, IsoFragment::FlowControlFlag flag, uint32_t blockSize, uint32_t st) {
    // flow control needs nothing special for CAN-FD

    IsoFragment f;

    f.addr = addr;
    f.separationTime = st;
    f.blockSize = blockSize;
    f.mType = Type::FLOW_CONTROL;
    f.fcFlag=flag;
    f.rawData = buffer;

    if (flag == FlowControlFlag::ContinueToSend)
        buffer[0] = 0;
    else if (flag == FlowControlFlag::Wait)
        buffer[0] = 1;
    else if (flag ==  FlowControlFlag::OverflowAbort)
        buffer[0] = 2;

    buffer[0] |= (3 << 4);
    buffer[1] = (uint8_t )blockSize;
    buffer[2] = (uint8_t )st;
    f.rawDataLength =3;
    f.rawData = buffer;

    return f;


}

IsoFragment IsoFragment::makeFirstFrame(bool flex, uint32_t addr, uint8_t *data, uint8_t *buffer, uint32_t totalDataLength) {

    IsoFragment f;

    f.addr = addr;
    f.totalDataLength = totalDataLength;
    f.mType = Type::INVALID;

    // sanity checks
    if (!flex) {
        if (totalDataLength < 8 || totalDataLength > 4095) return f;
    }
    else {
        if (totalDataLength < 60) return f;
    }

    f.mType = Type::FLOW_CONTROL;

    if (!flex)
    {
        buffer[0] = (uint8_t )(totalDataLength >> 8);
        buffer[1] = (uint8_t )(totalDataLength & 0xff);
        buffer[0] |= (1 << 4);
        // copy data
        for (int i = 0; i < 6; i++)
        {
            buffer[i+2] = data[i];
        }

        f.data = &buffer[2];
        f.dataLength = 6;
        f.rawDataLength = 8;
        f.rawData = buffer;
    }
    else
    {

        buffer[0] = (1 << 4);
        buffer[1] = 0;
        // for FD data length will be third and fourth bytes
        buffer[2] = (uint8_t)((totalDataLength >> 8) & 0xff);
        buffer[3] = (uint8_t)((totalDataLength) & 0xff);
        // copy data
        for (int i = 0; i < 60; i++)
        {
            buffer[i+4] = data[i];
        }

        f.data = &buffer[4];
        f.dataLength = 60;
        f.rawDataLength = 64;
        f.rawData = buffer;

    }

    return f;

}


IsoFragment
IsoFragment::makeConsecFrame(bool flex, uint32_t addr, uint8_t *data, uint8_t *buffer, uint32_t dataLength, uint32_t cseq) {

    IsoFragment f;

    f.rawDataLength = 1 + dataLength;
    f.data = &buffer[1];
    f.addr = addr;
    f.dataLength = dataLength;

    f.mType = Type::CONSEC_FRAME;

    buffer[0] = (uint8_t )cseq & (uint8_t )0x0f;
    buffer[0] |= (2 << 4);

    if (flex) f.mFlexRequired = true;

    if (!f.mFlexRequired && dataLength >7)
        f.mType = Type::INVALID; // sanity check
    else if (dataLength > 63)
        f.mType = Type::INVALID;

    if (f.mType == Type::INVALID)
        return f;

    // copy data
    for (int i = 0; i < dataLength; i++)
    {
        buffer[i+1] = data[i];
    }

    return f;
}

/**
 * Converts to can message.
 * @param flex - Can-FD or not
 * @return
 */
CanMessage IsoFragment::toCanMessage(bool flex) {

    if (!flex && mFlexRequired) flex = true; // sanity check
    CanMessage msg;
    msg.dataLength = this->rawDataLength;
    msg.data = this->rawData;
    msg.addresss = this->addr;
    msg.flexibleDataRate = flex; // its okay to make a non-flex-required frame a flex frame.


    return msg;

}

