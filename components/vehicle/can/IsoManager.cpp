//
// Created by Justin Hoogestraat on 10/29/18.
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
#include <esp_log.h>

#include "IsoManager.h"
#include "IsoFragment.h"



void IsoManager::onCANMessager(CanMessage message) {

    ESP_LOGI("TEST", "Recieved message");
    xSemaphoreTake(this->semaphore, portMAX_DELAY);

    auto lookup = this->mListenList.find(message.addresss);

    if (lookup == this->mListenList.end()) {
        // Ignore message
        xSemaphoreGive(this->semaphore);
        return;
    }


    // process message
    auto frag = IsoFragment(message);

    switch (frag.getType())
    {
        case IsoFragment::Type::INVALID: {
            // ignore
            ESP_LOGI("TEST", "Recieved INVALID");

            xSemaphoreGive(this->semaphore);
            break;
        }

        case IsoFragment::Type::SINGLE_FRAME: {
            // forward message

            ESP_LOGI("TEST", "Recieved SINGLE");

            IsoTpMessage tp;
            tp.address = frag.addr;
            tp.dataLength = frag.dataLength;
            tp.data = frag.data;

            IsoUpdate up;

            up.msg = tp;
            up.type = IsoUpdate::Type::MSG_COMPLETE;

            for (const auto& it : lookup->second)
            {
                forwardMessage(up, it.first, it.second);
            }

            xSemaphoreGive(this->semaphore);


            break;
        }
        case IsoFragment::Type::CONSEC_FRAME: {

            ESP_LOGI("TEST", "Recieved CONSEC");

            xSemaphoreGive(this->semaphore);

            break;
        }
        case IsoFragment::Type::FLOW_CONTROL: {

            ESP_LOGI("TEST", "Recieved FLOW");

            if (this->mCurState == State::SENDING_MULTIFRAME)
            {

            }
            xSemaphoreGive(this->semaphore);

            break;
        }
        case IsoFragment::Type::FIRST_FRAME: {
            ESP_LOGI("TEST", "Recieved FIRST");

            xSemaphoreGive(this->semaphore);
        }
        default:
            xSemaphoreGive(this->semaphore);
            break;
    }

}

/**
 * Forward message.
 * Note this is called with a lock on the semaphore.
 * @param msg
 * @param channelId
 * @param fun
 */
void IsoManager::forwardMessage(IsoUpdate msg, uint32_t channelId, updateFun fun) {
    fun(msg);


}


IsoManager::IsoManager(ICAN *CAN) {
    this->mIcan = CAN;

    auto a = xSemaphoreCreateBinary();

    this->messageQueue = xQueueCreate(5, sizeof(IsoTpMessage));


    xSemaphoreGive(a);

    this->semaphore = a;
}

void IsoManager::setICAN(ICAN *CAN) {
    this->mIcan = CAN;

    free(this->messageQueue);
    free(this->semaphore);
}

uint32_t IsoManager::openNonStandardChannel(uint32_t addr, IsoManager::updateFun) {
    return 0;
}

uint32_t IsoManager::openStandardChannel(uint32_t addr, IsoManager::updateFun fun) {
    xSemaphoreTake(this->semaphore, portMAX_DELAY);

    uint32_t id = channelCounter++;
    mIdToAddr.insert({id, addr}); // create id to addr entry

    auto channelMapLookup = mListenList.find(addr);


    ESP_LOGI("TEST", "Point A");

    if (channelMapLookup == mListenList.end())
    {
        ESP_LOGI("TEST", "Point B");

        // entry doesn't exist, create it
        std::map<uint32_t , updateFun> a;

        // tell ican to listen to the address
        mIcan->listenToWithFallback(addr, false); //@TODO extended

        mListenList.insert({addr, a});

        channelMapLookup = mListenList.find(addr);
        if (channelMapLookup == mListenList.end())
            id = 0; // no memory
    }
    if (id != 0)
    {
        if (!channelMapLookup->second.insert({id, fun}).second)
        {
            ESP_LOGI("TEST", "Point C");

            id = 0; // no memory
        }
        ESP_LOGI("TEST", "Point D");

    }

    xSemaphoreGive(this->semaphore);

    return id;

}

void IsoManager::closeChannel(uint32_t channelID) {
    xSemaphoreTake(this->semaphore, portMAX_DELAY);

    // map channel Id to address
    auto addr = mIdToAddr.find(channelID);


    if (addr != mIdToAddr.end())
    {
        mIdToAddr.erase(channelID);
        auto listener = mListenList.find(addr->second);

        if (listener != mListenList.end())
        {
            auto channelMap = listener->second;
            auto channelLookup = channelMap.find(channelID);
            if (channelLookup != channelMap.end())
            {
                channelMap.erase(channelLookup);
                if (channelMap.empty())
                {
                    // remove if empty
                    mListenList.erase(listener);
                    mIcan->stopListeningTo(addr->second, false); //@TODO extended
                }
            }


        }

    }

    xSemaphoreGive(this->semaphore);



}



bool IsoManager::queueIsoMessage(IsoTpMessage msg, IsoManager::updateFun fun) {

    auto result = xQueueSend(this->messageQueue, &msg, 5000 / portTICK_RATE_MS);

    if (result != pdTRUE)
        return false;

    doNext();

    return true;


}

void IsoManager::doNext() {
    xSemaphoreTake(this->semaphore, portMAX_DELAY);

    if (this->mCurState == State::NONE)
    {
        IsoTpMessage msg;
        auto result = xQueueReceive(this->messageQueue, &msg, 0);

        if (result == pdTRUE)
        {
            // send it
            processSendTpMessage(msg);
        }
    }


    xSemaphoreGive(this->semaphore);
}

/**
 *
 * @param dataSize  - size of data
 * @param fdSupported  - if fd is supported
 * @return - # of frames. 0 if invalid
 */
uint32_t calculateNeededFrames(uint32_t dataSize, bool fdSupported = false) {
    if (fdSupported)
    {
        if (dataSize <= 62)
            return 1; // can fit into single frame
        else {
            auto consecFrames = (dataSize - 58u) / 63u; // first frame is 58 bytes
            auto rem = (dataSize - 58u) % 63u; // consec frames hold up to 63 bytes
            if (rem != 0) consecFrames++;
            return consecFrames + 1; // 1 for first frame
        }

    } else {
        if (dataSize <= 7) return 1; // can fit into single frame
        else if (dataSize > 4095)
            return 0; // cant fit
        else {
            auto consecFrames = (dataSize - 6u) /  7u; // 6 bytes for ff, 7 for rem
            auto rem = (dataSize - 6u) % 7u;
            if (rem != 0) consecFrames++; // need + 1 frame for remaining bytes

            return consecFrames + 1; // plus 1 frame for first frame
        }
    }
};

bool IsoManager::processSendTpMessage(IsoTpMessage msg) {
    // assume no fd for now
    bool flex = false;


    uint32_t neededFrames = calculateNeededFrames(msg.dataLength, flex);// @TODO FD SUPPORT


    if (neededFrames == 0)
    {
        // invalid
        return false;
    } else if (neededFrames == 1)
    {
        msg.writeToBuffer(this->mBuffer, MAX_FRAME_BUFFER, 0);
        // single frame just send it
        IsoFragment frag = IsoFragment::makeSingleFrame(flex, msg.address, mBuffer, mFrameBuffer,
                                                 msg.dataLength);

        if (frag.getType() == IsoFragment::Type::INVALID)
            return false;

        this->mIcan->writeMessage(frag.toCanMessage(flex), ICAN::Priority::Med,
                [=](ICAN::MessageWriteResult result)->void {
            this->doNext();
        });

        return true;
    } else if (msg.dataLength <= MAX_FRAME_BUFFER){
        // multi frame no sub frames
        return false;
    } else {
        // multi frame with frames
        return false;
    }
}


