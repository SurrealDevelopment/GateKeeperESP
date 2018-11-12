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



void IsoManager::onCANMessage(CanMessage message) {

    ESP_LOGI("TEST", "Recieved message");
    xSemaphoreTakeRecursive(this->semaphore, portMAX_DELAY);

    auto nonStandardLookup = this->mNonStandardListenList.find(message.addresss);

    if (nonStandardLookup != this->mNonStandardListenList.end())
    {
        // just send the can message directly to any non standard
        for (const auto & it: nonStandardLookup->second)
        {
            if (it.second(message))
            {
                closeChannelNoLock(it.first);
            }
        }
    }



    // standard
    auto lookup = this->mListenList.find(message.addresss);

    if (lookup == this->mListenList.end()) {
        // Ignore message
        xSemaphoreGiveRecursive(this->semaphore);
        return;
    }


    // process message
    auto frag = IsoFragment(message);

    auto type = frag.getType();

    if (type == IsoFragment::Type::INVALID)
    {
        ESP_LOGW("ISOTP", "Received INVALID");

    }
    else if (type == IsoFragment::Type::SINGLE_FRAME)
    {
        // forward all singles immediately since we don't need to do anything

        ESP_LOGI("TEST", "Received SINGLE");

        IsoTpMessage tp;
        tp.transmitAddress = frag.addr;
        tp.dataLength = frag.dataLength;
        tp.data = frag.data;

        IsoUpdate up;

        up.msg = tp;
        up.type = IsoUpdate::Type::MSG_COMPLETE;

        for (const auto& it : lookup->second.second)
        {
            forwardMessage(up, it.first, it.second);
        }

    }
    else if (type == IsoFragment::Type::FIRST_FRAME)
    {
        // begin multi frame receive if possible
        if(mCurState == State::NONE)
        {

        }
        else {

            PendingReceive pendingMsg;


            pendingMsg.first.dataLength = frag.totalDataLength;
            pendingMsg.first.transmitAddress = frag.addr;
            pendingMsg.first.receiveAddress = lookup->first; // tx,rx

            pendingMsg.first.dataLength=frag.dataLength;
            pendingMsg.first.data = pendingMsg.second; // will likely lose validity. careful.

            frag.copyData(pendingMsg.second);

            auto result = xQueueSend(this->receiveQueue, &pendingMsg, 0);

            if (result == pdTRUE)
            {
                // send WAIT TO SEND
                auto wts = IsoFragment::makeFlowControl(lookup->first, this->mFrameBuffer,
                                                        IsoFragment::FlowControlFlag::Wait, 0, 0);

                mIcan->writeMessage(wts.toCanMessage(false), ICAN::Priority::High, nullptr, nullptr);
            }
            else
            {
                // SEND OVERFLOW
                auto overflow = IsoFragment::makeFlowControl(lookup->first, this->mFrameBuffer,
                                                        IsoFragment::FlowControlFlag::OverflowAbort, 0, 0);
                mIcan->writeMessage(overflow.toCanMessage(false), ICAN::Priority::High, nullptr, nullptr);

            }




        }
    }

    xSemaphoreGiveRecursive(this->semaphore);


}

/**
 * Forward message.
 * Note this is called with a lock on the semaphore.
 * @param msg
 * @param channelId
 * @param fun - return true to terminane channel
 */
void IsoManager::forwardMessage(IsoUpdate &msg, uint32_t channelId, updateFun fun) {
    if (fun(msg))
    {
        closeChannelNoLock(channelId);
    }

}


IsoManager::IsoManager(ICAN *CAN) {
    this->mIcan = CAN;

    this->semaphore = xSemaphoreCreateRecursiveMutex();

    this->messageQueue = xQueueCreate(5, sizeof(std::pair<IsoTpMessage, updateFun>));

    this->receiveQueue = xQueueCreate(MAX_QUEUED_RECEIVE_MULTIFRAMES, sizeof(PendingReceive));


    xSemaphoreGiveRecursive(this->semaphore);
}

void IsoManager::setICAN(ICAN *CAN) {
    this->mIcan = CAN;

    free(this->messageQueue);
    free(this->semaphore);
}

uint32_t IsoManager::openNonStandardChannelNoLock(uint32_t receiveAddress, IsoManager::updateFunNonStandard fun) {
    uint32_t id = channelCounter++;

    auto channelMapLookup = mNonStandardListenList.find(receiveAddress);

    if (channelMapLookup == mNonStandardListenList.end())
    {
        // entry doesn't exist, create it
        std::map<uint32_t , updateFun> newEntry;

        // tell ican to listen to the address
        mIcan->listenToWithFallback(receiveAddress, false); //@TODO extended

        channelMapLookup = mNonStandardListenList.find(receiveAddress);
        if (channelMapLookup == mNonStandardListenList.end())
            id = 0; // OOM
    }

    if (id != 0)
    {
        if (!channelMapLookup->second.insert({id, fun}).second)
            id = 0; // oom
    }

}

uint32_t IsoManager::openNonStandardChannel(uint32_t addr, IsoManager::updateFunNonStandard fun) {

    xSemaphoreTakeRecursive(this->semaphore, portMAX_DELAY);

    uint32_t id = openNonStandardChannelNoLock(addr, fun);

    xSemaphoreGiveRecursive(this->semaphore);

    return id;

}
uint32_t
IsoManager::openStandardChannelNoLock(uint32_t transmitAddress, uint32_t receiveAddress, IsoManager::updateFun fun) {
    uint32_t id = channelCounter++;
    mIdToAddr.insert({id, receiveAddress}); // create id to receive address entry

    auto channelMapLookup = mListenList.find(receiveAddress);


    ESP_LOGI("TEST", "Point A");

    if (channelMapLookup == mListenList.end())
    {
        ESP_LOGI("TEST", "Point B");

        // entry doesn't exist, create it
        std::map<uint32_t , updateFun> a;

        auto newEntry = std::make_pair(transmitAddress, a);

        // tell ican to listen to the address
        mIcan->listenToWithFallback(receiveAddress, false); //@TODO extended

        mListenList.insert({receiveAddress, newEntry});

        channelMapLookup = mListenList.find(receiveAddress);
        if (channelMapLookup == mListenList.end())
            id = 0; // no memory
    }
    if (id != 0)
    {
        // list entry exists, just add our new channel
        // map is second entry of the second entry.
        if (!channelMapLookup->second.second.insert({id, fun}).second)
        {
            ESP_LOGI("TEST", "Point C");

            id = 0; // no memory
        }
        ESP_LOGI("TEST", "Point D");

    }

    return id;

}


uint32_t IsoManager::openStandardChannel(uint32_t transmitAddress, uint32_t receiveAddress, IsoManager::updateFun fun) {
    xSemaphoreTakeRecursive(this->semaphore, portMAX_DELAY);

    uint32_t  id = openStandardChannelNoLock(transmitAddress, receiveAddress, fun);
    xSemaphoreGiveRecursive(this->semaphore);

    return id;

}

void IsoManager::closeChannelNoLock(uint32_t channelID) {
// map channel Id to address
    auto addr = mIdToAddr.find(channelID);


    if (addr != mIdToAddr.end())
    {
        mIdToAddr.erase(channelID);
        auto channelList = mListenList.find(addr->second);

        if (channelList != mListenList.end())
        {
            auto channelMap = channelList->second.second;
            auto channelLookup = channelMap.find(channelID);
            if (channelLookup != channelMap.end())
            {
                channelMap.erase(channelLookup);

                if (channelMap.empty())
                {
                    // remove if empty
                    mListenList.erase(channelList);

                    // if there are no non standards listening to this then also stop listening
                    if (mNonStandardListenList.find(addr->second) == mNonStandardListenList.end())

                        mIcan->stopListeningTo(addr->second, false); //@TODO extended
                }
            }


        }

        return;
    }

    // else try non standard
    auto addr2 = mNonStandardIdToAddr.find(channelID);

    if (addr2 != mNonStandardIdToAddr.end())
    {
        mNonStandardIdToAddr.erase(addr2);
        auto channelList = mNonStandardListenList.find(addr2->second);
        if (channelList != mNonStandardListenList.end())
        {
            auto map = channelList->second;
            auto lookup = map.find(channelID);
            if (lookup != map.end())
            {
                map.erase(lookup);
                if (map.empty())
                {
                    // delete entry entirely
                    mNonStandardListenList.erase(channelList);
                    // if no standards are using this receive address also stop listenning
                    if (mListenList.find(addr2->second) == mListenList.end())
                        mIcan->stopListeningTo(addr2->second, false); //@TODO extended

                }
            }

        }
    }

}



void IsoManager::closeChannel(uint32_t channelID) {
    xSemaphoreTakeRecursive(this->semaphore, portMAX_DELAY);

    closeChannelNoLock(channelID);

    xSemaphoreGiveRecursive(this->semaphore);



}



bool IsoManager::queueIsoMessage(IsoTpMessage msg, IsoManager::updateFun fun) {

    auto pair = std::make_pair(msg, fun);

    auto result = xQueueSend(this->messageQueue, &pair, 5000 / portTICK_RATE_MS);

    if (result != pdTRUE)
        return false;

    doNext();

    return true;


}

void IsoManager::doNext() {
    xSemaphoreTakeRecursive(this->semaphore, portMAX_DELAY);



    if (this->mCurState == State::NONE)
    {

        std::pair<IsoTpMessage, updateFun> msg;

        auto result = xQueueReceive(this->messageQueue, &msg, 0);


        if (result == pdTRUE)
        {
            // send it

            if (!processSendTpMessage(msg))
            {
                // failed for whatever reason, notify of failure

                auto update = IsoUpdate();
                update.type = IsoUpdate::Type::MSG_FAIL;
                msg.second(update); // dont care about its return
            }
        }
    }


    xSemaphoreGiveRecursive(this->semaphore);
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

bool IsoManager::processSendTpMessage(std::pair<IsoTpMessage, updateFun> msg) {
    // assume no fd for now
    bool flex = false;


    uint32_t neededFrames = calculateNeededFrames(msg.first.dataLength, flex);// @TODO FD SUPPORT

    ESP_LOGI("TEST", "NEEDED FRAMES IS %d", neededFrames);


    if (neededFrames == 0)
    {
        // invalid
        return false;
    } else if (neededFrames == 1)
    {

        msg.first.writeToBuffer(this->mBuffer, MAX_FRAME_BUFFER, 0);
        // single frame just send it
        IsoFragment frag = IsoFragment::makeSingleFrame(flex, msg.first.transmitAddress, mBuffer, mFrameBuffer,
                                                 msg.first.dataLength);


        if (frag.getType() == IsoFragment::Type::INVALID)
            return false;



        this->mIcan->writeMessage(frag.toCanMessage(flex), ICAN::Priority::Med,
                [=](ICAN::MessageWriteResult result)->void {
                    this->doNext(); // second message is wrote queue up the next one
        });

        return true;
    } else if (msg.first.dataLength <= MAX_FRAME_BUFFER){
        // multi frame no sub frames
        ESP_LOGI("TEST", "CP1");

        msg.first.writeToBuffer(this->mBuffer, MAX_FRAME_BUFFER, 0);

        uint32_t id = 0;
        // we need to crete our own channel to listen for flow control
        id = this->openStandardChannelNoLock(msg.first.transmitAddress, msg.first.receiveAddress,
                [=](IsoUpdate it)->bool {
            // called with lock
            if (it.type != IsoUpdate::Type::MSG_COMPLETE_FLOW_CONTROL)
                return false;

            if (it.flag == IsoFragment::FlowControlFlag::ContinueToSend)
            {
                // continue with sending
                if (this->mCurState == State::SENDING_MULTIFRAME_WAITING)
                {
                    sendConsecFrame(msg.first, multiFrameCsec++, flex);
                    this->mCurState = State::SENDING_MULTIFRAME;
                }

            }
            else if (it.flag == IsoFragment::FlowControlFlag::OverflowAbort)
            {
                // Means the message simply cannot be completed as is. So fail.
                this->closeChannel(id);
                this->mCurState = State::NONE; // free up state
                alertFail(msg.second);
                return true; // stop channel
            }
            else if (it.flag == IsoFragment::FlowControlFlag::Wait)
            {
                // wait...
            }

            return false;

        });

        if (id == 0)
            return false; // OOM, fail


        this->mCurState = State::SENDING_MULTIFRAME_WAITING;

        auto firstFrame = IsoFragment::makeFirstFrame(flex, msg.first.transmitAddress, mBuffer, mFrameBuffer, msg.first.dataLength);

        auto cm = firstFrame.toCanMessage(flex);

        ESP_LOGI("TEST", "CP3");


        for (uint32_t i = 0; i < cm.dataLength; i++)
        {
            ESP_LOGW("TEST", "DATA: %02x", cm.data[i]);
        }

        // write first frame
        this->mIcan->writeMessage(cm, ICAN::Priority::Med,
          [=](ICAN::MessageWriteResult result)->void {
             if (result != ICAN::MessageWriteResult::OKAY)
             {
                 ESP_LOGI("TEST", "WROTE");

                 // Then message failed entirely
                 this->mCurState = State::NONE;
                 this->closeChannel(id);

                 //alertFail(msg.second);
             }
          });


        return true;
    } else {
        // multi frame with frames
        msg.first.writeToBuffer(this->mBuffer, MAX_FRAME_BUFFER, 0);

        //@TODO

        return false;
    }
}

void IsoManager::sendConsecFrame(IsoTpMessage msg, uint32_t cseq, bool flex) {
    IsoFragment cf;
    if (flex)
    {
        uint32_t start = (msg.dataLength+58)+(cseq*63);
        auto size = msg.dataLength - start;
        if (size > 63) size = 63;
        cf = IsoFragment::makeConsecFrame(flex, msg.transmitAddress, mBuffer, mFrameBuffer, size, cseq % 15);
    }
    else
    {
        uint32_t start = (msg.dataLength+7)+(cseq*7);
        auto size = msg.dataLength - start;
        if (size > 7) size = 7;
        cf = IsoFragment::makeConsecFrame(flex, msg.transmitAddress, mBuffer, mFrameBuffer, size, cseq % 15);
    }

    this->mIcan->writeMessage(cf.toCanMessage(flex), ICAN::Priority::Med,
        [=](ICAN::MessageWriteResult result)->void {

        }
    );
}

void IsoManager::alertFail(IsoManager::updateFun fun) {
    IsoUpdate update;
    update.type = IsoUpdate::Type::MSG_FAIL;
    fun(update);
}


