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
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <freertos/semphr.h>
#include <vector>
#include <utility>
#include <map>
#include <freertos/queue.h>


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



public:
    class IsoUpdate{
    public:
        enum class Type {
            MSG_COMPLETE, // complete message received
            MSG_PARTIAL, // partial message since we cant store full frames
            MSG_FAIL, // message failed for some reason
            FINISHED // complete. terminated.
        };
        Type type = Type::MSG_FAIL;
        IsoTpMessage msg;
    };

    typedef std::function<bool(IsoUpdate)> updateFun;


    xQueueHandle messageQueue;


private:

    /**
     * Map of addresses to a map of pair <channelID, Update>
     */
    std::map <uint32_t, std::map<uint32_t , updateFun>> mListenList;

    /**
     * Map of channel id to address related to that id
     */
    std::map <uint32_t, uint32_t> mIdToAddr;

    /**
     * State of what IsoManager is doing
     */
    enum class State {
        NONE, // No or unknown state
        SENDING_MULTIFRAME,
        RECIEVING_MULTIFRAME,
    };

    ICAN * mIcan = nullptr;
    /**
     *  Buffer for aggregated ISO TP frame data
     */
    uint8_t mBuffer[MAX_FRAME_BUFFER];


    /**
     * Buffer for individual frames
     */
    uint8_t mFrameBuffer[64];


    uint32_t channelCounter = 1;

    State mCurState = State::NONE;

    struct IsoTransaction{
        uint32_t bytesRead;
    };

    void forwardMessage(IsoUpdate msg, uint32_t channelId, updateFun fun);
    /**
     * Semaphore for the iso manager
     * The iso manager needs to sync between many different tasks so this is a requirement.
     */
    QueueHandle_t semaphore;


    void doNext();

    bool processSendTpMessage(IsoTpMessage msg);



public:

    /**
     *
     */


    /**
     * Opens a standard channel for ISO TP Messages
     * @param addr - address to listen to
     * @param update - High ordered function called for updates
     * @return id. 0 is fail
     */
    uint32_t openStandardChannel(uint32_t addr, updateFun);


    /*
     *
     * @param addr - address to listen to
     * @param update - High ordered function called for updates
     * @return id. 0 is fail
     */
    uint32_t openNonStandardChannel(uint32_t addr, updateFun);


    /**
     * Queues an iso message
     * @param msg
     * @param fun
     */
    bool queueIsoMessage(IsoTpMessage msg, updateFun fun);




    void closeChannel(uint32_t channelID);



    void onCANMessager(CanMessage message) override;


    /**
     * Constructor which injects which ICAN the IsoManager will use.
     * @param CAN - the device the manager will use
     */
    explicit IsoManager(ICAN * CAN);

    /**
     * Alternative setter dependency injection
     * @param CAN - the device the manager will use
     */
    void setICAN(ICAN * CAN);
};


#endif //FDOMESP_ISOMANAGER_H