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
#include "IsoFragment.h"
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <freertos/semphr.h>
#include <vector>
#include <utility>
#include <map>
#include <freertos/queue.h>


#define MAX_FRAME_BUFFER 512 // max number of aggregated bytes

#define MAX_QUEUED_RECEIVE_MULTIFRAMES 3

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
            MSG_COMPLETE_FLOW_CONTROL, // message complete and is a flow control

            MSG_PARTIAL, // partial message since we cant store full frames
            MSG_FAIL, // message failed for some reason

            FINISHED // complete. terminated.
        };
        Type type = Type::MSG_FAIL;
        IsoTpMessage msg;

        // extra variables only useful for flow control
        IsoFragment::FlowControlFlag flag = IsoFragment::FlowControlFlag::ContinueToSend;
        uint32_t blockSize = 0;
        uint32_t st = 0;
    };

    typedef std::function<bool(IsoUpdate)> updateFun;

    typedef std::function<bool(CanMessage)> updateFunNonStandard;



    /**
     * Type for a pending recieve. Contains a buffer to hold first frame.
     */
    typedef std::pair<IsoTpMessage, uint8_t[58]> PendingReceive;

    /**
     * Queue of messages... pair<isotpmessage, updateFun>
     */
    xQueueHandle messageQueue;

    /**
     * Message receive queue.  allows us to queue up receiving multi frame messages.
     *
     * Ie if a first frame arrives and we are currently STATE::SENDING_MULTIFRAME
     * then that first frame will be added to the receiveQueue.
     */
    xQueueHandle receiveQueue;





private:

    /**
     * Map of receive ddresses to a pair <transmitAddress, map of pair <channelID, Update>>
     */
    std::map <uint32_t, std::pair<uint32_t ,std::map<uint32_t , updateFun>>> mListenList;

    /**
     * Non standard version
     * Map of recieve addresses to a map of pair <channelID, Update>
     */
    std::map <uint32_t, std::map<uint32_t , updateFunNonStandard>> mNonStandardListenList;

    /**
     * Map of channel id to the receive address related to that id
     */
    std::map <uint32_t, uint32_t> mIdToAddr;

    /**
     * Non standard version
     * Map of channel id to the receive address related to that id
     */
    std::map <uint32_t, uint32_t> mNonStandardIdToAddr;

    /**
     * State of what IsoManager is doing
     */
    enum class State {
        NONE, // No or unknown state
        SENDING_MULTIFRAME,
        SENDING_MULTIFRAME_WAITING,
        RECIEVING_MULTIFRAME,
    };

    ICAN * mIcan = nullptr;
    /**
     *  Buffer for aggregated ISO TP frame data
     */
    uint8_t mBuffer[MAX_FRAME_BUFFER];

    uint32_t channelCounter = 1;

    State mCurState = State::NONE;

    uint32_t multiFrameCsec = 0;


    void forwardMessage(IsoUpdate &msg, uint32_t channelId, updateFun fun);
    /**
     * Semaphore for the iso manager
     * The iso manager needs to sync between many different tasks so this is a requirement.
     */
    QueueHandle_t semaphore;


    void doNext();

    bool processSendTpMessage(std::pair<IsoTpMessage, updateFun> msg);

    /**
     * Transmit consec frame of msg
     * @param msg
     */
    void sendConsecFrame(IsoTpMessage msg, uint32_t cseq, bool flex);


    /**
     * Simple method to send a MSG_FAIL
     * @param fun
     */
    void alertFail(updateFun fun);


    /**
     * Close a channel with the specified channelID. Channel IDs are shared between standard and
     * non standard. No lock variation
     * @param channelID
     */
    void closeChannelNoLock(uint32_t channelID);


    uint32_t openStandardChannelNoLock(uint32_t transmitAddress, uint32_t receiveAddress, updateFun);

    uint32_t openNonStandardChannelNoLock(uint32_t receiveAddress, updateFunNonStandard);





public:

    /**
     * Opens a standard channel for ISO TP Messages
     * @param transmitAddress - address to transmit to
     *  Note it is assumed that the transmit Address will never ever change for an entire session.
     *  Meaning a receive address is perpetually fixed to a transmit address. The reverse is not true
     *  meaning a transmit address can be used for multiple recieve addresses (important because some modules
     *  have different addresses they transmit on for various reasons).
     * @param receiveAddress - address to listen to
     * @param update - High ordered function called for updates
     *  Function will return true if it should close after consuming the message
     *  Function will return false if it should keep being sent messages
     * @return id. 0 is fail
     */
    uint32_t openStandardChannel(uint32_t transmitAddress, uint32_t receiveAddress, updateFun);


    /*
     * Non standards are direct messages. There is no transport layer involved. ISO Manager
     * has this feature because ICAN itself does not manage one to many channels very well.
     *
     * @param receiveAddress - address to listen to
     * @param update - High ordered function called for updates
     *  The data will contain the full CAN data.
     * @return id. 0 is fail
     */
    uint32_t openNonStandardChannel(uint32_t receiveAddress, updateFunNonStandard);


    /**
     * Close a channel with the specified channelID. Channel IDs are shared between standard and
     * non standard.
     * @param channelID
     */
    void closeChannel(uint32_t channelID);



    /**
     * Queues an iso message
     * @param msg pointer to message
     * @param fun
     */
    bool queueIsoMessage(IsoTpMessage msg, updateFun fun);







    void onCANMessage(CanMessage message) override;


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