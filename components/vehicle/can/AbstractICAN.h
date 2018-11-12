//
// Created by Justin Hoogestraat on 11/7/18.
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
#ifndef FDOMESP_ABSTRACTICAN_H
#define FDOMESP_ABSTRACTICAN_H

#include "ICAN.h"
#include "CanMessage.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <freertos/task.h>


#define MED_QUEUE_SIZE 4
#define HIGH_QUEUE_SIZE 2


class AbstractICAN : public ICAN{


protected:
    xQueueHandle medPriority;
    xQueueHandle highPriority;


public:
    class PendingMessage {
    public:
        CanMessage message;
        // message sent to bus
        std::function<void(MessageWriteResult)> onFinish = nullptr;
        // bytes sent to buffer and are free to be released
        std::function<void()> onRelease = nullptr;
    };

    xQueueHandle taskQueue;


    explicit AbstractICAN(std::string name);
    ~AbstractICAN();


    void setCanListener(ICANListener * listener) override = 0;

    void writeMessage(CanMessage * message, ICAN::Priority priority, std::function<void(MessageWriteResult)> onFinish,
                      std::function<void()> onRelease) override;

    /**
     * Called whenever something is added to one of the message queues
     * from our task
     */
    virtual void onWriteMessageQueueChange() = 0;

    /**
     * Called on some form of interrupt from our task
     */
    virtual void onInterrupt() = 0;


};


#endif //FDOMESP_ABSTRACTICAN_H
