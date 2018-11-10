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
#include "AbstractICAN.h"


#define INTERRUPT 1

#define QUEUE_CHANGE 2

#define QUIT 100


static void canBaseTask(void * data) {
    bool stop = false;
    auto can = (AbstractICAN *)data;
    uint32_t a;
    while (!stop)
    {
        xQueueReceive(can->taskQueue, &a, portMAX_DELAY);
        switch (a)
        {
            case INTERRUPT:
                can->onInterrupt();
                break;
            case QUEUE_CHANGE:
                can->onWriteMessageQueueChange();
                break;
            case QUIT:
                stop = true;
                break;
            default:
                break;

        }
    }
}

AbstractICAN::AbstractICAN(std::string name) {
    taskQueue = xQueueCreate(5, sizeof(uint32_t));

    medPriority = xQueueCreate(MED_QUEUE_SIZE, sizeof(PendingMessage));
    highPriority = xQueueCreate(HIGH_QUEUE_SIZE, sizeof(PendingMessage));
    xTaskCreate(&canBaseTask, ("can_task_" + name).c_str(), 2048, this, 10, nullptr);


}
AbstractICAN::~AbstractICAN() {

    free(medPriority);
    free(highPriority);
    free(taskQueue);

}


void
AbstractICAN::writeMessage(CanMessage message, ICAN::Priority priority, std::function<void(MessageWriteResult)> onFinish) {

    PendingMessage msg;

    msg.message = message;
    msg.onFinish = onFinish;


    switch(priority)
    {
        case Priority::High:
            xQueueSendToBack(this->highPriority,&msg, portMAX_DELAY);
            break;
        case Priority::Med:
            xQueueSendToBack(this->medPriority,&msg, portMAX_DELAY);
            break;
        case Priority::Low: // just send lows to med
            xQueueSendToBack(this->medPriority,&msg, portMAX_DELAY);
            break;
    }

    uint32_t onQueueChange = QUEUE_CHANGE;
    xQueueSend(this->taskQueue, &onQueueChange, portMAX_DELAY);
}
