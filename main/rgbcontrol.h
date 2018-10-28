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

#ifndef _RGBCONTROL_H_
#define _RGBCONTROL_H_


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/ledc.h"

class RgbControl {

    

    private:
        QueueHandle_t colorQueue;
        const int queueSize = 3;
        TaskHandle_t xHandle;
        

    public:
        /**
         * Creates RBG control task on pin s
         */
        RgbControl(int red, int green, int blue);
        void stop();
        void set(int red, int green, int blue, bool blink = false, int onTime = 500, int offTime = 500, int duty = 200);



};


#endif