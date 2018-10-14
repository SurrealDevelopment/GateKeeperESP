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