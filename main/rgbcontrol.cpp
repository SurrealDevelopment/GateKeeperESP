#include "rgbcontrol.h"

struct Pins {
    int red;
    int green;
    int blue;
    QueueHandle_t colorQueue;

};

struct Command {
        int red; // brightness 0 = max, 8192 = min
        int green;
        int blue;
        bool blink; // blink yes orn o
        int onTime; // on time
        int offTime; // off time
        // duty between 0 and 1000
        int duty;
        bool stop;
};

/**
 * RGB Control task.
 */
static void rgbTask(void * param)
{
    auto pins = (Pins *) param;

    ledc_timer_config_t ledc_timer = {};

    ledc_timer.duty_resolution =LEDC_TIMER_13_BIT;
    ledc_timer.freq_hz = 1000;
    ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_timer.timer_num = LEDC_TIMER_0;

    ledc_timer_config(&ledc_timer) ;

    ledc_channel_config_t red = {};

    red.channel = LEDC_CHANNEL_0;
    red.duty = 8192;
    red.gpio_num = pins->red;
    red.speed_mode = LEDC_HIGH_SPEED_MODE;
    red.timer_sel = LEDC_TIMER_0;

    ledc_channel_config(&red);

    ledc_channel_config_t green = {};

    green.channel = LEDC_CHANNEL_1;
    green.duty = 8192;
    green.gpio_num = pins->green;
    green.speed_mode = LEDC_HIGH_SPEED_MODE;
    green.timer_sel = LEDC_TIMER_0;


    ledc_channel_config(&green);



    ledc_channel_config_t blue = {};

    blue.channel = LEDC_CHANNEL_2;
    blue.duty = 8192;
    blue.gpio_num = pins->blue;
    blue.speed_mode = LEDC_HIGH_SPEED_MODE;
    blue.timer_sel = LEDC_TIMER_0;

    ledc_channel_config(&blue);

    // Initialize fade service.

    ledc_fade_func_install(0);

    Command curCommand = {};

    curCommand.red = 6000;
    curCommand.blue = 8192;
    curCommand.green = 8192;
    curCommand.stop = false;
    curCommand.onTime = 0;
    curCommand.offTime = 0;
    curCommand.blink = false;
    curCommand.duty = 50;

    


    while(curCommand.stop == false)
    {
        if (curCommand.blink)
        {


            // turn on
            int fadeTime = (curCommand.onTime*curCommand.duty)/1000;
            ledc_set_fade_with_time(red.speed_mode,
                    red.channel, curCommand.red, fadeTime);
            ledc_fade_start(red.speed_mode,
                red.channel, LEDC_FADE_NO_WAIT);
            ledc_set_fade_with_time(blue.speed_mode,
                    blue.channel, curCommand.blue, fadeTime);
            ledc_fade_start(blue.speed_mode,
                blue.channel, LEDC_FADE_NO_WAIT);
            ledc_set_fade_with_time(green.speed_mode,
                    green.channel, curCommand.green, fadeTime);
            ledc_fade_start(green.speed_mode,
                green.channel, LEDC_FADE_NO_WAIT);


            if (xQueueReceive(pins->colorQueue, &curCommand, curCommand.onTime/portTICK_PERIOD_MS))
                continue;



            fadeTime = (curCommand.offTime*curCommand.duty)/1000;

            // turn off
            ledc_set_fade_with_time(red.speed_mode,
                    red.channel, 8192, fadeTime);
            ledc_fade_start(red.speed_mode,
                red.channel, LEDC_FADE_NO_WAIT);
            ledc_set_fade_with_time(blue.speed_mode,
                    blue.channel, 8192, fadeTime);
            ledc_fade_start(blue.speed_mode,
                blue.channel, LEDC_FADE_NO_WAIT);
            ledc_set_fade_with_time(green.speed_mode,
                    green.channel, 8192, fadeTime);
            ledc_fade_start(green.speed_mode,
                green.channel, LEDC_FADE_NO_WAIT);

            xQueueReceive(pins->colorQueue, &curCommand, curCommand.offTime/portTICK_PERIOD_MS);

        }
        else
        {

            ledc_set_fade_with_time(red.speed_mode,
                    red.channel, curCommand.red, 100);
            ledc_fade_start(red.speed_mode,
                red.channel, LEDC_FADE_NO_WAIT);
            ledc_set_fade_with_time(blue.speed_mode,
                    blue.channel, curCommand.blue, 100);
            ledc_fade_start(blue.speed_mode,
                blue.channel, LEDC_FADE_NO_WAIT);
            ledc_set_fade_with_time(green.speed_mode,
                    green.channel, curCommand.green, 100);
            ledc_fade_start(green.speed_mode,
                green.channel, LEDC_FADE_NO_WAIT);
            xQueueReceive(pins->colorQueue, &curCommand, portMAX_DELAY);

        }
        vTaskDelay(100/portTICK_PERIOD_MS);

    }



    free(param);

    vTaskDelete(NULL);
}

/**
 * RGB Control initalise with pin numbers.
 */
RgbControl::RgbControl(int red, int green, int blue) {

    Pins * pin = new Pins();

    pin->red = red;
    pin->green = green;
    pin->blue = blue;
    


    this->colorQueue = xQueueCreate( queueSize, sizeof( Command ) );

    pin->colorQueue = this->colorQueue;

    

    xTaskCreate(rgbTask,
        "RgbTask",
        2048,
        pin,
        4,
        &xHandle);
}


/**
 * Sends stop command to task.
 */
void RgbControl::stop() {
    Command newCommand;

   
    newCommand.stop = true;

    //xQueueSend(colorQueue, &newCommand, portMAX_DELAY);

}

/**
 * Set rgb and send to control queue.
 * red,grren,blue - set duty of rgb
 * blinl - bool for blink or not
 * onTime - how long to stay on
 * offTime - how long to stay off
 * duty - time of onTime spent ramping.
 */
void RgbControl::set(int red, int green, int blue, bool blink, int onTime, int offTime, int duty) {
    Command newCommand;

    newCommand.red = red;
    newCommand.green = green;
    newCommand.blue = blue;
    newCommand.blink = blink;
    newCommand.onTime = onTime;
    newCommand.offTime = offTime;
    newCommand.stop = false;
    newCommand.duty = duty;


    xQueueSend(colorQueue, &newCommand, portMAX_DELAY);
}