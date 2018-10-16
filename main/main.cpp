/**
 * Main entry point of program.
 * Sets up devices. Handles interrupts.
 */

#include "fdompin.h"
#include "fdom.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/gpio_struct.h"
#include <can/mcp2517fd.h>
#include "esp_heap_caps.h"


#define PARALLEL_LINES 16

static char LOG_TAG[] = "MAIN";


RgbControl * rgb1;

GattServer * gatt;

MCP2517FD * can1;

MCP2517FD * can2;




// queue for gpio handling

// interrupt handler for sending to cans

static xQueueHandle gpio_evt_queue = NULL;


static void IRAM_ATTR gpio_isr_handler(void * arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);

}


static void gpio_task(void* arg)
{
    bool toggle = false;
    uint32_t io_num;
    while(1){
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            if (toggle)
            {

                toggle = false;
                rgb1->set(3000,8192,8192,false, 10, 10, 0);

            }
            else
            {
                toggle = true;
                rgb1->set(8192,8192,8192,false, 10, 10, 0);
            }

        }
    }
}





void start()
{
    auto sizeP = xPortGetFreeHeapSize();
    auto lowestP = xPortGetMinimumEverFreeHeapSize();

    Fdom * fdom = new Fdom();
     

    fdom->rgb1 = rgb1;
    fdom->gatt = gatt;

    rgb1 = new RgbControl(RGB1_RED, RGB1_GRN,RGB1_BLU);



    
    gatt = new GattServer(rgb1, "FDOM GATT");


    esp_err_t ret;
    spi_device_handle_t handle1;
    spi_device_handle_t handle2;


    spi_bus_config_t can1cfg = {};

    //memset(&can1cfg, 0, sizeof(can1cfg));


    // Bus VSPI config
    can1cfg.miso_io_num=PIN_VSPI_SDI;
    can1cfg.mosi_io_num=PIN_VSPI_SDO;
    can1cfg.sclk_io_num=PIN_VSPI_SCK;
    can1cfg.quadhd_io_num=-1;
    can1cfg.quadwp_io_num=-1;
    can1cfg.max_transfer_sz=4094;
    can1cfg.flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO |
        SPICOMMON_BUSFLAG_MOSI | SPICOMMON_BUSFLAG_DUAL;
        

    spi_device_interface_config_t can1dev = {};

    //memset(&can1dev, 0, sizeof(can1dev));


    // Can 1 device config
    can1dev.clock_speed_hz=SPI_MASTER_FREQ_10M;
    can1dev.mode=0;
    can1dev.spics_io_num=PIN_VSPI_CS0;
    can1dev.queue_size=3;
    can1dev.address_bits=12;
    can1dev.command_bits=4;
    can1dev.dummy_bits = 0;
    can1dev.cs_ena_posttrans=0;
    can1dev.cs_ena_pretrans=0;
    //can1dev.flags =SPI_DEVICE_HALFDUPLEX; // now using full duplex for DMA



    spi_device_interface_config_t can2dev = {};

//    memset(&can2dev, 0, sizeof(can2dev));


    // Can 2 device config
    can2dev.clock_speed_hz=SPI_MASTER_FREQ_10M;
    can2dev.mode=0;
    can2dev.spics_io_num=PIN_VSPI_CS1;
    can2dev.queue_size=3;
    can2dev.address_bits=12;
    can2dev.command_bits=4;
    can2dev.dummy_bits = 0;
    can2dev.cs_ena_posttrans=0;
    can2dev.cs_ena_pretrans=0;
    //can2dev.flags =SPI_DEVICE_HALFDUPLEX;  // now using full duplex for DMA



    // We will use DMA. Which means we must run in full duplex.
    // However the chip is only half duplex but this won't be a big issue (do nothing).
    ret = spi_bus_initialize(VSPI_HOST, &can1cfg, 1);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(VSPI_HOST, &can1dev, &handle1);
    ESP_ERROR_CHECK(ret);


    ret = spi_bus_add_device(VSPI_HOST, &can2dev, &handle2);
    ESP_ERROR_CHECK(ret);

    can1 = new MCP2517FD(handle1);
    can2 = new MCP2517FD(handle2);
    //can2 = new MCP2517FD(handle2);

    ESP_LOGI(LOG_TAG, "Resetting...");

    can1->reset();
    can2->reset();

    vTaskDelay(200/portTICK_RATE_MS); // delay for reset
    ESP_LOGI(LOG_TAG, "Reset...");
    ESP_LOGI(LOG_TAG, "Init pins...");

    can1->initPins();

    can2->initPins();

    ESP_LOGI(LOG_TAG, "Init FIFO...");



    can1->initFifo();
    can2->initFifo();


    can1->startCAN(500000);
    can2->startCAN(500000);



    // begin purple breathing
    rgb1->set(4000, 8192, 4000, true, 1500, 1000, 500);


    // Configure interrupt pins
    gpio_config_t io_conf = {};

    // Int pin for CAN1
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1ULL << PIN_INT1;
    io_conf.pull_down_en=GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en=GPIO_PULLUP_DISABLE;

    gpio_config(&io_conf);

    // same for int2
    io_conf.pin_bit_mask = 1ULL << PIN_INT2;

    gpio_config(&io_conf);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(&gpio_task, "gpio_task", 2048, nullptr, 10, nullptr);

    gpio_install_isr_service(0);

    gpio_isr_handler_add((gpio_num_t)PIN_INT1, &gpio_isr_handler, (void*) PIN_INT1);
    gpio_isr_handler_add((gpio_num_t)PIN_INT2, &gpio_isr_handler, (void*) PIN_INT2);


    // button test
    io_conf.mode = GPIO_MODE_INPUT;        //Input
    io_conf.pin_bit_mask = ( 1ULL << (uint64_t) FRNT_USR_BTN);    //Set pin where button is connected
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE; //Disable pullup
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    gpio_isr_handler_add((gpio_num_t)FRNT_USR_BTN, gpio_isr_handler, (void *)FRNT_USR_BTN);


    // setup can select
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << CAN_MUX_SEL_A) | (1ULL << CAN_MUX_SEL_B);
    gpio_config(&io_conf);

    gpio_set_level((gpio_num_t)CAN_MUX_SEL_A, 1);
    gpio_set_level((gpio_num_t)CAN_MUX_SEL_B, 0);


    auto size = xPortGetFreeHeapSize();
    auto lowest = xPortGetMinimumEverFreeHeapSize();


    ESP_LOGI(LOG_TAG, "Pre Init memory. Available: 0x%08x Lowest: 0x%08x",sizeP, lowestP);

    ESP_LOGI(LOG_TAG, "Post Init memory. Available: 0x%08x Lowest: 0x%08x",size, lowest);











}

extern "C" {
    // idf.py
    void app_main()
    {
        start();
    }


}
