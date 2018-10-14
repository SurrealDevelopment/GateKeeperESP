#include "fdompin.h"
#include "fdom.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include <can/mcp2517fd.h>


#define PARALLEL_LINES 16

static char LOG_TAG[] = "MAIN";


RgbControl * rgb1;

GattServer * gatt;

MCP2517FD * can1;

MCP2517FD * can2;



typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

void start()
{
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


    // begin purple breathing
    rgb1->set(4000, 8192, 4000, true, 1500, 1000, 500);







}

extern "C" {
    // idf.py
    void app_main()
    {
        start();
    }
}
