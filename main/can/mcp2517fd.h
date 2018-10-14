#ifndef _MCP_2517FD_H_
#define _MCP_2517FD_H_

#include "esp_system.h"
#include "driver/spi_master.h"
#include <esp_heap_caps.h>
#include <esp_log.h>
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/FreeRTOS.h"



class MCP2517FD {

private:

    const uint32_t HEAP_SIZE = 32; // heap size in bytes
    spi_device_handle_t mHandle;

    /** reads register at address non async */
    uint32_t * readRegister(uint32_t address);
    void writeRegister(uint32_t address); // write register with register buffer
    void writeRegister(uint32_t address, uint32_t data);


    void writeCrc(uint32_t address);

    void stopInterrupts();
    void resumeInterrupts();


    // 2 buffer for data
    void * mReadBuffer;
    void * mWriteBuffer;

    // quick casts to uint32_ts
    uint32_t * mWriteReg;
    uint32_t * mReadReg;

     


public:
    /**
     * Give device handle. Assumes configured properely.
     * */
    MCP2517FD(spi_device_handle_t handle);


    ~MCP2517FD();


    /**
     * Initalises secondary pins on device.
     * @return success.
     */
    bool initPins();


    /**
     * Initialises FIFO setup
     * @return if init suceeded true=success
     */
    bool initFifo();


    /**
     * SEnd reset command
     */
    void reset();


};



#endif // _MCP_2517FD_H_