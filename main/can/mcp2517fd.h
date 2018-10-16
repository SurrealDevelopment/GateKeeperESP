#ifndef _MCP_2517FD_H_
#define _MCP_2517FD_H_

#include "esp_system.h"
#include "driver/spi_master.h"
#include <esp_heap_caps.h>
#include <esp_log.h>
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/FreeRTOS.h"
#include "mcp2517fd_spi.h"


class MCP2517FD {

private:

    /**
     * The RegisterRecord struct stores copies of some of the registers
     * of the MCP2517FD. This is required as we may need to write registers
     * but don't want to change our settings. This issue is made further
     * since we can only read/write in 32 bit increments. (i.e. we may only
     * need to write 8 bits but we have to write 32).
     */
    class RegisterRecord{
    public:
        REG_CiCON canCon; // Can Controller
        // 4 FIFOCON
        REG_CiTXQCON fifo0con;
        REG_CiFIFOCONm fifo1con;
        REG_CiFIFOCONm fifo2con;
        REG_CiFIFOCONm fifo3con;


    };

    /**
     * Initalises register record.
     */
    void initRegisterRecord();

    const uint32_t HEAP_SIZE = 32; // heap size in bytes
    spi_device_handle_t mHandle;

    /** reads register at address non async */
    uint32_t * readRegister(uint32_t address);
    void writeRegister(uint32_t address); // write register with register buffer
    void writeRegister(uint32_t address, uint32_t data);
    void writeRegisterFromAddress(uint32_t address, void *data);


    /**
     * Change mode using can control reigster
     * @param mode id
     * @return sucess
     */
    bool changeMode(uint32_t mode);

    void stopInterrupts();
    void resumeInterrupts();


    // 2 buffer for data
    void * mReadBuffer;
    void * mWriteBuffer;

    void * mRegisterVoidPtr;

    // quick casts to uint32_ts and record
    uint32_t * mWriteReg;
    uint32_t * mReadReg;
    RegisterRecord * mRegRecord;


    // Keep a canController  register lying around


     


public:
    /**
     * Give device handle. Assumes configured properely.
     * */
    explicit MCP2517FD(spi_device_handle_t handle);


    /**
     * Interrupt call. This class doesn't handle calling interrupt on its own.
     * It must be done externally.
     */
    void interrupt();


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
     * Starts up CAN with specifiec Baud Rate
     * Normal CAN can work up to 1mbps baud rate (and go down by multiples of 2)
     *
     * @param CAN_Baud_Rate mutliple of 2s
     * @return success
     */
    bool startCAN(uint32_t CAN_Baud_Rate, bool listenOnly = false);


    /**
     * Starts up CAN with specifiec nominal and  data baud rate.
     * Note FD will only work with higher than 500k nominal rates else it will return false.
     *
     * @param nominal_CAN_Baud_Rate nominal can rate (usually lower than data)
     * @param data_CAN_BAUD_Rate higher speed data rate
     * @return success
     */
    bool startCANFD(uint32_t nominal_CAN_Baud_Rate, uint32_t data_CAN_BAUD_Rate);


    /**
     * Starts up CAN with automatic Baud Rate
     * @return success
     */
    bool startCANAutoBaud();

    /**
     * SEnd reset command
     */
    void reset();


};



#endif // _MCP_2517FD_H_