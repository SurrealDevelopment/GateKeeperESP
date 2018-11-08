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

#ifndef _MCP_2517FD_H_
#define _MCP_2517FD_H_

#include "esp_system.h"
#include "driver/spi_master.h"
#include <esp_heap_caps.h>
#include <esp_log.h>
#include<bits/stdc++.h>
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/FreeRTOS.h"
#include "mcp2517fd_spi.h"
#include "AbstractICAN.h"

#define MED_PRI_QUEUESIZE 8
#define HIGH_PRI_QUEUESIZE 2

class MCP2517FD: public AbstractICAN {

private:

    /**
     * The RegisterRecord struct stores copies of some of the registers
     * of the MCP2517FD. This is required as we may need to write registers
     * but don't want to change our settings. This issue is made further
     * since we can only read/write in 32 bit increments. (i.e. we may only
     * need to write 8 bits but we have to write 32).
     */
    struct RegisterRecord{
    public:
        REG_CiCON canCon; // Can Controller
        // 4 FIFOCON
        REG_CiTXQCON fifo0con;
        REG_CiFIFOCONm fifo1con;
        REG_CiFIFOCONm fifo2con;
        REG_CiFIFOCONm fifo3con;
        REG_CiINT interrupt;

        REG_CiFLTCONm fltcon[32]; // 32 filter controllers (8 bytes). We keep a local copy of these.

        std::pair<uint32_t ,uint32_t> fltAddrMask[32];

        xQueueHandle medPriority;
        xQueueHandle highPriority;

        /*
         * Sequence number for transmissions
         */
        uint32_t transmitCSeq = 0;

    } mRegRecord;
private:

    const char * name;

    /**
     * Represents if device is in listen all mode
     * in which case listenTo requests should be ignored.
     */
    bool listenAllMode = false;

    uint32_t filterStackCount = 0;

    /**
     * Initalises register record.
     */
    void initRegisterRecord();

    /**
     * Heap size in bytes
     * largest is going to be one full size can fd frame.
     * which is 8 control bytes, 4 time stamp  bytes, and 64 data bytes
     */
    const uint32_t HEAP_SIZE = 80; // heap size in bytes

    spi_device_handle_t mHandle;

    /** reads register at address non async */
    uint32_t * pollingReadRegister(uint32_t address);


    /**
     * Read specified number of bytes into the buffer
     * @param address  address to read from
     * @param bytes  number of bytes to read
     */
    void pollingReadAddress(uint32_t address, uint32_t bytes);



    /**
      * Read specified number of bytes into the buffer (interrupts)
      * @param address  address to read from
      * @param bytes  number of bytes to read
      */
    void intReadAddress(uint32_t address, uint32_t bytes);

    /**
     * Write register using specified data. Will copy data to write buffer and call
     * @see writeRegister()
     *
     * @param address - address to write to
     * @param data - data to write to in uint32_t form
     */
    void pollingWriteRegister(uint32_t address, uint32_t data);
    void pollingWriteRegisterFromWriteBuffer(uint32_t address); // write register with buffer

    /**
     * Writes register from an address (rather than from buffer or etc).
     * @param address - address to write
     * @param data - pointer to data to write
     * @param queue - if
     */
    void pollingWriteRegisterFromAddress(uint32_t address, void *data);

    /**
     * Writes from an address (rather than from buffer or etc).
     * @param address - address to write
     * @param data - pointer to data to write
     * @param length length of data in bytes
     */
    void pollingWriteAddress(uint32_t address, void *data, uint32_t length);


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


    // quick casts to uint32_ts and record
    uint32_t * mWriteReg;
    uint32_t * mReadReg;

    ICANListener * listener = nullptr;

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
     * overwrite all filters with current buffer.
     * This is probably faster than writing them piece by piece
     * especially since we are regualrly rewritting all filters.
     */
    void writeAllFiltersStatus();

    /**
     * set filter status. Does not write.
     * Call writeAllFilters when done
     * @param filterNum
     * @param status
     */
    void setFilterStatus(uint32_t filterNum, bool status);



    void setFilter(uint32_t filterNum, uint32_t id, uint32_t mask, bool extended = false);

    /**
     * Writes filter mask and address
     * Filter must be disabled
     * @param filterNum filter to write
     */
    void writeFilter(uint32_t filterNum);

    // quick function to disable all filters. No write
    void disableAllFilters();

    void optimizeFilters();
    bool optimizeFiltersStep(); // recursive optimize filter step


    /**
     * Interrupt call. This class doesn't handle calling interrupt on its own.
     * It must be done externally.
     */
    void interrupt();


    /**
     * Alternative interrupt sourced from program not chip
     */
    void softInterrupt();







public:
    /**
     * Give device handle. Assumes configured properely.
     * */
    explicit MCP2517FD(spi_device_handle_t handle, const char * name);


    virtual void setCanListener(ICANListener * listener);





    ~MCP2517FD();




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
     * @param listenOnly - if CAN should start in listen only mode
     * @return success
     */
    bool startCANFD(uint32_t nominal_CAN_Baud_Rate, uint32_t data_CAN_BAUD_Rate, bool listenOnly = false);


    /**
     * General init does a broad polling initalization on the device.
     * Such as setting up the clock and other necesities.
     * @return success or not
     */
    bool generalInit();

    /**
     * SEnd reset command
     */
    void reset();


    /**
     * listen to everyhting.
     * Note this will reset filters
     */
    void listenAll();

    /**
     * Stop listening to everything
     * Note this will reset filters
     * Allows usage of listenTo and stopListeningTo
     */
    void stopListenAll();


    /**
     * Tells mcp device to start listening to specified address
     * MCP has 32 filters for listening and will automatically try to optimize the filters.
     * For example listening on 7E8..7EF will be optimized to a single filter
     * But listening to 7E8 and 7EA will be two filters
     *
     * Optimizations are delayed to absolute latest possible time. The optimization stage will be
     * similar to a context switch so avoid churning too many addresses.
     *
     * If you think you will exceed the 32 filter count often consider using listenAll in lieu of
     * listenTo.
     *
     * @param address to listen to
     * @return success
     */
    bool listenTo(uint32_t address, bool extended);


    /**
    * same as listenTo but adds a fallback in case of filter overflow
     * will switch to listenAllMode.
    *
    * @param address to listen to
    * @return success
    */
    void listenToWithFallback(uint32_t address, bool extended);


    /**
     * Tells mcp device to stop listening to specified address
     *
     * Will search for address and, if found, remove and reoptimize as needed.
     *
     * @param address to stop listening  to
     */
    void stopListeningTo(uint32_t, bool extended);

    void debugPrintFilters();

    void debugPrintRemoteFilters();

    void onWriteMessageQueueChange() override;

    void onInterrupt() override;


};



#endif // _MCP_2517FD_H_