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

#include "mcp2517fd.h"
#include <cstring>
#include "mcp2517fd_spi.h"
#include <inttypes.h>
#include "mcp2517fd.h"
#include "mcp2517fd_spi.h"


#define DEBUG_REGISTERS false
#define CLOCK_RATE_MHZ 20 // clock to mcp in MHz
#define CAN_SAMPLE_POINT 0.8 // Sample point for CAN. Were going to use 80%
#define TIMESTAMP 1 // if timestamps are being used

static char LOG_TAG[] = "CANC";



/**
 * Implementation for mcp2517fd low level spi
 *
 * Note there are some annoying things with how the esp32 handles spi.
 * Since we are using DMA we can only read/write in increments of 32 bits or 4 bytes.
 * We must also always do a full duplex transaction even though the mcp2517fd is a
 * half duplex device. For the most part is fine but not following this rule can cause
 * the SPI interface to become glitched until restart.
 */




// init everything in reg record to 0 to be safe.
void MCP2517FD::initRegisterRecord() {
    mRegRecord.canCon.word=0;
    mRegRecord.fifo0con.word=0;
    mRegRecord.fifo1con.word=0;
    mRegRecord.fifo2con.word=0;
    mRegRecord.fifo3con.word=0;


    // set all filters to target fifo3
    for (int i = 0; i < 32; i++)
    {
        mRegRecord.fltcon[i].byte = 0;
        mRegRecord.fltcon[i].b.targetFifo=3;
    }
}



MCP2517FD::MCP2517FD(spi_device_handle_t handle, const char * name): AbstractICAN(name) {

    this->name = name;
    mReadBuffer = heap_caps_malloc(HEAP_SIZE, MALLOC_CAP_DMA);
    mWriteBuffer = heap_caps_malloc(HEAP_SIZE, MALLOC_CAP_DMA);
    mReadReg = (uint32_t *)(mReadBuffer); 
    mWriteReg = (uint32_t *)(mWriteBuffer);


    // queues for MCP's data
    mRegRecord.medPriority = xQueueCreate(MED_PRI_QUEUESIZE, sizeof(PendingMessage));
    mRegRecord.highPriority = xQueueCreate(HIGH_PRI_QUEUESIZE, sizeof(PendingMessage));


    initRegisterRecord();


    
    ESP_LOGI(LOG_TAG, "Created new MCP2517FD Object");
    this->mHandle = handle;

}

MCP2517FD::~MCP2517FD()  {

    heap_caps_free(mReadBuffer);
    heap_caps_free(mWriteBuffer);

    free(mRegRecord.medPriority);
    free(mRegRecord.highPriority);


}

/**
 * Checks if a register was wrote correctly by comparing them with an &.
 * @param write copy of what was written. Do not include Read only bits
 *  and be careful with set bits.
 * @param read register that was returned
 * @return true if okay
 */
bool quickRegisterCheck(uint32_t write, uint32_t read)
{
    return ((write & read) == write);
}

void log_write_data_error(uint32_t address)
{
    ESP_LOGW(LOG_TAG, "Data write error @ address 0x%04x", address);
}

void log_read_data_error(uint32_t address)
{
    ESP_LOGW(LOG_TAG,"Data write error @ address 0x%04x", address);
}


uint32_t * MCP2517FD::pollingReadRegister(uint32_t address) {
    pollingReadAddress(address, 4); // one register is 4 bytes
    return mReadReg;
}


void MCP2517FD::pollingReadAddress(uint32_t address, uint32_t bytes) {
    spi_transaction_t t = { };

    t.cmd = CMD_READ;
    t.addr = address;
    t.length = bytes * 8;
    t.rx_buffer = mReadBuffer;
    t.tx_buffer = mWriteBuffer;

    ESP_ERROR_CHECK(spi_device_polling_transmit(mHandle, &t));

}

void MCP2517FD::intReadAddress(uint32_t address, uint32_t bytes) {
    spi_transaction_t t = { };

    t.cmd = CMD_READ;
    t.addr = address;
    t.length = bytes * 8;
    t.rx_buffer = mReadBuffer;
    t.tx_buffer = mWriteBuffer;

    ESP_ERROR_CHECK(spi_device_transmit(mHandle, &t));

}


// write 4 byte register at default write buffer
void MCP2517FD::pollingWriteRegisterFromWriteBuffer(uint32_t address) {
    return pollingWriteRegisterFromAddress(address, mWriteBuffer);

}


// overload to write register from data word
void MCP2517FD::pollingWriteRegister(uint32_t address, uint32_t data) {
    
    // set data to buffer and call other function
    *(uint32_t *)mWriteBuffer = data;
    return pollingWriteRegisterFromWriteBuffer(address);
}

// write register from pointer somewhere in a DMA buffer
void MCP2517FD::pollingWriteRegisterFromAddress(uint32_t address, void *data) {



    /* Doing this apparently breaks the write. Avoid reading before writting. */
#if DEBUG_REGISTERS
    ESP_LOGI(LOG_TAG, "Writting Registor 0x%04x with: 0x%08x", address, *(uint32_t *)data);
    //uint32_t * reg = pollingReadRegister(address);
    //ESP_LOGI(LOG_TAG, "REGISTOR 0x%04x PRE: 0x%08x", address, *reg);
#endif
    pollingWriteAddress(address, data, 4);



#if DEBUG_REGISTERS
    uint32_t * reg2 = pollingReadRegister(address);
    ESP_LOGI(LOG_TAG, "REGISTOR 0x%04x POST: 0x%08x",address, *reg2);
#endif
}

void MCP2517FD::pollingWriteAddress(uint32_t address, void *data, uint32_t length) {
    spi_transaction_t t = { };

    t.cmd = CMD_WRITE;
    t.addr = address;
    t.length = 8*length;
    t.rx_buffer = mReadBuffer;
    t.tx_buffer = data;

    ESP_ERROR_CHECK(spi_device_polling_transmit(mHandle, &t));

}







void MCP2517FD::resumeInterrupts()
{
    mRegRecord.interrupt.b.InvalidMessage=1; // Invalid MSG Interrupt
    mRegRecord.interrupt.b.BusWakeUp=1; // BUS WAKE UP for waking on low power
    mRegRecord.interrupt.b.CANBusError = 1; // CAN Bus error
    mRegRecord.interrupt.b.SystemError = 1; //
    mRegRecord.interrupt.b.ReceiveObjectOverflow = 1; // RxOverflow
    mRegRecord.interrupt.b.RxFIFO = 1; // recieve fifo
    mRegRecord.interrupt.b.TxFIFO = 1; // TX Fifo


    pollingWriteRegister(ADDR_C1INT, mRegRecord.interrupt.word);

}

void MCP2517FD::stopInterrupts()
{
    pollingWriteRegister(ADDR_C1INT, 0);
}

bool MCP2517FD::initPins() {


    // Setup oscillator
    REG_OSC regOsc = { };
    regOsc.b.PLL_ENABLE=0;
    regOsc.b.SYSTEM_CLOCK_DIVISOR = 0;
    regOsc.b.CLOCK_OUTPUT_DIVISOR = 0; // set clock output to raw (no divsion)

    pollingWriteRegister(ADDR_OSC, regOsc.word);

    // read back and verify
    auto reg2 = (REG_OSC *) pollingReadRegister(ADDR_OSC);

    // check
    if (reg2->b.PLL_ENABLE != regOsc.b.PLL_ENABLE ||
        reg2->b.SYSTEM_CLOCK_DIVISOR != regOsc.b.SYSTEM_CLOCK_DIVISOR ||
        reg2->b.CLOCK_OUTPUT_DIVISOR != regOsc.b.CLOCK_OUTPUT_DIVISOR)
    {
        log_write_data_error(ADDR_OSC);
        return false;
    }

    // setup gpio
    REG_IOCON regIOCON = {};

    regIOCON.b.XSTNDBY = 1;
    regIOCON.b.PM0 = 1; // standby and set as gpio
    regIOCON.b.LAT0 = 0; // latch to 0
    regIOCON.b.PM1 = 1;
    regIOCON.b.LAT1 = 0; // this is shut down mode
    regIOCON.b.TRIS0 = 0;
    regIOCON.b.TRIS1 = 0;


    pollingWriteRegister(ADDR_IOCON, regIOCON.word);


    auto ioconcheck = (REG_IOCON *) pollingReadRegister(ADDR_IOCON);


    // check
    if (ioconcheck->b.XSTNDBY != regIOCON.b.XSTNDBY ||
        ioconcheck->b.PM0 != regIOCON.b.PM0 ||
        ioconcheck->b.PM1 != regIOCON.b.PM1)
    {
        log_write_data_error(ADDR_IOCON);
        return false;
    }



    
    
        

    return true;

}


/**
 * Reset (send nothing basically)
 */
void MCP2517FD::reset() {
    spi_transaction_t t = { };

    // !!!! IMPORTANT !!!!
    // read/write 32 just to avoid silly DMA bugs
    // Yes its stupid
    t.rx_buffer = mReadBuffer;
    t.tx_buffer = mWriteBuffer;
    t.length = 32;
    t.cmd = 0;
    t.addr = 0;
    
    ESP_ERROR_CHECK(spi_device_polling_transmit(mHandle, &t));
}

// init fifo and other related things
bool MCP2517FD::initFifo() {

    // time stamps
    REG_CiTSCON tsCon = {};

    tsCon.b.TimeBaseCounterPrescaler = 399; // 40 MHz clock. 10 microsecond takes 400 cycles, 0=1 so 399
    tsCon.b.TimeBaseCounterEnable = 1; // enable for now, not sure if we will use

    pollingWriteRegister(ADDR_C1TSCON, tsCon.word);

    auto tsConCheck = (REG_CiTSCON *) pollingReadRegister(ADDR_C1TSCON);

    // check
    if (tsConCheck->b.TimeBaseCounterPrescaler != tsCon.b.TimeBaseCounterPrescaler ||
            tsConCheck->b.TimeBaseCounterEnable != tsCon.b.TimeBaseCounterEnable)
    {
        log_write_data_error(ADDR_C1TSCON);
        return false;
    }

    // configure default Tx FIFO
    REG_CiTXQCON * txcon = &(mRegRecord.fifo0con);

    txcon->b.PayLoadSize = 0b111; // 64 bytes
    txcon->b.FifoSize = MED_PRI_QUEUESIZE; // 4 messages (256 bytes)
    txcon->b.TxRetransmissionAttempts = 0b01; // or 3 attempts
    txcon->b.TxPriority = 0b00111; // midish priority
    txcon->b.TxEmptyInterruptEn = 0;

    pollingWriteRegister(ADDR_C1TXQCON, txcon->word);

    // verify
    auto txconcheck = (REG_CiTXQCON *) pollingReadRegister(ADDR_C1TXQCON);


    // check
    if (txconcheck->b.PayLoadSize != txcon->b.PayLoadSize ||
        txconcheck->b.FifoSize != txcon->b.FifoSize ||
        txconcheck->b.TxRetransmissionAttempts != txcon->b.TxRetransmissionAttempts ||
        txconcheck->b.TxPriority != txcon->b.TxPriority)
    {
        log_write_data_error(ADDR_C1TXQCON);
        return false;
    }

    // configure priority 1 transmit FIFO
    REG_CiFIFOCONm * fifoCon = &(mRegRecord.fifo1con);

    fifoCon->b.TxRxSel=1;
    fifoCon->b.FIFOPayload = 0b111; // 64 bytes
    fifoCon->b.FIFOSize = HIGH_PRI_QUEUESIZE; // 2 messages (128 bytes)
    fifoCon->b.TxRetransmissionAttempt = 0b01; // or 3 attempts
    fifoCon->b.TxTransmitPriority = 0b11111; // top priority

    pollingWriteRegister(ADDR_C1FIFOCON1, fifoCon->word);

    // verify
    auto fifoConCheck = (REG_CiFIFOCONm *) pollingReadRegister(ADDR_C1FIFOCON1);


    // check
    if (fifoConCheck->b.FIFOPayload != fifoCon->b.FIFOPayload ||
        fifoConCheck->b.FIFOSize != fifoCon->b.FIFOSize ||
        fifoConCheck->b.TxRetransmissionAttempt != fifoCon->b.TxRetransmissionAttempt ||
        fifoConCheck->b.TxTransmitPriority != fifoCon->b.TxTransmitPriority ||
        fifoConCheck->b.TxRxSel != fifoCon->b.TxRxSel)
    {
        log_write_data_error(ADDR_C1FIFOCON1);
        return false;
    }



    // Recieve FIFO3
    fifoCon = &(mRegRecord.fifo3con);

    fifoCon->word = 0;

    fifoCon->b.TxRxSel=0; // recieve
    fifoCon->b.FIFOPayload = 0b111; // 64 bytes
    fifoCon->b.FIFOSize = 16; // 16 messages (1024 bytes, 1kB)
    //fifoC->n.b.TxRetransmissionAttempt = 2; // or 3 attempts
    //fifoC->n.b.TxTransmitPriority = 0b11111; // top priority
    fifoCon->b.FIFOEmptyInterruptEn = 1; // enable interrupt if FIFO is full
    fifoCon->b.FIFONotFullInterruptEn = 1; // interrupt if something is in the FIFO
    fifoCon->b.ReceiveMessageTimeStamps = TIMESTAMP; // enable time stamps on messages


    pollingWriteRegister((ADDR_C1FIFOCON3), fifoCon->word);

    fifoConCheck = (REG_CiFIFOCONm *) pollingReadRegister(ADDR_C1FIFOCON3);


    // check
    if (fifoConCheck->b.FIFOPayload != fifoCon->b.FIFOPayload ||
        fifoConCheck->b.FIFOSize != fifoCon->b.FIFOSize ||
        fifoConCheck->b.FIFOEmptyInterruptEn != fifoCon->b.FIFOEmptyInterruptEn ||
        fifoConCheck->b.FIFONotFullInterruptEn != fifoCon->b.FIFONotFullInterruptEn ||
        fifoConCheck->b.TxRxSel != fifoCon->b.TxRxSel ||
        fifoConCheck->b.ReceiveMessageTimeStamps != fifoCon->b.ReceiveMessageTimeStamps)
    {
        log_write_data_error(ADDR_C1FIFOCON3);
        return false;
    }


    // if we got here then it succeeded
    return true;
}

bool MCP2517FD::startCAN(uint32_t CAN_Baud_Rate, bool listenOnly) {

    // Insanely unlikely anything above 1mbit is going to work outside of
    // CAN-FD so return false
    if (CAN_Baud_Rate > 1000000)
    {
        ESP_LOGV(LOG_TAG, "CAN Baud Rate Too High!");
        return false;
    }

    // stop interrupts for now just to stay on the safe side
    stopInterrupts();


    // Reference to CAN Control Register
    auto con = &(mRegRecord.canCon);

    con->b.DNetFilterCount = 0; // basically using data bytes for id. We're not doing that
    con->b.IsoCrcEnable = 0; // This is a rule to patch issues with CAN CRC. Unlikely old CAN uses this so no
    con->b.ProtocolExceptionEventDisable = 0; // Nah
    con->b.WakeUpFilterEnable = 1; // need wake up for low power use
    con->b.WakeUpFilterTime = 2; // min time for can wakeup
    con->b.BitRateSwitchDisable = 1; // No FD
    con->b.RestrictReTxAttempts = 1;
    con->b.EsiInGatewayMode = 0;
    con->b.SystemErrorToListenOnly = 1;
    con->b.StoreInTEF = 0;
    con->b.TXQEnable = 1; // basically FIFO0 which we are using
    con->b.TxBandWidthSharing = 0; // This adds a delay at end of transmisisons. Which we don't want.
    con->b.RequestOpMode = CAN_CONFIGURATION_MODE; // make sure its in config mode (it probably already is)

    // write controller
    pollingWriteRegister(ADDR_C1CON, mRegRecord.canCon.word);

    con->b.RequestOpMode = 0;
    mRegRecord.canCon.b.OpMode=0;


    // check
    if (!quickRegisterCheck(con->word, *pollingReadRegister(ADDR_C1CON)))
    {
        log_write_data_error(ADDR_C1CON);
        return false;
    }


    // This register deals with bit timing
    // see each segment here
    // https://en.wikipedia.org/wiki/CAN_bus


    REG_CiNBTCFG nominalBitTimeConfig = {};

    /**
     * Most of the calculation is going to have the idea of a sample point.
     * ARINC sets the prefered sample point to 75%.
     * But many controllers use 87.5% as well.
     * Anything from 50% to 90% is generally okay
     */

    nominalBitTimeConfig.b.SyncJumpWidth = 1; // keep this 1 for now

    // i.e 40 MHZ with .75 will be 29
    // 1 is the SJW
    // we will have 1 TQ for each MHz
    auto a = (uint32_t)(CLOCK_RATE_MHZ*CAN_SAMPLE_POINT)-1;

    // i.e. 40-29-1 is 10
    auto b = CLOCK_RATE_MHZ - a - 1;

    // Do some sanity checks
    if (a < 1 || a > 255 || b < 1 || b > 127) {
        ESP_LOGV(LOG_TAG, "CAN Bit Nominal Timing Invalid Time Segments");
        return false;
    }

    nominalBitTimeConfig.b.TimeSegment1 = a - 1; // this is prop + Phase segment 1. minus 1 due to calc
    nominalBitTimeConfig.b.TimeSegment2 = b - 1; // Phase segment 2 only

    // from calculating Time Segments with SYS CLOCK we dont need to calculate that here
    // it will work in multiples of 2 so 1000000 would be 0
    // This controls how long each Time quanta is.
    nominalBitTimeConfig.b.BaudRatePreScaler = (1000000ul / CAN_Baud_Rate) - 1;


    // dont worry about data register
    // write bit timing
    pollingWriteRegister(ADDR_C1NBTCFG, nominalBitTimeConfig.word);

    // check
    if (!quickRegisterCheck(nominalBitTimeConfig.word, *pollingReadRegister(ADDR_C1NBTCFG)))
    {
        log_write_data_error(ADDR_C1NBTCFG);
        return false;
    }


    // Set mode
    if (listenOnly) {
        if (!changeMode(CAN_LISTEN_ONLY_MODE))
            return false;
    }
    else {
        if (!changeMode(CAN_CLASSIC_MODE))
            return false;
    }

    resumeInterrupts();




    // Good!
    return true;
}

bool MCP2517FD::startCANFD(uint32_t nominal_CAN_Baud_Rate, uint32_t data_CAN_BAUD_Rate, bool listenOnly) {



    // Check options
    if (nominal_CAN_Baud_Rate < 500000 || data_CAN_BAUD_Rate < 1000000)
    {
        ESP_LOGV(LOG_TAG, "Invalid CAN FD Baud rates");
        return false;
    }

    // stop interrupts for now
    stopInterrupts();



    // Can control register
    auto canCon = &(mRegRecord.canCon);

    // reset
    canCon->word = 0;

    canCon->b.DNetFilterCount = 0; // dont use
    canCon->b.IsoCrcEnable = 1; // Use CRC for can-fd
    canCon->b.ProtocolExceptionEventDisable = 0; // Nah
    canCon->b.WakeUpFilterEnable = 1; // need wake up for low power use
    canCon->b.WakeUpFilterTime = 2;
    canCon->b.BitRateSwitchDisable = 0; // We are using Fd so set to 0
    canCon->b.RestrictReTxAttempts = 1;
    canCon->b.EsiInGatewayMode = 0;
    canCon->b.SystemErrorToListenOnly = 1;
    canCon->b.StoreInTEF = 0;
    canCon->b.TXQEnable = 1; // basically FIFO0 which we are using
    canCon->b.TxBandWidthSharing = 0; // This adds a delay at end of transmisisons. Which we don't want.
    canCon->b.RequestOpMode = CAN_CONFIGURATION_MODE; // make sure its in config mode (it probably already is)

    if (!changeMode(CAN_CONFIGURATION_MODE))
        return false;

    // This register deals with bit timing
    // see each segment here
    // https://en.wikipedia.org/wiki/CAN_bus
    REG_CiNBTCFG nominalBitTimeConfig = {};

    /**
     * Most of the calculation is going to have the idea of a sample point.
     * ARINC sets the prefered sample point to 75%.
     * But many controllers use 87.5% as well.
     * Anything from 50% to 90% is generally okay
     */

    nominalBitTimeConfig.b.SyncJumpWidth = 1; // still keep this 1 for now


    // This is time quantas we can get away with if we assume 0 pre scaler
    // i.e. 500k would be 50 TQs
    auto TQs = (1000000 * CLOCK_RATE_MHZ) / nominal_CAN_Baud_Rate;

    // i.e 40 MHZ with .75 will be 29
    // 1 is the SJW
    // we will have 1 TQ for each MHz
    auto a = (uint32_t)(TQs*CAN_SAMPLE_POINT)-1;

    // i.e. 40-29-1 is 10
    auto b = TQs - a - 1;

    // Do some sanity checks
    if (a < 1 || a > 255 || b < 1 || b > 127) {
        ESP_LOGV(LOG_TAG, "CAN Bit Nominal Timing Invalid Time Segments");
        return false;
    }

    nominalBitTimeConfig.b.TimeSegment1 = a - 1; // this is prop+ Phase segment 1, minus 1 due to calc
    nominalBitTimeConfig.b.TimeSegment2 = b - 1; // Phase segment 2 only

    // We want to keep things as accuruate as possible so assume BPS is 0
    nominalBitTimeConfig.b.BaudRatePreScaler = 0;


    // Data bit timing
    REG_CiDBTCFG dataBitTimingConfig = {};

    dataBitTimingConfig.b.SyncJumpWidth = 1; // Again just keep SJW as 1 Qt

    // similiar calc to previous just using data speed
    TQs = (1000000ul * CLOCK_RATE_MHZ) / data_CAN_BAUD_Rate;
    a = (uint32_t)(TQs*CAN_SAMPLE_POINT)-1;
    b = TQs - a - 1;

    if (a < 1 || a > 255 || b < 1 || b > 127) {
        ESP_LOGV(LOG_TAG, "CAN Bit Data Timing Invalid Time Segments");
        return false;
    }

    dataBitTimingConfig.b.TimeSegment1=a-1;
    dataBitTimingConfig.b.TimeSegment2=b-a;
    dataBitTimingConfig.b.BaudRatePreScaler=0;



    // write bit nominal timing
    pollingWriteRegister(ADDR_C1NBTCFG, nominalBitTimeConfig.word);

    // check
    if (!quickRegisterCheck(nominalBitTimeConfig.word, *pollingReadRegister(ADDR_C1NBTCFG)))
    {
        log_write_data_error(ADDR_C1NBTCFG);
        return false;
    }

    // write bit data timing
    pollingWriteRegister(ADDR_C1DBTCFG, dataBitTimingConfig.word);

    // check
    if (!quickRegisterCheck(dataBitTimingConfig.word, *pollingReadRegister(ADDR_C1DBTCFG)))
    {
        log_write_data_error(ADDR_C1DBTCFG);
        return false;
    }


    // Set mode as final step
    if (listenOnly) {
        if (!changeMode(CAN_LISTEN_ONLY_MODE))
            return false;
    }
    else {
        if (!changeMode(CAN_NORMAL_MODE))
            return false;
    }

    resumeInterrupts();


    return false;
}



bool MCP2517FD::changeMode(uint32_t mode) {
    mRegRecord.canCon.b.RequestOpMode = mode;

    // write controller
    pollingWriteRegister(ADDR_C1CON, mRegRecord.canCon.word);

    mRegRecord.canCon.b.RequestOpMode = 0;
    mRegRecord.canCon.b.OpMode=mode;

    // give it time
    vTaskDelay(50/portTICK_RATE_MS);

    auto check = pollingReadRegister(ADDR_C1CON);

    // check
    if (!quickRegisterCheck((mRegRecord.canCon.word), *check)
        || ((REG_CiCON *)check)->b.OpMode != mode)
    {
        ESP_LOGV(LOG_TAG, "Failed to change CAN Mode.");
        return false;
    }
    return true;

}


/**
 * Converts dlc to real message length
 *
 * @param dlc 0..15
 * @param fd indicate if its a can fd frame
 * @return length in bytes
 */
uint32_t dlcToLength(uint32_t dlc, bool fd)
{
    if (!fd)
    {
        if (dlc <= 8)
            return dlc;
        else
            return 8;
    }
    else
    {
        if (dlc <= 8)
            return dlc;
        else if (dlc <= 11)
            return 8+(4*(dlc - 8u)); // all lengths after 8 mul by 4
        else if (dlc <= 13)
            return (20+(6*(dlc-11u)));
        else if (dlc <= 15)
            return (32+(16*(dlc-13u)));
        else
            return 64;

    }
}

/**
 * Converts a message length to dlc
 * Rounds up
 *
 * @param length 0..64
 * @param fd indicate if its a can fd frame
 * @return length in bytes
 */
uint32_t lengthToDlc(uint32_t length, bool fd)
{
    if (!fd)
    {
        if (length <= 8)
            return length;
        else return 8;
    }
    else
    {
        if (length <= 8)
            return length;
        else if (length <= 20)
        {
            auto spare = length - 8;
            auto mod = spare % 4;
            auto div = spare / 4;
            if (mod != 0) div++;
            return (div+8);

        }
        else if (length <= 32)
        {
            auto spare = length - 20;
            auto mod = spare % 6;
            auto div = spare / 6;
            if (mod != 0) div++;
            return (div+11);
        }
        else if (length <= 64)
        {
            auto spare = length - 32;
            auto mod = spare % 16;
            auto div = spare / 16;
            if (mod != 0) div++;
            return (div+13);
        }
        else
            return  15;


    }
}


/**
 * Interrupt logic. Interrupts are going to use their own transactions
 */
void MCP2517FD::interrupt() {

    // get exclusive access on bus for quicker speeds
    spi_device_acquire_bus(mHandle, portMAX_DELAY);

    const auto reg = (REG_CiINT *) pollingReadRegister(ADDR_C1INT);


    const auto iReg = reg->word;




    if (iReg & 1) // Tx FIFO
    {
        /*
         * It may be worth it here to see the interrupt code register
         * but since we only have 2 tx fifos we can just check both
         */
        softInterrupt();
    }
    if (iReg & 2) // Rx FIFO
    {
        // read all
        auto statusReg = *pollingReadRegister(ADDR_C1FIFOSTA3);

        // if first bit is 1 there is content in FIFO
        while (statusReg & 1)
        {
            // we only have one receive fifo, fifo3, so just read from that
            // first step is get current TAIL of the FIFO using user address register

            // rxTail needs to be offset because its not the absolute address
            auto rxTail = *pollingReadRegister(ADDR_C1FIFOUA3); // no reg cast needed. This is the address

            rxTail = rxTail + 0x400;

            // read whole fifo message object since its probably faster than constant switching
#if TIMESTAMP==0
            pollingReadAddress(rxTail, 72); // 64 data bytes + 8 initial
#else
            pollingReadAddress(rxTail, 76); // 64 data bytes + 8 initial + 4 timestamp
#endif
            //auto regs = (uint32_t *)mReadBuffer;

            auto data32 = (uint32_t *)mReadBuffer;

            auto timestamp = data32[2];
            auto test = data32[3];




            uint32_t id = data32[0] & 0x1FFFFFFF;

            if (id == 0x0c9)
            {
                // because reverse endianess
                uint32_t rpm = (data32[3] & 0x00FF0000) >> 16;
                rpm = rpm | (data32[3] & 0x0000FF00);
                auto rpmf = (rpm)/4.0;
                ESP_LOGI(LOG_TAG, "Engine RPM from 0x%04x is %f, %s",id, rpmf, name);

            }


            //ESP_LOGI(LOG_TAG, "%s RX from 0x%04x 0x%04x%04x",name,id, data32[3], data32[4]);



            //ESP_LOGI(LOG_TAG, "New CAN Message AT FIFO 0x%04x Status: 0x%08x", rxTail, statusReg);
            //ESP_LOGI(LOG_TAG, "Byte 2: 0x%08x",(bytes[1]));
            //ESP_LOGI(LOG_TAG, "Byte 3: 0x%08x",(bytes[2]));
            //ESP_LOGI(LOG_TAG, "Byte 4: 0x%08x",(bytes[3]));

            // set uinc of FIFOCON to increment the FIFO because we read the message
            mRegRecord.fifo3con.b.IncrementHeadTail = 1;
            pollingWriteRegister(ADDR_C1FIFOCON3, mRegRecord.fifo3con.word);
            mRegRecord.fifo3con.b.IncrementHeadTail = 0;



            // get status again
            statusReg = *pollingReadRegister(ADDR_C1FIFOSTA3);

        }

    }
    if (iReg & 8) // op mode change
    {
        // read change
        pollingReadRegister(ADDR_C1CON);

        auto mode = ((REG_CiCON *)mReadBuffer)->b.OpMode;

        if (mode != mRegRecord.canCon.b.OpMode)
            ESP_LOGW(LOG_TAG, "Unknown Opmode change to %d", mode);


    }
    if (iReg & 2048) // Rx FIFO overflow
    {
        ESP_LOGV(LOG_TAG, "FIFO OVERFLOW");
    }
    if (iReg & 4096) // system error
    {
        ESP_LOGV(LOG_TAG, "SYSTEM ERROR");

    }
    if (iReg & 8192) // CAN error
    {
        ESP_LOGW(LOG_TAG, "CAN ERROR");

    }
    if (iReg & 0x4000) // WAKE UP
    {
        ESP_LOGI(LOG_TAG, "HIGH VOLTAGE WAKE UP");

    }
    if (iReg & 0x8000) // invalid msg
    {
        ESP_LOGW(LOG_TAG, "INVALID MESSAGE");
    }

    // write back INT to clear the interrupt
    pollingWriteRegister(ADDR_C1INT, mRegRecord.interrupt.word);

}

void MCP2517FD::softInterrupt() {
    // check queues


    //med pri
    auto statusMed = (REG_CiFIFOSTAm *) pollingReadRegister(ADDR_C1TXQSTA); // med pri
    auto statusHigh = (REG_CiFIFOSTAm *) pollingReadRegister(ADDR_C1FIFOSTA1); // high pri

    PendingMessage msg;

    while (statusMed->b.FIFONotFullEmptyInterruptFlag == 1) // while FIFO not full
    {
        auto pull = xQueueReceive(this->medPriority, &msg, 0);

        if (pull != pdTRUE)
            break; // no messages to send
        // else msg is valid

        // dont send if not in correct mode
        if ( !(this->mRegRecord.canCon.b.OpMode == CAN_NORMAL_MODE
            || this->mRegRecord.canCon.b.OpMode == CAN_CLASSIC_MODE))
        {
            // refuse
            if (msg.onRelease != nullptr) msg.onRelease();
            if (msg.onFinish != nullptr) msg.onFinish(MessageWriteResult::FAIL);
            continue;
        }

        // dont send fd frames if not in fd mode
        if (this->mRegRecord.canCon.b.OpMode != CAN_NORMAL_MODE &&
            msg.message.flexibleDataRate)
        {
            if (msg.onRelease != nullptr) msg.onRelease();
            if (msg.onFinish != nullptr) msg.onFinish(MessageWriteResult::FAIL);
            continue;
        }

        auto addr = *pollingReadRegister(ADDR_C1TXQUA); // med pri
        addr = addr + 0x400; // offset

        auto buf = (TRANSMIT_MESSAGE_OBJECT *)mWriteBuffer;
        auto dlc = lengthToDlc(msg.message.dataLength, msg.message.flexibleDataRate);
        auto adjustLength = dlcToLength(dlc, msg.message.flexibleDataRate);

        buf->prim.word1=0;
        buf->prim.word2=0;
        buf->control.standardIdentifier=msg.message.addresss;
        buf->control.ExtesnionFlag = (uint32_t )msg.message.extended;
        buf->control.FDF = (uint32_t )msg.message.flexibleDataRate;
        buf->control.BitRateSwitch = (uint32_t )msg.message.flexibleDataRate;
        buf->control.DLC=
        buf->control.Sequence = this->mRegRecord.transmitCSeq++;

        // copy data
        for (uint32_t i = 0; i<adjustLength; i++)
        {
            if (i >= msg.message.dataLength)
            {
                // add 0xAA as padding
                buf->prim.data[i] = 0xAA;
            } else {
                buf->prim.data[i] = msg.message.data[i];
            }
        }

        // notify it can release its memory
        msg.onRelease();

        pollingWriteAddress(addr, mWriteBuffer, 64); // 64 bytes + 8 control

        // set uinc of FIFOCON to increment the FIFO because we transmit the message
        mRegRecord.fifo0con.b.IncrementHeadTail = 1;
        mRegRecord.fifo0con.b.TxRequest = 1;

        pollingWriteRegister(ADDR_C1TXQCON, mRegRecord.fifo0con.word);
        mRegRecord.fifo0con.b.IncrementHeadTail = 0;
        mRegRecord.fifo0con.b.TxRequest = 0;


        statusMed = (REG_CiFIFOSTAm *) pollingReadRegister(ADDR_C1TXQSTA); // med pri

        //@Todo set on write to only occur after we recieve a transmit event

        msg.onFinish(MessageWriteResult::OKAY);

    }

}


bool MCP2517FD::generalInit() {

    initRegisterRecord(); // make sure register records are reset

    bool failFlag = true;
    // get exclusive access on bus for quicker speeds
    spi_device_acquire_bus(mHandle, portMAX_DELAY);

    reset();
    vTaskDelay(200/portTICK_RATE_MS); // give it 200 ms of time for reset.

    if (!initPins()) failFlag = false;
    // continue anyway even if we fail just in case
    if (!initFifo()) failFlag = false;


    // write filter status to insure all are disabled
    writeAllFiltersStatus();
    writeAllFiltersStatus();


    filterStackCount = 0;

    // release bus
    spi_device_release_bus(mHandle);

    return failFlag;
}



void MCP2517FD::listenAll() {
    disableAllFilters();
    writeAllFiltersStatus();
    setFilter(0, 0x0, 0x0, false);
    writeFilter(0);
    setFilterStatus(0, true);
    writeAllFiltersStatus();

    listenAllMode = true;
}


void MCP2517FD::stopListenAll() {
    disableAllFilters();
    writeAllFiltersStatus();
    listenAllMode = false;

}


void MCP2517FD::writeAllFiltersStatus() {
    // overwrite all filters with current.
    // each filter is 1 byte so write 32 bytes since there are 32

    auto flt =  (mRegRecord.fltcon);
    auto buffer = (uint8_t *) mWriteBuffer;
    // put filters in mWritebuf
    for (int i = 0; i < 32; i++)
    {
        buffer[i] = flt[i].byte;
    }
    pollingWriteAddress(ADDR_C1FLTCON0, mWriteBuffer, 32);
}

void MCP2517FD::disableAllFilters() {
    for (auto && item : mRegRecord.fltcon)
    {
        item.b.en=0;
    }
}



// sets mask and id filter of filter num
// Does not enable filter
void MCP2517FD::setFilter(uint32_t filterNum, uint32_t id, uint32_t mask, bool extended) {
    if (filterNum > 31) return; // sanity

    if (extended) id |= 1 << 30; // set extended bit (only will accept extended)

    mask |= 1 << 30; // insures mask follows extended bit

    mRegRecord.fltAddrMask[filterNum].first = id;
    mRegRecord.fltAddrMask[filterNum].second = mask;

}

void MCP2517FD::writeFilter(uint32_t filterNum) {
    if (filterNum > 31) return; // sanity

    uint32_t id = mRegRecord.fltAddrMask[filterNum].first;
    uint32_t mask = mRegRecord.fltAddrMask[filterNum].second;

    pollingWriteRegister(ADDR_C1FLTOBJ0 + (FLT_OBJMASK_SPACING * filterNum), id);

    pollingWriteRegister(ADDR_C1MASK0 + (FLT_OBJMASK_SPACING * filterNum), mask);

}




void MCP2517FD::setFilterStatus(uint32_t filterNum, bool status) {
    if (filterNum > 31) return; // sanity

    if (status)
        mRegRecord.fltcon[filterNum].b.en=1;
    else
        mRegRecord.fltcon[filterNum].b.en=0;

}

/**
 * Function for optimizing filters
 * Note this will take some time to do and we can miss messages here
 */
void MCP2517FD::optimizeFilters()
{
    // set disabled filter addrs to max
    for (int i = 0; i < 32; i++)
    {
        const auto flt = mRegRecord.fltcon[i];

        if (!flt.b.en)
        {
            (mRegRecord.fltAddrMask[i].second) = UINT32_MAX;
        }
    }
    while (optimizeFiltersStep()); // repeat step as long as true


    // now that we optimized our lists lets rewrite everything
    // disable all filters
    disableAllFilters();
    writeAllFiltersStatus();
    // rewrite all filters
    for (uint32_t i = 0; i < filterStackCount; i++)
    {
        setFilter(i, mRegRecord.fltAddrMask[i].first, mRegRecord.fltAddrMask[i].second, false); // @TODO extended addressing
        setFilterStatus(i, true);
        writeFilter(i);
    }

    writeAllFiltersStatus();



}


bool MCP2517FD::optimizeFiltersStep() {
    // sort
    std::sort(std::begin(mRegRecord.fltAddrMask), std::end(mRegRecord.fltAddrMask));

    // count size
    for (filterStackCount = 0; filterStackCount < 32; filterStackCount++)
    {
        if (mRegRecord.fltAddrMask[filterStackCount].first == UINT32_MAX)
            break;
    }

    bool change = false;
    // merge
    for (int i = 0; i < filterStackCount -1; i++)
    {
        auto fltA = &(mRegRecord.fltAddrMask[i]);
        auto fltB = &(mRegRecord.fltAddrMask[i+1]);

        if ((fltA->second & 0x1FFFFFFF) == (fltB->second & 0x1FFFFFFF)) // If masks are same they are contenders for a merger
        {
            auto newMask = (fltA->second << 1) & 0x1FFFFFFF;
            // works just like listenTo
            uint32_t calc = fltB->first & newMask;

            if (calc == fltA->first) // ok for merger
            {
                // make new filter the second one so that it is available for next cycle
                fltB->first = fltA->first;
                fltB->second =newMask;
                fltA->first=UINT32_MAX;
                change = true;
            }

        }
    }
    return change;


}

bool MCP2517FD::listenTo(uint32_t address, bool extended) {

    if (listenAllMode) return true;

    // keep track of last filter we can write into
    uint32_t availableFilter = filterStackCount;

    // check if in filters already
    for (uint32_t i = 0; i < filterStackCount; i++)
    {
        const auto flt= mRegRecord.fltcon[i];
        uint32_t mask= (mRegRecord.fltAddrMask[i].second) & 0x1FFFFFFF; // and since there is extra stuff in register
        uint32_t addr= (mRegRecord.fltAddrMask[i].first) & 0x1FFFFFFF;
        uint32_t calc = address & mask;  // if address in filter calc will == addr


        if (flt.b.en && calc == addr)
        {
            return true; // do nothing just return
        } else if (!flt.b.en && availableFilter == filterStackCount)
        {
            availableFilter = i;
        }
    }

    // check if filter exists
    if (availableFilter > 31)
    {
        // try optimizing filters to get it under 31
        //optimizeFilters();
        availableFilter = filterStackCount;
        if (filterStackCount > 31)
        {
            return false;
        }
    }

    // add filter
    setFilter(availableFilter, address, 0x1FFFFFFF, false); // @TODO accept extended addressing
    setFilterStatus(availableFilter, true);
    writeFilter(availableFilter);
    writeAllFiltersStatus();
    if (availableFilter == filterStackCount)
        filterStackCount++;



    return true;
}

void MCP2517FD::listenToWithFallback(uint32_t address, bool extended) {

    if (!listenTo(address, extended))
    {
        ESP_LOGW(LOG_TAG, "%s Filter Overflow. Switching to fallback mode.", name);
        listenAll();
    }
}



void MCP2517FD::stopListeningTo(uint32_t address, bool extended) {
    // stop listening is going to be a lot harder than simply listening
    // reason being we need to deal with splitting filters up.

    if (listenAllMode) return; // return if in listen all mode

    // check if in filters already
    for (uint32_t i = 0; i < filterStackCount; i++)
    {
        const auto flt= (mRegRecord.fltcon[i]);
        uint32_t  mask= (mRegRecord.fltAddrMask[i].second);
        uint32_t  addr= (mRegRecord.fltAddrMask[i].first);
        uint32_t calc = address & mask;  // if address in filter calc will == addr


        if (flt.b.en && calc == addr)
        {
            uint32_t range = (mask ^ 0x1FFFFFFF) + 1; // xor will be range
            // filter found
            // disable filter
            setFilterStatus(i, false);
            writeAllFiltersStatus();

            // Its common for the removed filter to be the last filter
            // so we can save some optimization steps by subtracting the stack counter
            if (i == filterStackCount-1)
                filterStackCount--;

            for (uint32_t b = addr; b < addr + range; b++)
            {
                // If is target filter skip else add filter
                if (b == address)
                    continue;

                listenTo(b, extended);
            }

            break; // should only be present once so just break

        }
    }

    // else it doesnt exist just return
    return;

}



void MCP2517FD::setCanListener(ICANListener * listener) {
    this->listener = listener;
}

void MCP2517FD::debugPrintFilters() {
    ESP_LOGI(LOG_TAG, "%s Local Filters", name);
    for (uint32_t i = 0; i < 32; i++)
    {

        ESP_LOGI(LOG_TAG, "Filter RECORD is 0x%02x ADDR: %04x MASK %04x",
                 mRegRecord.fltcon[i].byte,
                 mRegRecord.fltAddrMask[i].first,
                 mRegRecord.fltAddrMask[i].second & 0x1FFFFFFF);


    }
    ESP_LOGI(LOG_TAG, "----------End Round------------");

}


void MCP2517FD::debugPrintRemoteFilters() {
    ESP_LOGI(LOG_TAG, "%s Remote Filters", name);
    for (uint32_t i = 0; i < 32/4; i++)
    {

        auto thing = *pollingReadRegister(ADDR_C1FLTCON0+i*4);

        for (uint32_t k = 0; k < 4; k++)
        {
            auto mask = *pollingReadRegister(ADDR_C1MASK0+ (FLT_OBJMASK_SPACING*(i*4+k)));
            auto addr = *pollingReadRegister(ADDR_C1FLTOBJ0+ (FLT_OBJMASK_SPACING*(i*4+k)));


            ESP_LOGI(LOG_TAG, "Filter RECORD is 0x%02x ADDR: %04x MASK %04x",
                     thing & 0xff,
                     addr,
                     mask & 0xfff);

            thing = thing >> 8;
        }


    }
    ESP_LOGI(LOG_TAG, "----------End Round------------");

}

void MCP2517FD::onWriteMessageQueueChange() {
    this->softInterrupt();
}

void MCP2517FD::onInterrupt() {
    this->interrupt();
}

