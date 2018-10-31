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

#include <obd2/can/mcp2517fd.h>
#include <cstring>
#include <obd2/can/mcp2517fd_spi.h>
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
    mRegRecord->canCon.word=0;
    mRegRecord->fifo0con.word=0;
    mRegRecord->fifo1con.word=0;
    mRegRecord->fifo2con.word=0;
    mRegRecord->fifo3con.word=0;



    // set all filters to target fifo3
    for (int i = 0; i < 32; i++)
    {
        mRegRecord->fltcon[i].b.targetFifo=3;
    }
}



MCP2517FD::MCP2517FD(spi_device_handle_t handle) {
    mReadBuffer = heap_caps_malloc(HEAP_SIZE, MALLOC_CAP_DMA);
    mWriteBuffer = heap_caps_malloc(HEAP_SIZE, MALLOC_CAP_DMA);
    mReadReg = (uint32_t *)(mReadBuffer); 
    mWriteReg = (uint32_t *)(mWriteBuffer);

    // allocate DMA register buffer
    mRegisterVoidPtr = heap_caps_malloc(sizeof(RegisterRecord), MALLOC_CAP_DMA);
    mRegRecord = (RegisterRecord *) mRegisterVoidPtr;

    initRegisterRecord();


    
    ESP_LOGI(LOG_TAG, "Created new MCP2517FD Object");
    this->mHandle = handle;
}

MCP2517FD::~MCP2517FD() {

    heap_caps_free(mReadBuffer);
    heap_caps_free(mWriteBuffer);
    heap_caps_free(mRegisterVoidPtr);


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
    mRegRecord->interrupt.b.InvalidMessage=1; // Invalid MSG Interrupt
    mRegRecord->interrupt.b.BusWakeUp=1; // BUS WAKE UP for waking on low power
    mRegRecord->interrupt.b.CANBusError = 1; // CAN Bus error
    mRegRecord->interrupt.b.SystemError = 1; //
    mRegRecord->interrupt.b.ReceiveObjectOverflow = 1; // RxOverflow
    mRegRecord->interrupt.b.RxFIFO = 1; // recieve fifo
    mRegRecord->interrupt.b.TxFIFO = 1; // TX Fifo


    pollingWriteRegisterFromAddress(ADDR_C1INT, mRegRecord);

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
    REG_CiTXQCON * txcon = &(mRegRecord->fifo0con);

    txcon->b.PayLoadSize = 0b111; // 64 bytes
    txcon->b.FifoSize = 4; // 4 messages (256 bytes)
    txcon->b.TxRetransmissionAttempts = 2; // or 3 attempts
    txcon->b.TxPriority = 0b00111; // midish priority
    txcon->b.TxEmptyInterruptEn = 0;

    pollingWriteRegisterFromAddress(ADDR_C1TXQCON, txcon);

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
    REG_CiFIFOCONm * fifoCon = &(mRegRecord->fifo1con);

    fifoCon->b.TxRxSel=1;
    fifoCon->b.FIFOPayload = 0b111; // 64 bytes
    fifoCon->b.FIFOSize = 2; // 2 messages (128 bytes)
    fifoCon->b.TxRetransmissionAttempt = 2; // or 3 attempts
    fifoCon->b.TxTransmitPriority = 0b11111; // top priority

    pollingWriteRegisterFromAddress(ADDR_C1FIFOCON1, fifoCon);

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

    // FIFO 2 config

    fifoCon = &(mRegRecord->fifo2con);
    fifoCon->b.TxRxSel=1;
    fifoCon->b.FIFOPayload = 0b111; // 64 bytes
    fifoCon->b.FIFOSize = 2; // 2 messages (128 bytes)
    fifoCon->b.TxRetransmissionAttempt = 2; // or 3 attempts
    fifoCon->b.TxTransmitPriority = 0; // lowest priority
    pollingWriteRegisterFromAddress(ADDR_C1FIFOCON2, fifoCon);


    fifoConCheck = (REG_CiFIFOCONm *) pollingReadRegister(ADDR_C1FIFOCON2);


    // check
    if (fifoConCheck->b.FIFOPayload != fifoCon->b.FIFOPayload ||
        fifoConCheck->b.FIFOSize != fifoCon->b.FIFOSize ||
        fifoConCheck->b.TxRetransmissionAttempt != fifoCon->b.TxRetransmissionAttempt ||
        fifoConCheck->b.TxTransmitPriority != fifoCon->b.TxTransmitPriority ||
        fifoConCheck->b.TxRxSel != fifoCon->b.TxRxSel)
    {
        log_write_data_error(ADDR_C1FIFOCON2);
        return false;
    }




    // Recieve FIFO3
    fifoCon = &(mRegRecord->fifo3con);

    fifoCon->word = 0;

    fifoCon->b.TxRxSel=0; // recieve
    fifoCon->b.FIFOPayload = 0b111; // 64 bytes
    fifoCon->b.FIFOSize = 16; // 16 messages (1024 bytes, 1kB)
    //fifoC->n.b.TxRetransmissionAttempt = 2; // or 3 attempts
    //fifoC->n.b.TxTransmitPriority = 0b11111; // top priority
    fifoCon->b.FIFOEmptyInterruptEn = 1; // enable interrupt if FIFO is full
    fifoCon->b.FIFONotFullInterruptEn = 1; // interrupt if something is in the FIFO
    fifoCon->b.ReceiveMessageTimeStamps = TIMESTAMP; // enable time stamps on messages


    pollingWriteRegisterFromAddress((ADDR_C1FIFOCON3), fifoCon);

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
    auto con = &(mRegRecord->canCon);

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
    pollingWriteRegisterFromAddress(ADDR_C1CON, &(mRegRecord->canCon));

    con->b.RequestOpMode = 0;

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
    auto canCon = &(mRegRecord->canCon);

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
    mRegRecord->canCon.b.RequestOpMode = mode;

    // write controller
    pollingWriteRegisterFromAddress(ADDR_C1CON, &(mRegRecord->canCon));

    mRegRecord->canCon.b.RequestOpMode = 0;

    // check
    if (!quickRegisterCheck((mRegRecord->canCon.word), *pollingReadRegister(ADDR_C1CON)))
    {
        ESP_LOGV(LOG_TAG, "Failed to change CAN Mode.");
        return false;
    }
    return true;

}

/**
 * Interrupt logic. Interrupts are going to use their own transactions
 */
void MCP2517FD::interrupt() {

    // get exclusive access on bus for quicker speeds
    spi_device_acquire_bus(mHandle, portMAX_DELAY);

    const auto reg = (REG_CiINT *) pollingReadRegister(ADDR_C1INT);


    const auto iReg = reg->word;

    //ESP_LOGI(LOG_TAG, "INTERRUPT DETECTED! REGISTER: 0x%08x",(iReg));


    if (iReg & 1) // Tx FIFO
    {
        //@TODO check our queue, send more if we can
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
                ESP_LOGI(LOG_TAG, "Engine RPM from 0x%04x is %f",id, rpmf);

            }


            ESP_LOGI(LOG_TAG, "RX from 0x%04x 0x%04x%04x",id, data32[3], data32[4]);



            //ESP_LOGI(LOG_TAG, "New CAN Message AT FIFO 0x%04x Status: 0x%08x", rxTail, statusReg);
            //ESP_LOGI(LOG_TAG, "Byte 2: 0x%08x",(bytes[1]));
            //ESP_LOGI(LOG_TAG, "Byte 3: 0x%08x",(bytes[2]));
            //ESP_LOGI(LOG_TAG, "Byte 4: 0x%08x",(bytes[3]));

            // set uinc of FIFOCON to increment the FIFO because we read the message
            mRegRecord->fifo3con.b.IncrementHeadTail = 1;
            pollingWriteRegisterFromAddress(ADDR_C1FIFOCON3, &(mRegRecord->fifo3con));
            mRegRecord->fifo3con.b.IncrementHeadTail = 0;



            // get status again
            statusReg = *pollingReadRegister(ADDR_C1FIFOSTA3);

        }

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

    // write back INT acking the interrupt
    pollingWriteRegisterFromAddress(ADDR_C1INT, &(mRegRecord->interrupt));


}

bool MCP2517FD::generalInit() {

    initRegisterRecord(); // make sure register records are reset

    bool failFlag = true;
    // get exclusive access on bus for quicker speeds
    spi_device_acquire_bus(mHandle, portMAX_DELAY);

    reset();
    vTaskDelay(200/portTICK_RATE_MS); // give it 50 ms of time for reset.

    if (!initPins()) failFlag = false;
    // continue anyway even if we fail just in case
    if (!initFifo()) failFlag = false;

    // reset filters which will target fifo3
    writeAllFiltersStatus();

    // release bus
    spi_device_release_bus(mHandle);

    return failFlag;
}



void MCP2517FD::listenAll() {
    disableAllFilters();
    writeAllFiltersStatus();
    setFilter(0, 0x0c9, 0x7ff, false);
    setFilterStatus(0, true);
    writeAllFiltersStatus();
}

void MCP2517FD::writeAllFiltersStatus() {
    // overwrite all filters with current.
    // each filter is 1 byte so write 4 bytes since there are 32
    pollingWriteAddress(ADDR_C1FLTCON0, &(mRegRecord->fltcon), 32 / 4);
}

void MCP2517FD::disableAllFilters() {
    for (auto && item : mRegRecord->fltcon)
    {
        item.b.en=0;
    }
}



// sets mask and id filter of filter num
// NOTE filter must be DISABLEd for this to work
// Does not enable filter
void MCP2517FD::setFilter(uint32_t filterNum, uint32_t id, uint32_t mask, bool extended) {
    if (filterNum > 31) return; // sanity

    if (extended) id |= 1 << 30; // set extended bit (only will accept extended)

#include<bits/stdc++.h>#include<bits/stdc++.h
"xtensa-esp32-elf/bits/c++config.h"
"assert.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"
"xtensa-esp32-elf/bits/c++config.h"

    pollingWriteRegister(ADDR_C1FLTOBJ0 + (FLT_OBJMASK_SPACING * filterNum), id);

    mask |= 1 << 30; // insures mask follows extended bit

    pollingWriteRegister(ADDR_C1MASK0 + (FLT_OBJMASK_SPACING * filterNum), mask);
}

void MCP2517FD::setFilterStatus(uint32_t filterNum, bool status) {
    if (filterNum > 31) return; // sanity

    if (status)
        mRegRecord->fltcon[filterNum].b.en=1;
    else
        mRegRecord->fltcon[filterNum].b.en=0;

}

/**
 * Function for optimizing filters
 */
void MCP2517FD::optimizeFilters()
{
    // set disabled filter addrs to max
    for (int i = 0; i < 32; i++)
    {
        const auto flt = &(mRegRecord->fltcon[i]);
        uint32_t * addr= &(mRegRecord->fltAddrMask[i].first);

        if (flt->b.en)
        {
            *addr = UINT32_MAX;
        }
    }
    while (optimizeFiltersStep()); // repeat step as long as true


    // now that we optimized our lists lets rewrite everything
    // Note this will take some time to do and we can miss messages here
    // disable all filters
    disableAllFilters();
    // rewrite all filters
    for (uint32_t i = 0; i < filterStackCount; i++)
    {
        setFilter(i, mRegRecord->fltAddrMask->first, mRegRecord->fltAddrMask->second, false); // @TODO extended addressing
        setFilterStatus(i, true);
    }

    writeAllFiltersStatus();

}

bool MCP2517FD::optimizeFiltersStep() {
    // sort
    std::sort(std::begin(mRegRecord->fltAddrMask), std::end(mRegRecord->fltAddrMask));

    // count size
    for (filterStackCount = 0; filterStackCount < 32; filterStackCount++)
    {
        if (mRegRecord->fltAddrMask[filterStackCount].first == UINT32_MAX)
            break;
    }

    bool change = false;
    // merge
    for (int i = 0; i < filterStackCount -1; i++)
    {
        auto fltA = &(mRegRecord->fltAddrMask[i]);
        auto fltB = &(mRegRecord->fltAddrMask[i+1]);
        auto calc = fltA->first ^ fltB->first;
        // check octa
        if ((fltA->second & 0x7FF) == 0x7FC && (fltB->second & 0x7FF) == 0x7FC && calc == 4)
        {
            // upgrade to octa
            fltB->first=fltA->first;
            fltB->second=0x7F8; // @TODO extended addressing support
            fltA->first=UINT32_MAX; // insure to make flt A irrelevant
            change = true;
            // making fltA irrelevant means we can continue on the algo
        }
        else if ((fltA->second & 0x7FF) == 0x7FE && (fltB->second & 0x7FF) == 0x7FE && calc == 2)
        {
            // upgrade to quad
            fltB->first=fltA->first;
            fltB->second=0x7FC;
            fltA->first=UINT32_MAX; // insure to make flt A irrelevant
            change = true;

        }
        else if ((fltA->second & 0x7FF) == 0x7FF && (fltB->second & 0x7FF) == 0x7FF && calc == 1)
        {
            // upgrade to pair
            fltB->first=fltA->first;
            fltB->second=0x7FE;
            fltA->first=UINT32_MAX; // insure to make flt A irrelevant
            change = true;
        }
        else
        {
            // do nothing, loners
        }
    }
    return change;


}

bool MCP2517FD::listenTo(uint32_t address, bool extended) {


    // check if in filters already
    for (int i = 0; i < filterStackCount; i++)
    {
        const auto flt= &(mRegRecord->fltcon[i]);
        uint32_t * mask= &(mRegRecord->fltAddrMask[i].second);
        uint32_t * addr= &(mRegRecord->fltAddrMask[i].first);
        uint32_t calc = address & *mask;  // if address in filter calc will == addr


        if (flt->b.en && calc == *addr)
        {
            return true; // do nothing just return
        }
    }

    // check if filter exists
    if (filterStackCount > 31)
    {
        // try optimizing filters to get it under 31
        optimizeFilters();
        if (filterStackCount > 31)
        {
            ESP_LOGW(LOG_TAG, "Filter Overflow");
            return false;
        }
    }

    // add filter
    setFilter(filterStackCount, address, 0x7FF, false); // @TODO accept extended addressing
    setFilterStatus(filterStackCount, true);
    writeAllFiltersStatus();
    filterStackCount++;
    return false;
}

void MCP2517FD::stopListeningTo(uint32_t address, bool extended) {
    // stop listening is going to be a lot harder than simply listening
    // reason being we need to deal with splitting filters up.

    // check if in filters already
    for (uint32_t i = 0; i < filterStackCount; i++)
    {
        const auto flt= &(mRegRecord->fltcon[i]);
        uint32_t * mask= &(mRegRecord->fltAddrMask[i].second);
        uint32_t * addr= &(mRegRecord->fltAddrMask[i].first);
        uint32_t calc = address & *mask;  // if address in filter calc will == addr


        if (flt->b.en && calc == *addr)
        {
            uint32_t range = (*mask ^ 0x7FF) + 1; // xor will be range
            // filter found
            // disable filter
            setFilterStatus(i, false);
            writeAllFiltersStatus();

            // Its common for the removed filter to be the last filter
            // so we can save some optimization steps by subtracting the stack counter
            if (i == filterStackCount-1)
                filterStackCount--;

            for (uint32_t b = *addr; b < *addr + range; b++)
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



void MCP2517FD::writeTest() {

    auto status = (REG_CiFIFOSTAm *) pollingReadRegister(ADDR_C1TXQSTA); // med pri


    // same story as recieving frames
    while (status->word & 1)
    {

        vTaskDelay(100/portTICK_RATE_MS);

        auto addr = *pollingReadRegister(ADDR_C1TXQUA); // med pri

        addr = addr + 0x400; // offset


        ESP_LOGI(LOG_TAG, "ADDR: 0x%08x",addr);


        auto buf = (TRANSMIT_MESSAGE_OBJECT *)mWriteBuffer;

        buf->prim.word1=0;
        buf->prim.word2=0;
        buf->control.standardIdentifier=0x7E0;
        buf->control.ExtesnionFlag=0;
        buf->control.FDF = 0;
        buf->control.DLC=8;

        buf->prim.data[0] = 01;


        pollingWriteAddress(addr, mWriteBuffer, 64); // 64 bytes + 8 control

        // set uinc of FIFOCON to increment the FIFO because we transmit the message
        mRegRecord->fifo0con.b.IncrementHeadTail = 1;
        mRegRecord->fifo0con.b.TxRequest = 1;

        pollingWriteRegisterFromAddress(ADDR_C1TXQCON, &(mRegRecord->fifo0con));
        mRegRecord->fifo0con.b.IncrementHeadTail = 0;
        mRegRecord->fifo0con.b.TxRequest = 0;


        status = (REG_CiFIFOSTAm *) pollingReadRegister(ADDR_C1TXQSTA);
    }


}

