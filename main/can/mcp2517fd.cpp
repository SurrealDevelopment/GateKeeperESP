#include <can/mcp2517fd.h>
#include <cstring>
#include <can/mcp2517fd_spi.h>
#include <inttypes.h>
#include "mcp2517fd.h"
#include "mcp2517fd_spi.h"


#define DEBUG_REGISTERS true
#define CLOCK_RATE_MHZ 40 // clock to mcp in MHz
#define CAN_SAMPLE_POINT 0.8 // Sample point for CAN. Were going to use 80%

static char LOG_TAG[] = "CANC";




// init everything in reg record to 0 to be safe.
void MCP2517FD::initRegisterRecord() {
    mRegRecord->canCon.word=0;
    mRegRecord->fifo0con.word=0;
    mRegRecord->fifo1con.word=0;
    mRegRecord->fifo2con.word=0;
    mRegRecord->fifo3con.word=0;
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


uint32_t * MCP2517FD::readRegister(uint32_t address) {
    spi_transaction_t t = { };
   
    t.cmd = CMD_READ;
    t.addr = address;
    t.length = 32;
    t.rx_buffer = mReadBuffer;;
    t.tx_buffer = mWriteBuffer;

    ESP_ERROR_CHECK(spi_device_polling_transmit(mHandle, &t));
    return mReadReg;
}

// write 4 byte register at default write buffer
void MCP2517FD::writeRegister(uint32_t address) {
    return writeRegisterFromAddress(address, mWriteBuffer);

}


// overload to write register from data word
void MCP2517FD::writeRegister(uint32_t address, uint32_t data) {
    
    // set data to buffer and call other function
    *(uint32_t *)mWriteBuffer = data;
    return writeRegister(address);
}

// write register from pointer somewhere in a DMA buffer
void MCP2517FD::writeRegisterFromAddress(uint32_t address, void *data) {

    spi_transaction_t t = { };


    /* Doing this apparently breaks the write. Avoid reading before writting. */
#if DEBUG_REGISTERS
    ESP_LOGI(LOG_TAG, "Writting Registor 0x%04x with: 0x%08x", address, *(uint32_t *)data);
    //uint32_t * reg = readRegister(address);
    //ESP_LOGI(LOG_TAG, "REGISTOR 0x%04x PRE: 0x%08x", address, *reg);
#endif
    t.cmd = CMD_WRITE;
    t.addr = address;
    t.length = 32;
    t.rx_buffer = mReadBuffer;
    t.tx_buffer = data;

    ESP_ERROR_CHECK(spi_device_polling_transmit(mHandle, &t));


#if DEBUG_REGISTERS
    uint32_t * reg2 = readRegister(address);
    ESP_LOGI(LOG_TAG, "REGISTOR 0x%04x POST: 0x%08x",address, *reg2);
#endif
}

void MCP2517FD::resumeInterrupts()
{
    writeRegister(ADDR_C1INT, 0xB8030000);
     //Enable Invalid Msg, Bus err, sys err, rx overflow, rx fifo, tx fifo interrupts

}

void MCP2517FD::stopInterrupts()
{
    writeRegister(ADDR_C1INT, 0);
}

bool MCP2517FD::initPins() {


    // Setup oscillator
    REG_OSC regOsc = { };
    regOsc.b.PLL_ENABLE=0;
    regOsc.b.SYSTEM_CLOCK_DIVISOR = 0;
    regOsc.b.CLOCK_OUTPUT_DIVISOR = 0; // set clock output to raw (no divsion)

    writeRegister(ADDR_OSC, regOsc.word);

    // read back and verify
    auto reg2 = (REG_OSC *) readRegister(ADDR_OSC);

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
    regIOCON.b.LAT1 = 1;
    regIOCON.b.TRIS0 = 0;
    regIOCON.b.TRIS1 = 0;


    writeRegister(ADDR_IOCON, regIOCON.word);


    auto ioconcheck = (REG_IOCON *) readRegister(ADDR_IOCON);


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

    writeRegister(ADDR_C1TSCON, tsCon.word);

    auto tsConCheck = (REG_CiTSCON *) readRegister(ADDR_C1TSCON);

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

    writeRegisterFromAddress(ADDR_C1TXQCON, &(txcon->word));

    // verify
    auto txconcheck = (REG_CiTXQCON *) readRegister(ADDR_C1TXQCON);


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

    writeRegisterFromAddress(ADDR_C1FIFOCON1, fifoCon);

    // verify
    auto fifoConCheck = (REG_CiFIFOCONm *) readRegister(ADDR_C1FIFOCON1);


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
    writeRegisterFromAddress(ADDR_C1FIFOCON2, fifoCon);


    fifoConCheck = (REG_CiFIFOCONm *) readRegister(ADDR_C1FIFOCON2);


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
    fifoCon->b.ReceiveMessageTimeStamps = 1; // enable time stamps on messages


    writeRegisterFromAddress((ADDR_C1FIFOCON3), fifoCon);

    fifoConCheck = (REG_CiFIFOCONm *) readRegister(ADDR_C1FIFOCON3);


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

    return false;
}

bool MCP2517FD::startCAN(uint32_t CAN_Baud_Rate, bool listenOnly) {



    if (CAN_Baud_Rate > 1000000)
    {
        ESP_LOGV(LOG_TAG, "CAN Baud Rate Too High!");
        return false;
    }

    // stop interrupts for now
    stopInterrupts();

    // Can control register

    // reset
    auto con = &(mRegRecord->canCon);

    con->b.DNetFilterCount = 0; // dont use
    con->b.IsoCrcEnable = 0; // Use CRC for can-fd (useless here)
    con->b.ProtocolExceptionEventDisable = 0; // Nah
    con->b.WakeUpFilterEnable = 1; // need wake up for low power use
    con->b.WakeUpFilterTime = 2; // min time
    con->b.BitRateSwitchDisable = 1; // No FD
    con->b.RestrictReTxAttempts = 1;
    con->b.EsiInGatewayMode = 0;
    con->b.SystemErrorToListenOnly = 1;
    con->b.StoreInTEF = 0;
    con->b.TXQEnable = 1; // basically FIFO0 which we are using
    con->b.TxBandWidthSharing = 0; // This adds a delay at end of transmisisons. Which we don't want.
    con->b.RequestOpMode = CAN_CONFIGURATION_MODE; // make sure its in config mode (it probably already is)

    // write controller
    writeRegisterFromAddress(ADDR_C1CON, &(mRegRecord->canCon.word));

    con->b.RequestOpMode = 0;

    // check
    if (!quickRegisterCheck(con->word, *readRegister(ADDR_C1CON)))
    {
        log_write_data_error(ADDR_C1CON);
        return false;
    }


    // This register deals with bit itming
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

    nominalBitTimeConfig.b.TimeSegment1 = a - 1; // this is prop+ Phase segment 1, minus 1 due to calc
    nominalBitTimeConfig.b.TimeSegment2 = b - 1; // Phase segment 2 only

    // from calculating Time Segments with SYS CLOCK we dont need to calculate that here
    // it will work in multiples of 2 so 1000000 would be 0
    // This controls how long each Time quanta is.
    nominalBitTimeConfig.b.BaudRatePreScaler = (1000000ul / CAN_Baud_Rate) - 1;


    // dont worry about data register
    // write bit timing
    writeRegister(ADDR_C1NBTCFG, nominalBitTimeConfig.word);

    // check
    if (!quickRegisterCheck(nominalBitTimeConfig.word, *readRegister(ADDR_C1NBTCFG)))
    {
        log_write_data_error(ADDR_C1NBTCFG);
        return false;
    }


    // Set mode
    if (listenOnly) {
        if (!changeMode(CAN_CLASSIC_MODE))
            return false;
    }
    else {
        if (!changeMode(CAN_LISTEN_ONLY_MODE))
            return false;
    }

    resumeInterrupts();
    // Good!
    return true;
}

bool MCP2517FD::startCANFD(uint32_t nominal_CAN_Baud_Rate, uint32_t data_CAN_BAUD_Rate) {



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
    writeRegister(ADDR_C1NBTCFG, nominalBitTimeConfig.word);

    // check
    if (!quickRegisterCheck(nominalBitTimeConfig.word, *readRegister(ADDR_C1NBTCFG)))
    {
        log_write_data_error(ADDR_C1NBTCFG);
        return false;
    }

    // write bit data timing
    writeRegister(ADDR_C1DBTCFG, dataBitTimingConfig.word);

    // check
    if (!quickRegisterCheck(dataBitTimingConfig.word, *readRegister(ADDR_C1DBTCFG)))
    {
        log_write_data_error(ADDR_C1DBTCFG);
        return false;
    }


    if (!changeMode(CAN_CONFIGURATION_MODE))
        return false;


    resumeInterrupts();




    return false;
}











bool MCP2517FD::changeMode(uint32_t mode) {
    mRegRecord->canCon.b.RequestOpMode = mode;

    // write controller
    writeRegisterFromAddress(ADDR_C1CON, &(mRegRecord->canCon.word));

    mRegRecord->canCon.b.RequestOpMode = 0;

    // check
    if (!quickRegisterCheck((mRegRecord->canCon.word), *readRegister(ADDR_C1CON)))
    {
        ESP_LOGV(LOG_TAG, "Failed to change CAN Mode.");
        return false;
    }
    return true;

}

/**
 * Interrupt logic
 */
void MCP2517FD::interrupt() {
    const auto reg = (REG_CiINT *) readRegister(ADDR_C1INT);
    const auto iReg = reg->word;


    if (iReg & 1) // Tx FIFO
    {
        //@TODO check our queue, send more if we can
    }
    if (iReg & 2) // Rx FIFO
    {
        // we only have one receive fifo, fifo3, so just read from that
        // first step is get current TAIL of the FIFO using user address register
        auto rxTail = *readRegister(ADDR_C1FIFOUA3);

        // rxTail needs to be offset because hey why not even though its a 32 bit register
        rxTail = rxTail + 0x400;



    }
    if (iReg & (1<<11)) // Rx FIFO overflow
    {

    }
    if (iReg & (1<<12)) // system error
    {

    }
    if (iReg & (1<<13)) // CAN error
    {

    }
    if (iReg & (1<<14)) // WAKE UP
    {

    }
    if (iReg & (1<<15)) // invalid msg
    {

    }
}


