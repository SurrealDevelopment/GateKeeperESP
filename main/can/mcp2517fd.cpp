#include <can/mcp2517fd.h>
#include <cstring>
#include <can/mcp2517fd_spi.h>
#include <inttypes.h>
#include "mcp2517fd.h"
#include "mcp2517fd_spi.h"


#define DEBUG_REGISTERS true
uint32_t cmd;

static char LOG_TAG[] = "CANC";

MCP2517FD::MCP2517FD(spi_device_handle_t handle) {
    mReadBuffer = heap_caps_malloc(HEAP_SIZE, MALLOC_CAP_DMA);
    mWriteBuffer = heap_caps_malloc(HEAP_SIZE, MALLOC_CAP_DMA);
    mReadReg = (uint32_t *)(mReadBuffer); 
    mWriteReg = (uint32_t *)(mWriteBuffer); 

    
    ESP_LOGI(LOG_TAG, "Created new MCP2517FD Object");
    this->mHandle = handle;
}

MCP2517FD::~MCP2517FD() {

    heap_caps_free(mReadBuffer);
    heap_caps_free(mWriteBuffer);


}


void log_write_data_error(uint32_t address)
{
    ESP_LOGI(LOG_TAG, "Data write error detected address 0x%04x", address);
}

void log_read_data_error(uint32_t address)
{
    ESP_LOGI(LOG_TAG,"Data write error detected address 0x%04x", address);
}


uint32_t * MCP2517FD::readRegister(uint32_t address) {
    spi_transaction_t t = { };
   
    t.cmd = CMD_READ;
    t.addr = address;
    t.length = 32;
    t.rx_buffer = mReadBuffer;;
    t.tx_buffer = mWriteBuffer;

    ESP_ERROR_CHECK(spi_device_polling_transmit(mHandle, &t));
    return (uint32_t *)mReadBuffer;
}

// write 4 byte register at address
void MCP2517FD::writeRegister(uint32_t address) {
    spi_transaction_t t = { };

   
    /* Doing this apparently breaks the write. Avoid reading before writting. */
    #if DEBUG_REGISTERS
        ESP_LOGI(LOG_TAG, "Writting Registor 0x%04x with: 0x%08x", address, *(uint32_t *)mWriteReg);
        //uint32_t * reg = readRegister(address);
        //ESP_LOGI(LOG_TAG, "REGISTOR 0x%04x PRE: 0x%08x", address, *reg);
    #endif
    t.cmd = CMD_WRITE;
    t.addr = address;
    t.length = 32;
    t.rx_buffer = mReadBuffer;
    t.tx_buffer = mWriteBuffer;

    ESP_ERROR_CHECK(spi_device_polling_transmit(mHandle, &t));
    

    #if DEBUG_REGISTERS
        uint32_t * reg2 = readRegister(address);
        ESP_LOGI(LOG_TAG, "REGISTOR 0x%04x POST: 0x%08x",address, *reg2);
    #endif

    return;


}


// overload to write register from data word
void MCP2517FD::writeRegister(uint32_t address, uint32_t data) {
    
    // set data to buffer and call other function
    *(uint32_t *)mWriteBuffer = data;
    return writeRegister(address);
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
    REG_CiTXQCON txcon = {};

    txcon.b.PayLoadSize = 0b111; // 64 bytes
    txcon.b.FifoSize = 4; // 4 messages (256 bytes)
    txcon.b.TxRetransmissionAttempts = 2; // or 3 attempts
    txcon.b.TxPriority = 0b00111; // midish priority
    txcon.b.TxEmptyInterruptEn = 0;

    writeRegister(ADDR_C1TXQCON, txcon.word);

    // verify
    auto txconcheck = (REG_CiTXQCON *) readRegister(ADDR_C1TXQCON);


    // check
    if (txconcheck->b.PayLoadSize != txcon.b.PayLoadSize ||
        txconcheck->b.FifoSize != txcon.b.FifoSize ||
        txconcheck->b.TxRetransmissionAttempts != txcon.b.TxRetransmissionAttempts ||
        txconcheck->b.TxPriority != txcon.b.TxPriority)
    {
        log_write_data_error(ADDR_C1TXQCON);
        return false;
    }

    // configure priority 1 transmit FIFO
    REG_CiFIFOCONm fifoCon = {};

    fifoCon.b.TxRxSel=1;
    fifoCon.b.FIFOPayload = 0b111; // 64 bytes
    fifoCon.b.FIFOSize = 2; // 2 messages (128 bytes)
    fifoCon.b.TxRetransmissionAttempt = 2; // or 3 attempts
    fifoCon.b.TxTransmitPriority = 0b11111; // top priority

    writeRegister(ADDR_C1FIFOCON1, fifoCon.word);

    // verify
    auto fifoConCheck = (REG_CiFIFOCONm *) readRegister(ADDR_C1FIFOCON1);


    // check
    if (fifoConCheck->b.FIFOPayload != fifoCon.b.FIFOPayload ||
        fifoConCheck->b.FIFOSize != fifoCon.b.FIFOSize ||
        fifoConCheck->b.TxRetransmissionAttempt != fifoCon.b.TxRetransmissionAttempt ||
        fifoConCheck->b.TxTransmitPriority != fifoCon.b.TxTransmitPriority ||
        fifoConCheck->b.TxRxSel != fifoCon.b.TxRxSel)
    {
        log_write_data_error(ADDR_C1FIFOCON1);
        return false;
    }


    // Recieve FIFO
    fifoCon.word = 0;

    fifoCon.b.TxRxSel=0; // recieve
    fifoCon.b.FIFOPayload = 0b111; // 64 bytes
    fifoCon.b.FIFOSize = 16; // 16 messages (1024 bytes, 1kB)
    //fifoCon.b.TxRetransmissionAttempt = 2; // or 3 attempts
    //fifoCon.b.TxTransmitPriority = 0b11111; // top priority
    fifoCon.b.FIFOEmptyInterruptEn = 1; // enable interrupt if FIFO is full
    fifoCon.b.FIFONotFullInterruptEn = 1; // interrupt if something is in the FIFO
    fifoCon.b.ReceiveMessageTimeStamps = 1; // enable time stamps on messages


    writeRegister((ADDR_C1FIFOCON2), fifoCon.word);

    fifoConCheck = (REG_CiFIFOCONm *) readRegister(ADDR_C1FIFOCON2);


    // check
    if (fifoConCheck->b.FIFOPayload != fifoCon.b.FIFOPayload ||
        fifoConCheck->b.FIFOSize != fifoCon.b.FIFOSize ||
        fifoConCheck->b.FIFOEmptyInterruptEn != fifoCon.b.FIFOEmptyInterruptEn ||
        fifoConCheck->b.FIFONotFullInterruptEn != fifoCon.b.FIFONotFullInterruptEn ||
        fifoConCheck->b.TxRxSel != fifoCon.b.TxRxSel ||
        fifoConCheck->b.ReceiveMessageTimeStamps != fifoCon.b.ReceiveMessageTimeStamps)
    {
        log_write_data_error(ADDR_C1FIFOCON2);
        return false;
    }









    return false;
}
