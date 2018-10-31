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
 /**
 * SPI Deffinitions header
 * http://ww1.microchip.com/downloads/en/DeviceDoc/MCP2517FD-External-CAN-FD-Controller-with-SPI-Interface-20005688B.pdf
 *

 */
#ifndef _MCP_2517FD_SPI_H
#define _MCP_2517FD_SPI_H


#ifdef __cplusplus 
extern "C" {
#endif

#include <stdint.h>


// SPI INSTRUCTIONS CMD
#define CMD_RESET 0x00
#define CMD_WRITE 0x02
#define CMD_READ 0x03
#define CMD_WRITE_CRC 0x0A
#define CMD_READ_CRC 0x0B
#define CMD_WRITE_SAFE 0x0C

// REGISTERS ADDRESSES
#define ADDR_C1CON 0x000
#define ADDR_C1NBTCFG 0x004
#define ADDR_C1DBTCFG 0x008
#define ADDR_C1TDC 0x00C

#define ADDR_C1TBC 0x010
#define ADDR_C1TSCON 0x014
#define ADDR_C1VEC 0x018
#define ADDR_C1INT 0x01C

#define ADDR_C1RXIF 0x020
#define ADDR_C1TXIF 0x024
#define ADDR_C1RXOVIF 0x028
#define ADDR_C1TXATIF 0x02C
#define ADDR_C1TXREQ 0x030
#define ADDR_C1TREC 0x034
#define ADDR_C1BDIAG0 0x038
#define ADDR_C1BDIAG1 0x03C
#define ADDR_C1TEFCON 0x040
#define ADDR_C1TEFSTA 0x044
#define ADDR_C1TEFUA 0x048
#define ADDR_RESERVED1 0x04C
#define ADDR_C1TXQCON 0x050
#define ADDR_C1TXQSTA 0x054

#define ADDR_C1TXQUA 0x058
#define ADDR_C1FIFOCON1 0x05C
#define ADDR_C1FIFOSTA1 0x060
#define ADDR_C1FIFOUA1 0x064
#define ADDR_C1FIFOCON2 0x068
#define ADDR_C1FIFOSTA2 0x06C
#define ADDR_C1FIFOUA2 0x070
#define ADDR_C1FIFOCON3 0x074
#define ADDR_C1FIFOSTA3 0x078
#define ADDR_C1FIFOUA3 0x07C

// Repeat until 1CF for a total of 32 FIFO regs

#define ADDR_C1FLTCON0 0x1D0
#define ADDR_C1FLTCON1 0x1D4
#define ADDR_C1FLTCON2 0x1D8
#define ADDR_C1FLTCON3 0x1DC
#define ADDR_C1FLTCON4 0x1E0
#define ADDR_C1FLTCON5 0x1E4
#define ADDR_C1FLTCON6 0x1E8
#define ADDR_C1FLTCON7 0x1EC

#define FLTCON_SIZE 1 // flt con size in bytes


#define ADDR_C1FLTOBJ0 0x1F0
#define ADDR_C1MASK0 0x1F4
#define FLT_OBJMASK_SPACING 0x08





// MCP2517FD REGISTER
#define ADDR_OSC 0xE00
#define ADDR_IOCON 0xE04
#define ADDR_CRC 0xE08
#define ADDR_ECCCON 0xE0C
#define ADDR_ECCSTAT 0xE10






typedef enum {
    CAN_NORMAL_MODE = 0x00,
    CAN_SLEEP_MODE = 0x01,
    CAN_INTERNAL_LOOPBACK_MODE = 0x02,
    CAN_LISTEN_ONLY_MODE = 0x03,
    CAN_CONFIGURATION_MODE = 0x04,
    CAN_EXTERNAL_LOOPBACK_MODE = 0x05,
    CAN_CLASSIC_MODE = 0x06,
    CAN_RESTRICTED_MODE = 0x07,
    CAN_INVALID_MODE = 0xFF
} CAN_OPERATION_MODE;



// Structures for registers

// *****************************************************************************

// System specific registers


// Oscilattor config register
typedef union _REG_OSC {
    struct {
        uint32_t PLL_ENABLE: 1; // READ_WRITE
        uint32_t RESERVED : 1;
        uint32_t OSCILLATOR_DISABLE : 1;
        uint32_t RESERVED2 : 1;
        uint32_t SYSTEM_CLOCK_DIVISOR: 1; // READ WRITE
        uint32_t CLOCK_OUTPUT_DIVISOR: 2; // 11 divided by 10, 00 divided by 1 READ-WRITE
        uint32_t RESERVED3 : 1; // READ ONLY
        uint32_t OSC_READY : 1; // READ ONLY
        uint32_t RESERVED4 : 1;
        uint32_t SCLKRDY : 1; // READ ONLY
    } b;
    uint32_t word;
    uint8_t byte[4];

} REG_OSC;

typedef union _REG_IOCON {
    struct {
        uint32_t TRIS0 : 1;
        uint32_t TRIS1 : 1;
        uint32_t RES : 4;
        uint32_t XSTNDBY : 1;
        uint32_t RES2 : 1;
        uint32_t LAT0 : 1;
        uint32_t LAT1 : 1;
        uint32_t RES3 : 6;
        uint32_t GPIO0 : 1;
        uint32_t GPIO1 : 1;
        uint32_t RES4 : 6;
        uint32_t PM0 : 1;
        uint32_t PM1 : 1;
        uint32_t RES5: 2;
        uint32_t TXCANOD : 1;
        uint32_t SOF : 1;
        uint32_t INTOD : 1;

    } b;
    uint32_t word;
    uint8_t byte[4];

} REG_IOCON;

/**
 * Non system specific registers
 */

//! CAN Control Register 3-4
typedef union _REG_CiCON {

    struct {
        uint32_t DNetFilterCount : 5; //Device Net Filter Bit Number bits RW
        uint32_t IsoCrcEnable : 1; // Enable ISO CRC in CAN FD Frames bit RW
        uint32_t ProtocolExceptionEventDisable : 1; // Protocol Exception Event Detection Disabled bit RW
        uint32_t unimplemented1 : 1;
        uint32_t WakeUpFilterEnable : 1; // Enable CAN Bus Line Wake-up Filter bit RW
        uint32_t WakeUpFilterTime : 2; // Selectable Wake-up Filter Time bits RW
        uint32_t Busy : 1; //  CAN Module is Busy bit R
        uint32_t BitRateSwitchDisable : 1; // Bit Rate Switching Disable bit RW ( CAN FD)
        uint32_t unimplemented3 : 3;
        uint32_t RestrictReTxAttempts : 1; // Restrict Retransmission Attempts bit RW
        uint32_t EsiInGatewayMode : 1; // Transmit ESI in Gateway Mode bit RW
        uint32_t SystemErrorToListenOnly : 1; // Transition to Listen Only Mode on System Error bit RW
        uint32_t StoreInTEF : 1; // : Store in Transmit Event FIFO bit RW
        uint32_t TXQEnable : 1; // : Enable Transmit Queue bit RW
        uint32_t OpMode : 3; // : Operation Mode Status bits R
        uint32_t RequestOpMode : 3; // : Request Operation Mode bits RW
        uint32_t AbortAllTx : 1; // : Abort All Pending Transmissions bit
        uint32_t TxBandWidthSharing : 4; // : Transmit Bandwidth Sharing bits
    } b;
    uint32_t word;
    uint8_t byte[4];
} REG_CiCON;


// NOMINAL BIT TIME CONFIGURATION REGISTER 3-7
typedef union _REG_CiNBTCFG {

    struct {
        uint32_t SyncJumpWidth : 7;
        uint32_t RES1 : 1;
        uint32_t TimeSegment2 : 7;
        uint32_t RES2 : 1;
        uint32_t TimeSegment1 : 8;
        uint32_t BaudRatePreScaler : 8;

    } b;
    uint32_t word;
    uint8_t byte[4];
} REG_CiNBTCFG;


//  DATA BIT TIME CONFIGURATION REGISTER 3-8
typedef union _REG_CiDBTCFG {

    struct {
        uint32_t SyncJumpWidth : 4;
        uint32_t RES1 : 4;
        uint32_t TimeSegment2 : 4;
        uint32_t RES2 : 4;
        uint32_t TimeSegment1 : 5;
        uint32_t BaudRatePreScaler : 8;
    } b;
    uint32_t word;
    uint8_t byte[4];
} REG_CiDBTCFG;


//  TRANSMITTER DELAY COMPENSATION REGISTER 3-9
typedef union _REG_CiTDC{

    struct {
        uint32_t TransmitterDelayCompensation : 6;
        uint32_t RES1 : 2;
        uint32_t TransmitterDelayOffset : 7;
        uint32_t RES2 : 1;
        uint32_t TransmitterDelayCompensationMode : 2;
        uint32_t RES3 : 6;
        uint32_t SID11EN : 1;
        uint32_t EDGFLTEN : 1;

    } b;
    uint32_t word;
    uint8_t byte[4];
} REG_CiTDC;


// Time Base Counter Register 3-10
typedef union _REG_CiTBC {

    struct {
        uint32_t TimeBaseCounter : 32;
    } b;
    uint32_t word;
    uint8_t byte[4];
} REG_CiTBC;


// Time Stamp Control Register 3-11
typedef union _REG_CiTSCON {

    struct {
        uint32_t TimeBaseCounterPrescaler : 10;
        uint32_t RES1 : 6;
        uint32_t TimeBaseCounterEnable : 1;
        uint32_t TimeStampEOF : 1;
        uint32_t TimeStampRes : 1; // Fd only

    } b;
    uint32_t word;
    uint8_t byte[4];
} REG_CiTSCON;


//  INTERRUPT CODE REGISTER 3-12
typedef union _REG_CiVEC{

    struct {
        uint32_t InterruptFlagCode : 7;
        uint32_t RES1 : 1;
        uint32_t FilterHitNumbers : 5;
        uint32_t RES2 : 3;
        uint32_t TransmitInterruptFlagCode : 7;
        uint32_t RES3 : 1;
        uint32_t RecieveInterruptFlagCode : 7;
        uint32_t RES4 : 1;

    } b;
    uint32_t word;
    uint8_t byte[4];
} REG_CiVEC;


//  INTERRUPT REGISTER 3-13
typedef union _REG_CiINT{

    struct {
        uint32_t TxFIFOFlag : 1;
        uint32_t RxFIFOFlag : 1;
        uint32_t TimeBaseCounterOverflowFlag : 1;
        uint32_t OperationModeChangeFlag : 1;
        uint32_t TxEventFIFOFlag : 1;
        uint32_t RES1 : 3;

        uint32_t ECCErrorFlag : 1;
        uint32_t SPICRCErrorFlag : 1;
        uint32_t TransmitAttemptInterruptFlag : 1;
        uint32_t ReceiveObjectOverflowInterruptFlag : 1;
        uint32_t SystemErrorFLag : 1;
        uint32_t CANBusErrorFlag : 1;
        uint32_t BusWakeUpFlag : 1;
        uint32_t InvalidMessageFlag : 1;

        uint32_t TxFIFO : 1;
        uint32_t RxFIFO : 1;
        uint32_t TimeBaseCounterOverflow : 1;
        uint32_t OperationModeChange : 1;
        uint32_t TxEventFIFO : 1;

        uint32_t RES2 : 3;

        uint32_t ECCError : 1;
        uint32_t SPICRCError : 1;
        uint32_t TransmitAttemptInterrupt : 1;
        uint32_t ReceiveObjectOverflow : 1;
        uint32_t SystemError : 1;
        uint32_t CANBusError : 1;
        uint32_t BusWakeUp : 1;
        uint32_t InvalidMessage : 1;



    } b;
    uint32_t word;
    uint8_t byte[4];
} REG_CiINT;


// *****************************************************************************
// fifo control regs
// *****************************************************************************

//  FIFO CONTROL REGISTER
#define FIFO_CTRL_SIZE (3*4) // 3 fifo registers each, for 12 bytes
typedef union _REG_CiFIFOCONm{

    struct {

        uint32_t FIFONotFullInterruptEn : 1; // Not empty if  Rx
        uint32_t FIFOHalfEmptyInterruptEn : 1; // half full if rx
        uint32_t FIFOEmptyInterruptEn : 1; // full if rx
        uint32_t OverflowInterruptEn : 1;
        uint32_t TransmitAttemptsExhaustedInterruptEn : 1;
        uint32_t ReceiveMessageTimeStamps : 1;
        uint32_t AutoRTREn : 1;
        uint32_t TxRxSel : 1;

        uint32_t IncrementHeadTail : 1;
        uint32_t MessageSendRequestBit : 1; // no effect if rx
        uint32_t FifoResetBit : 1;

        uint32_t RES1 : 5;

        uint32_t TxTransmitPriority : 5;
        uint32_t TxRetransmissionAttempt : 2; // This feature is enabled when CiCON.RTXAT is set.,
        uint32_t RES2 : 1;

        uint32_t FIFOSize : 5; // 1 ... 32
        uint32_t FIFOPayload : 3; // 1 ... 64







    } b;
    uint32_t word;
    uint8_t byte[4];
} REG_CiFIFOCONm;

//  FIFO STATUS Register
typedef union _REG_CiFIFOSTAm{

    struct {

        uint32_t FIFONotFullEmptyInterruptFlag : 1; // Not empty if  Rx
        uint32_t FIFOHalfEmptyFullInterruptFlag : 1; // half full if rx
        uint32_t FIFOEmptyFullInterruptFlag : 1; // full if rx
        uint32_t RecieveFIFOOverflowInterruptFlag : 1;
        uint32_t TransmitAttemptsExhaustedInterruptPending : 1;
        uint32_t ErrorDetectedDuringTransmission : 1;
        uint32_t MessageLostArbitratonStatus : 1;
        uint32_t MessageAbortedStatus : 1;

        uint32_t FifoMessageIndexBits : 5;

    } b;
    uint32_t word;
    uint8_t byte[4];
} REG_CiFIFOSTAm;


//  FIFO User Address Register
typedef union _REG_CiFIFOUAm{

    struct {

        uint32_t address: 32;
    } b;
    uint32_t word;
    uint8_t byte[4];
} REG_CiFIFOUAm;




// *****************************************************************************
// Transmit FIFO Registers
// *****************************************************************************

// Transmit Queue Control Register

typedef union _REG_CiTXQCON {

    struct {
        uint32_t TxNotFullInterruptEn : 1;
        uint32_t RES1 : 1;
        uint32_t TxEmptyInterruptEn : 1;
        uint32_t RES2 : 1;
        uint32_t TxAttemptsExhaustedInterruptEn : 1;
        uint32_t RES3 : 2;
        uint32_t TxEnable : 1;
        uint32_t IncrementHeadTail : 1;
        uint32_t TxRequest : 1;
        uint32_t FifoReset : 1;
        uint32_t RES4 : 5;
        uint32_t TxPriority : 5;
        uint32_t TxRetransmissionAttempts : 2;
        uint32_t RES5 : 1;
        uint32_t FifoSize : 5; // 1 ... 32
        uint32_t PayLoadSize : 3; // 8,12 ... 32,48,64
    } b;
    uint32_t word;
    uint8_t byte[4];
} REG_CiTXQCON;

//  Tx queue STATUS Register
typedef union _REG_CiTXQSTA{

    struct {

        uint32_t TransmitQueueNotFullInterruptFlag : 1; // Not empty if  Rx
        uint32_t RES0 : 1; // half full if rx
        uint32_t TransmitQueueEmptyINterruptFlag : 1; // full if rx
        uint32_t RES1 : 1;
        uint32_t TransmitAttemptsExhaustedInterruptPending : 1;
        uint32_t ErrorDetectedDuringTransmission : 1;
        uint32_t MessageLostArbitratonStatus : 1;
        uint32_t MessageAbortedStatus : 1;

        uint32_t FifoMessageIndexBits : 5;

    } b;
    uint32_t word;
    uint8_t byte[4];
} REG_CiTXQSTA;


//  Tx queue User Address Register
typedef union _REG_CiTXQUA{

    struct {

        uint32_t address: 32;
    } b;
    uint32_t word;
    uint8_t byte[4];
} REG_CiTXQUA;



//  Buffer control register (byte)
typedef union _REG_CiFLTCONm{

    struct {

        uint32_t targetFifo: 5;
        uint32_t res: 2;
        uint32_t  en: 1;

    } b;
    uint8_t byte;
} REG_CiFLTCONm;


// transmission message object (64 byte payload max)
// made to be cast to transmit buffer
typedef union _TRANSMIT_MESSAGE_OBJECT {
    struct {

        uint32_t standardIdentifier : 11;
        uint32_t extendedIdentifier: 10;
        uint32_t SID11 : 1;
        uint32_t RES1 : 2;

        // T1
        uint32_t DLC : 4;
        uint32_t ExtesnionFlag : 1;
        uint32_t RemoteTransmissionRequest : 1; // not used in can fd
        uint32_t BitRateSwitch : 1; // if data bit rate switching should be used
        uint32_t FDF : 1; // distinguish if FD frame
        uint32_t ESI : 1; // error status indiciator
        uint32_t Sequence: 7;
        uint32_t RES2 : 16;
        uint8_t data[64];


    } control;
    struct {
       uint32_t word1;
       uint32_t word2;
       uint8_t data[64];

    } prim;
} TRANSMIT_MESSAGE_OBJECT;






#ifdef __cplusplus
}
#endif

#endif // _MCP_2517FD_SPI_H