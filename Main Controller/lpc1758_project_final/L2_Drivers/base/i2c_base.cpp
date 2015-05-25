/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

#include <string.h>         // memcpy
#include "i2c_base.hpp"
#include "lpc_sys.h"
#include "printf_lib.h"
#include "io.hpp"
#include "utilities.h"
#include "stdlib.h"
#include "semphr.h"
#include "project.hpp"

/**
 * Instead of using a dedicated variable for read vs. write, we just use the LSB of
 * the user address to indicate read or write mode.
 */
#define I2C_SET_READ_MODE(addr)     (addr |= 1)     ///< Set the LSB to indicate read-mode
#define I2C_SET_WRITE_MODE(addr)    (addr &= 0xFE)  ///< Reset the LSB to indicate write-mode
#define I2C_READ_MODE(addr)         (addr & 1)      ///< Read address is ODD
#define I2C_WRITE_ADDR(addr)        (addr & 0xFE)   ///< Write address is EVEN
#define I2C_READ_ADDR(addr)         (addr | 1)      ///< Read address is ODD

extern SemaphoreHandle_t i2c_receive;

bool config = false;
bool status = false;
int i = 0, j = 0;
uint8_t config_register[5] = { 0 };
int interrupted_mailbox = 0;

bool I2C_Base::initSlave(uint8_t address, uint8_t *mem)     // Setting the slave address as 0x50
{
    LPC_I2C2->I2CONSET = 0x44;
    LPC_I2C2->I2ADR0 = address;
//    LPC_I2C2->I2MASK0 = 0x00;
    return true;
}

void I2C_Base::handleInterrupt()
{
    /* If transfer finished (not busy), then give the signal */
    if (busy != i2cStateMachine()) {
        long higherPriorityTaskWaiting = 0;
        xSemaphoreGiveFromISR(mTransferCompleteSignal, &higherPriorityTaskWaiting);
        portEND_SWITCHING_ISR(higherPriorityTaskWaiting);
    }
}

uint8_t I2C_Base::readReg(uint8_t deviceAddress, uint8_t registerAddress)
{
    uint8_t byte = 0;
    readRegisters(deviceAddress, registerAddress, &byte, 1);
    return byte;
}

bool I2C_Base::readRegisters(uint8_t deviceAddress, uint8_t firstReg, uint8_t* pData, uint32_t bytesToRead)
{
    I2C_SET_READ_MODE(deviceAddress);
    return transfer(deviceAddress, firstReg, pData, bytesToRead);
}

bool I2C_Base::writeReg(uint8_t deviceAddress, uint8_t registerAddress, uint8_t value)
{
    return writeRegisters(deviceAddress, registerAddress, &value, 1);
}

bool I2C_Base::writeRegisters(uint8_t deviceAddress, uint8_t firstReg, uint8_t* pData, uint32_t bytesToWrite)
{
    I2C_SET_WRITE_MODE(deviceAddress);
    return transfer(deviceAddress, firstReg, pData, bytesToWrite);
}

bool I2C_Base::transfer(uint8_t deviceAddress, uint8_t firstReg, uint8_t* pData, uint32_t transferSize)
{
    bool status = false;
    if(mDisableOperation || !pData) {
        return status;
    }

    // If scheduler not running, perform polling transaction
    if(taskSCHEDULER_RUNNING != xTaskGetSchedulerState())
    {
        i2cKickOffTransfer(deviceAddress, firstReg, pData, transferSize);

        // Wait for transfer to finish
        const uint64_t timeout = sys_get_uptime_ms() + I2C_TIMEOUT_MS;
        while (!xSemaphoreTake(mTransferCompleteSignal, 0)) {
            if (sys_get_uptime_ms() > timeout) {
                break;
            }
        }

        status = (0 == mTransaction.error);
    }
    else if (xSemaphoreTake(mI2CMutex, OS_MS(I2C_TIMEOUT_MS)))
    {
        // Clear potential stale signal and start the transfer
        xSemaphoreTake(mTransferCompleteSignal, 0);
        i2cKickOffTransfer(deviceAddress, firstReg, pData, transferSize);

        // Wait for transfer to finish and copy the data if it was read mode
        if (xSemaphoreTake(mTransferCompleteSignal, OS_MS(I2C_TIMEOUT_MS))) {
            status = (0 == mTransaction.error);
        }

        xSemaphoreGive(mI2CMutex);
    }

    return status;
}

bool I2C_Base::checkDeviceResponse(uint8_t deviceAddress)
{
    uint8_t dummyReg = 0;
    uint8_t notUsed = 0;

    // The I2C State machine will not continue after 1st state when length is set to 0
    uint32_t lenZeroToTestDeviceReady = 0;

    return readRegisters(deviceAddress, dummyReg, &notUsed, lenZeroToTestDeviceReady);
}

I2C_Base::I2C_Base(LPC_I2C_TypeDef* pI2CBaseAddr) :
        mpI2CRegs(pI2CBaseAddr),
        mDisableOperation(false)
{
    mI2CMutex = xSemaphoreCreateMutex();
    mTransferCompleteSignal = xSemaphoreCreateBinary();

    /// Binary semaphore needs to be taken after creating it
    xSemaphoreTake(mTransferCompleteSignal, 0);

    if((unsigned int)mpI2CRegs == LPC_I2C0_BASE)
    {
        mIRQ = I2C0_IRQn;
    }
    else if((unsigned int)mpI2CRegs == LPC_I2C1_BASE)
    {
        mIRQ = I2C1_IRQn;
    }
    else if((unsigned int)mpI2CRegs == LPC_I2C2_BASE)
    {
        mIRQ = I2C2_IRQn;
    }
    else {
        mIRQ = (IRQn_Type)99; // Using invalid IRQ on purpose
    }
}

bool I2C_Base::init(uint32_t pclk, uint32_t busRateInKhz)
{
    // Power on I2C
    switch(mIRQ) {
        case I2C0_IRQn: lpc_pconp(pconp_i2c0, true);  break;
        case I2C1_IRQn: lpc_pconp(pconp_i2c1, true);  break;
        case I2C2_IRQn: lpc_pconp(pconp_i2c2, true);  break;
        default: return false;
    }

    mpI2CRegs->I2CONCLR = 0x6C;           // Clear ALL I2C Flags

    /**
     * Per I2C high speed mode:
     * HS mode master devices generate a serial clock signal with a HIGH to LOW ratio of 1 to 2.
     * So to be able to optimize speed, we use different duty cycle for high/low
     *
     * Compute the I2C clock dividers.
     * The LOW period can be longer than the HIGH period because the rise time
     * of SDA/SCL is an RC curve, whereas the fall time is a sharper curve.
     */
    const uint32_t percent_high = 40;
    const uint32_t percent_low = (100 - percent_high);
    const uint32_t freq_hz = (busRateInKhz > 1000) ? (100 * 1000) : (busRateInKhz * 1000);
    const uint32_t half_clock_divider = (pclk / freq_hz) / 2;
    mpI2CRegs->I2SCLH = (half_clock_divider * percent_high) / 100;
    mpI2CRegs->I2SCLL = (half_clock_divider * percent_low ) / 100;

    // Set I2C slave address and enable I2C
    mpI2CRegs->I2ADR0 = 0;
    mpI2CRegs->I2ADR1 = 0;
    mpI2CRegs->I2ADR2 = 0;
    mpI2CRegs->I2ADR3 = 0;

    // Enable I2C and the interrupt for it
    mpI2CRegs->I2CONSET = 0x40;
    NVIC_EnableIRQ(mIRQ);

    return true;
}



/// Private ///

void I2C_Base::i2cKickOffTransfer(uint8_t devAddr, uint8_t regStart, uint8_t* pBytes, uint32_t len)
{
    mTransaction.error     = 0;
    mTransaction.slaveAddr = devAddr;
    mTransaction.firstReg  = regStart;
    mTransaction.trxSize   = len;
    mTransaction.pMasterData   = pBytes;

    // Send START, I2C State Machine will finish the rest.
    mpI2CRegs->I2CONSET = 0x20;
}

/*
 * I2CONSET bits
 * 0x04 AA
 * 0x08 SI
 * 0x10 STOP
 * 0x20 START
 * 0x40 ENABLE
 *
 * I2CONCLR bits
 * 0x04 AA
 * 0x08 SI
 * 0x20 START
 * 0x40 ENABLE
 */
I2C_Base::mStateMachineStatus_t I2C_Base::i2cStateMachine()
{
    enum {
        // General states :
        busError        = 0x00,
        start           = 0x08,
        repeatStart     = 0x10,
        arbitrationLost = 0x38,

        // Master Transmitter States:
        slaveAddressAcked  = 0x18,
        slaveAddressNacked = 0x20,
        dataAckedBySlave   = 0x28,
        dataNackedBySlave  = 0x30,

        // Master Receiver States:
        readAckedBySlave      = 0x40,
        readModeNackedBySlave = 0x48,
        dataAvailableAckSent  = 0x50,
        dataAvailableNackSent = 0x58,

        // Slave Transmitter States:
        writeAddressAcked           = 0xA8,
        dataTransmittedAcked        = 0xB8,
        dataTransmittedNacked       = 0xC0,
        lastDataTransmittedAcked    = 0xC8,

        // Slave Receiver States:
        slaveWriteReceivedAcked         = 0x60,
        dataReceivedAckSent             = 0x80,
        dataReceivedNackSent            = 0x88,
        stopOrRepeatedStart             = 0xA0,
};

    mStateMachineStatus_t state = busy;

    /*
     ***********************************************************************************************************
     * Write-mode state transition :
     * start --> slaveAddressAcked --> dataAckedBySlave --> ... (dataAckedBySlave) --> (stop)
     *
     * Read-mode state transition :
     * start --> slaveAddressAcked --> dataAcked --> repeatStart --> readAckedBySlave
     *  For 2+ bytes:  dataAvailableAckSent --> ... (dataAvailableAckSent) --> dataAvailableNackSent --> (stop)
     *  For 1  byte :  dataAvailableNackSent --> (stop)
     ***********************************************************************************************************
     */

    /* Me being lazy and using #defines instead of inline functions :( */
    #define clearSIFlag()       mpI2CRegs->I2CONCLR = (1<<3)
    #define setSTARTFlag()      mpI2CRegs->I2CONSET = (1<<5)
    #define clearSTARTFlag()    mpI2CRegs->I2CONCLR = (1<<5)
    #define setAckFlag()        mpI2CRegs->I2CONSET = (1<<2)
    #define setNackFlag()       mpI2CRegs->I2CONCLR = (1<<2)

    /* yep ... lazy again */
    #define setStop()           clearSTARTFlag();                           \
                                mpI2CRegs->I2CONSET = (1<<4);               \
                                clearSIFlag();                              \
                                while((mpI2CRegs->I2CONSET&(1<<4)));        \
                                if(I2C_READ_MODE(mTransaction.slaveAddr))   \
                                    state = readComplete;                   \
                                else                                        \
                                    state = writeComplete;

    switch (mpI2CRegs->I2STAT)
    {
        case slaveWriteReceivedAcked:
            setAckFlag();                                               // Set ack flag
            clearSIFlag();                                              // Clear SI flag
            i = 0, j = 0;
            status = false;
            break;
        case dataReceivedAckSent:
            if(i == 0)              // Condition to check for the first byte in a transaction
            {
                if((mpI2CRegs->I2DAT > 0) && (mpI2CRegs->I2DAT <= 0xf))     // Check if the first byte is a valid mailbox number
                {
                    config_register[i] = mpI2CRegs->I2DAT;
                    interrupted_mailbox = mpI2CRegs->I2DAT;
                    i++;
                }
                else if(mpI2CRegs->I2DAT == 0xfe)       // Check if the first byte is address of Status Register
                {
                    status = true;
                    i++;
                }
                else if(mpI2CRegs->I2DAT == 0xff)       // Check if the first byte is address of Data Register
                {
                    j = 1;
                    i++;
                }
                else                                    // Error handling
                    u0_dbg_printf("Wrong Register Address\n");
            }
            else if(!!j)                                // Condition which stores the data to be sent over CAN to a buffer
            {
                i2c_mem[j-1] = mpI2CRegs->I2DAT;
                j++;
            }
            else                                        // Condition which stores the configuration data of mailboxes to a buffer
            {
                config_register[i] = mpI2CRegs->I2DAT;
                i++;
            }
            if(i == 2)
                config = true;
            clearSIFlag();                                              // Clear SI flag
            break;
        case dataReceivedNackSent:
            setAckFlag();                                               // Set ack flag
            clearSIFlag();                                              // Clear SI flag
            break;
        case stopOrRepeatedStart:
            if(j)                                       // Give a semaphore if data is to be sent over CAN bus
                if(xSemaphoreGiveFromISR(i2c_receive, NULL))
                    u0_dbg_printf("Semaphore Given");
            if(config)                                  // Conditional loop which configures the mailbox according to data sent by I2C Master
            {
                uint8_t mailbox_number = config_register[0];
                mailbox[mailbox_number].mask = config_register[1] >> 7;
                mailbox[mailbox_number].size = config_register[1] >> 3;
                mailbox[mailbox_number].id = config_register[1];
                mailbox[mailbox_number].id <<= 8;
                mailbox[mailbox_number].id |= config_register[2];
                mailbox[mailbox_number].mask_id = config_register[3];
                mailbox[mailbox_number].mask_id <<= 8;
                mailbox[mailbox_number].mask_id |= config_register[4];
                u0_dbg_printf("****** Configure ******\nNum: %02d, Size: %02d, ID: %03x, Mask: %d, Mask_Id: %03x\n\n",
                        mailbox_number, mailbox[mailbox_number].size, mailbox[mailbox_number].id,
                        mailbox[mailbox_number].mask, mailbox[mailbox_number].mask_id);
                config = false;
                for(int k = 0; k < 5; k++)
                    config_register[k] = 0;
            }
            u0_dbg_printf("\n");
            i = 0, j = 0;
            clearSIFlag();                                              // Clear SI flag
            break;
        case writeAddressAcked:
            if(status)                      // Send Status Register data
                mpI2CRegs->I2DAT = (uint8_t)status_register;
            else                            // Send data stored in mailbox
                mpI2CRegs->I2DAT = mailbox[interrupted_mailbox].data[j];
            j++;
            clearSIFlag();                                              // Clear SI flag
            break;
        case dataTransmittedAcked:
            if(status)                      // Send Status Register data
                mpI2CRegs->I2DAT = (uint8_t)(status_register >> 8);
            else                            // Send data stored in mailbox
                mpI2CRegs->I2DAT = mailbox[interrupted_mailbox].data[j];
            j++;
            clearSIFlag();                                              // Clear SI flag
            break;
        case dataTransmittedNacked:
            mpI2CRegs->I2DAT;
            if(!config && !status)
            {
                u0_dbg_printf("Semaphore Given to allow interrupts\n");
                status_register &= ~(1 << interrupted_mailbox);         // Reset the bit in Status Register once the data is sent to I2C Master
                mailbox[interrupted_mailbox].num = 0;
            }
            clearSIFlag();                                              // Clear SI flag
            break;
        case lastDataTransmittedAcked:
            clearSIFlag();                                              // Clear SI flag
            break;

        case start:
            mpI2CRegs->I2DAT = I2C_WRITE_ADDR(mTransaction.slaveAddr);
            clearSIFlag();
            break;
        case repeatStart:
            mpI2CRegs->I2DAT = I2C_READ_ADDR(mTransaction.slaveAddr);
            clearSIFlag();
            break;

        case slaveAddressAcked:
            clearSTARTFlag();
            // No data to transfer, this is used just to test if the slave responds
            if(0 == mTransaction.trxSize) {
                setStop();
            }
            else {
                mpI2CRegs->I2DAT = mTransaction.firstReg;
                clearSIFlag();
            }
            break;

        case dataAckedBySlave:
            if (I2C_READ_MODE(mTransaction.slaveAddr)) {
                setSTARTFlag(); // Send Repeat-start for read-mode
                clearSIFlag();
            }
            else {
                if(0 == mTransaction.trxSize) {
                    setStop();
                }
                else {
                    mpI2CRegs->I2DAT = *(mTransaction.pMasterData);
                    ++mTransaction.pMasterData;
                    --mTransaction.trxSize;
                    clearSIFlag();
                }
            }
            break;

        /* In this state, we are about to initiate the transfer of data from slave to us
         * so we are just setting the ACK or NACK that we'll do AFTER the byte is received.
         */
        case readAckedBySlave:
            clearSTARTFlag();
            if(mTransaction.trxSize > 1) {
                setAckFlag();  // 1+ bytes: Send ACK to receive a byte and transition to dataAvailableAckSent
            }
            else {
                setNackFlag();  //  1 byte : NACK next byte to go to dataAvailableNackSent for 1-byte read.
            }
            clearSIFlag();
            break;
        case dataAvailableAckSent:
            *mTransaction.pMasterData = mpI2CRegs->I2DAT;
            ++mTransaction.pMasterData;
            --mTransaction.trxSize;

            if(1 == mTransaction.trxSize) { // Only 1 more byte remaining
                setNackFlag();// NACK next byte --> Next state: dataAvailableNackSent
            }
            else {
                setAckFlag(); // ACK next byte --> Next state: dataAvailableAckSent(back to this state)
            }

            clearSIFlag();
            break;
        case dataAvailableNackSent: // Read last-byte from Slave
            *mTransaction.pMasterData = mpI2CRegs->I2DAT;
            setStop();
            break;

        case arbitrationLost:
            // We should not issue stop() in this condition, but we still need to end our  transaction.
            state = I2C_READ_MODE(mTransaction.slaveAddr) ? readComplete : writeComplete;
            mTransaction.error = mpI2CRegs->I2STAT;
            break;

        case slaveAddressNacked:    // no break
        case dataNackedBySlave:     // no break
        case readModeNackedBySlave: // no break
        case busError:              // no break
        default:
            mTransaction.error = mpI2CRegs->I2STAT;
            setStop();
            break;
    }

    return state;
}
