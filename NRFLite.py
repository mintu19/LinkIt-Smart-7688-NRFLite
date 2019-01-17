from nRF24L01 import *
import mraa
import time
import monotonic
from SPI import S_SPI

# SS_PIN = 6 # For normal SPI
SS_PIN = 14 # For software spi

# SPI = mraa.Spi(0)
SPI = S_SPI()
# SPI.lsbmode(False)
SS = mraa.Gpio(SS_PIN)
SS.dir(mraa.DIR_IN)

class NRFLite:

    def __init__(self):
        # Delay used to discharge the radio's CSN pin when operating in 2-pin mode.
        # Determined by measuring time to discharge CSN on a 1MHz ATtiny using 0.1uF capacitor and 1K resistor.
        self._CSN_DISCHARGE_MICROS = 500.0

        self._OFF_TO_POWERDOWN_MILLIS = 100.0 # Vcc > 1.9V power on reset time.
        self._POWERDOWN_TO_RXTX_MODE_MICROS = 4630.0 # 4500 to Standby + 130 to RX or TX mode.
        self._CE_TRANSMISSION_MICROS = 10.0 # Time to initiate data transmission.

        self._momi_PORT = None
        self._momi_DDR = None
        self._momi_PIN = None
        self._sck_PORT = None
        self._cePin = None
        self._csnPin = None
        self._momi_MASK = None
        self._sck_MASK = None
        self._resetInterruptFlags = None
        self._useTwoPinSpiTransfer = None
        self._transmissionRetryWaitMicros = 0
        self._maxHasDataIntervalMicros = 0
        self._microsSinceLastDataCheck = None

    class BitRates:
        BITRATE2MBPS=0
        BITRATE1MBPS=1
        BITRATE250KBPS=2

    class SendType:
        REQUIRE_ACK = 0
        NO_ACK = 1

    # Methods for receivers and transmitters.
    # initSimple = Turns the radio on and puts it into receiving mode.  Returns 0 if it cannot communicate with the radio.
    #              Channel can be 0-125 and sets the exact frequency of the radio between 2400 - 2525 MHz.
    # readData   = Loads a received data packet or acknowledgment packet into the specified data parameter.
    # powerDown  = Power down the radio.  It only draws 900 nA in this state.  Turn the radio back on by calling one of the 
    #              'hasData' or 'send' methods.
    # printDetails = For debugging, it prints most radio registers if a serial object is provided in the constructor.

    def initSimple(self, radioId, cePin, csnPin, bitrate = BitRates.BITRATE2MBPS, channel = 100):
        self._useTwoPinSpiTransfer = 0
        self._cePin = mraa.Gpio(cePin)
        if cePin == csnPin:
            self._csnPin = self._cePin
        else:
            self._csnPin = mraa.Gpio(csnPin)

        self._cePin.dir(mraa.DIR_OUT)
        self._csnPin.dir(mraa.DIR_OUT)
        self._cePin.write(1)

        savedSS = SS.read()
        # SPI.begin()
        if (csnPin != SS_PIN):
            SS.write(savedSS)

        return self._prepForRx(radioId, bitrate, channel)

    def readData(self):
        # Determine length of data in the RX FIFO buffer and read it.
        dataLength = 0
        dataLength = self._spiTransfer(self._SpiTransferType.READ_OPERATION, R_RX_PL_WID, [dataLength], 1)[0]
        data = "\0"*dataLength
        data = self._spiTransfer(self._SpiTransferType.READ_OPERATION, R_RX_PAYLOAD, data, dataLength)
        
        # Clear data received flag.
        statusReg = self._readRegisterData(STATUS_NRF)
        if (statusReg & (1 << RX_DR)):
            self._writeRegisterData(STATUS_NRF, statusReg | (1 << RX_DR))
        
        return data

    def powerDown(self):
        # If we have separate CE and CSN pins, we can gracefully stop listening or transmitting.
        if (self._cePin != self._csnPin):
            self._cePin.write(0)
        
        # Turn off the radio.  Only consumes around 900 nA in this state!
        self._writeRegisterData(CONFIG, self._readRegisterData(CONFIG) & ~(1 << PWR_UP))

    # Methods for transmitters.
    # send = Sends a data packet and waits for success or failure.  The default REQUIRE_ACK sendType causes the radio
    #        to attempt sending the packet up to 16 times.  If successful a 1 is returned.  Optionally the NO_ACK sendType
    #        can be used to transmit the packet a single time without any acknowledgement.
    # hasAckData = Checks to see if an ACK data packet was received and returns its length.

    def send(self, toRadioId, data, length, sendType = SendType.REQUIRE_ACK):
        self._prepForTx(toRadioId, sendType)

        # Clear any previously asserted TX success or max retries flags.
        statusReg = self._readRegisterData(STATUS_NRF)
        if (statusReg & (1 << TX_DS)) or (statusReg & (1 << MAX_RT)):
            self._writeRegisterData(STATUS_NRF, statusReg | (1 << TX_DS) | (1 << MAX_RT))
        
        # Add data to the TX FIFO buffer, with or without an ACK request.
        if (sendType == self.SendType.NO_ACK):
            self._spiTransfer(self._SpiTransferType.WRITE_OPERATION, W_TX_PAYLOAD_NO_ACK, data, length)
        else:
            self._spiTransfer(self._SpiTransferType.WRITE_OPERATION, W_TX_PAYLOAD, data, length)

        # Start transmission.
        # If we have separate pins for CE and CSN, CE will be LOW and we must pulse it to start transmission.
        # If we use the same pin for CE and CSN, CE will already be HIGH and transmission will have started
        # when data was loaded into the TX FIFO.
        if (self._cePin != self._csnPin):
            self._cePin.write(1)
            time.sleep(self._CE_TRANSMISSION_MICROS/1000000)
            self._cePin.write(0)

        # Wait for transmission to succeed or fail.
        while True:
            time.sleep(self._transmissionRetryWaitMicros/1000000)
            statusReg = self._readRegisterData(STATUS_NRF)
            
            if (statusReg & (1 << TX_DS)):
                self._writeRegisterData(STATUS_NRF, statusReg | (1 << TX_DS))  # Clear TX success flag.
                return True                                                    # Return success.
            elif (statusReg & (1 << MAX_RT)):
                self._spiTransfer(self._SpiTransferType.WRITE_OPERATION, FLUSH_TX, None, 0)    # Clear TX FIFO buffer.
                self._writeRegisterData(STATUS_NRF, statusReg | (1 << MAX_RT))  # Clear flag which indicates max retries has been reached.
                return False                                       # Return failure.

    def hasAckData(self):
        # If we have a pipe 0 packet sitting at the top of the RX FIFO buffer, we have auto-acknowledgment data.
        # We receive ACK data from other radios using the pipe 0 address.
        if (self._getPipeOfFirstRxPacket() == 0):
            return self._getRxPacketLength() # Return the length of the data packet in the RX FIFO buffer.
        else:
            return 0

    # Methods for receivers.
    # hasData    = Checks to see if a data packet has been received and returns its length.
    # addAckData = Enqueues an acknowledgment packet for sending back to a transmitter.  Whenever the transmitter sends the 
    #              next data packet, it will get this ACK packet back in the response.  The radio will store up to 3 ACK packets
    #              and will not enqueue more if full, so you can clear any stale packets using the 'removeExistingAcks' parameter.
    def hasData(self, usingInterrupts = 0):
        # If using the same pins for CE and CSN, we need to ensure CE is left HIGH long enough to receive data.
        # If we don't limit the calling program, CE may mainly be LOW and the radio won't get a chance
        # to receive packets.  However, if the calling program is using an interrupt handler and only calling
        # hasData when the data received flag is set, we should skip this check since we know the calling program
        # is not continually polling hasData.  So 'usingInterrupts' = 1 bypasses the logic.
        if (self._cePin == self._csnPin and not(usingInterrupts)):
            if (monotonic.monotonic()*1000000 - self._microsSinceLastDataCheck < self._maxHasDataIntervalMicros):
                return 0 # Prevent the calling program from forcing us to bring CE low, making the radio stop receiving.
            else:
                self._microsSinceLastDataCheck = monotonic.monotonic()*1000000
        
        # Ensure radio is powered on and in RX mode in case the radio was powered down or in TX mode.
        originalConfigReg = self._readRegisterData(CONFIG)
        newConfigReg = originalConfigReg | (1 << PWR_UP) | (1 << PRIM_RX)
        if (originalConfigReg != newConfigReg):
            self._writeRegisterData(CONFIG, newConfigReg)
        
        # Ensure we're listening for packets by setting CE HIGH.  If we share the same pin for CE and CSN,
        # it will already be HIGH since we always keep CSN HIGH to prevent the radio from listening to the SPI bus.
        if (self._cePin != self._csnPin):
            if self._cePin.read():
                pass
            else:
                self._cePin.write(1)
        
        # If the radio was initially powered off, wait for it to turn on.
        if ((originalConfigReg & (1 << PWR_UP)) == 0):
            time.sleep(self._POWERDOWN_TO_RXTX_MODE_MICROS/1000000)

        # If we have a pipe 1 packet sitting at the top of the RX FIFO buffer, we have data.
        # We listen for data from other radios using the pipe 1 address.
        if (self._getPipeOfFirstRxPacket() == 1):
            return self._getRxPacketLength() # Return the length of the data packet in the RX FIFO buffer.
        else:
            return 0

    def addAckData(self, data, length, removeExistingAcks = 0):
        if (removeExistingAcks):
            self._spiTransfer(self._SpiTransferType.WRITE_OPERATION, FLUSH_TX, None, 0) # Clear the TX FIFO buffer.
        
        # Add the packet to the TX FIFO buffer for pipe 1, the pipe used to receive packets from radios that
        # send us data.  When we receive the next transmission from a radio, we'll provide this ACK data in the
        # auto-acknowledgment packet that goes back.
        self._spiTransfer(self._SpiTransferType.WRITE_OPERATION, (W_ACK_PAYLOAD | 1), data, length)

    # Methods when using the radio's IRQ pin for interrupts.
    # Note that if interrupts are used, do not use the send and hasData functions.  Instead the functions below must be used.
    # startSend    = Start sending a data packet without waiting for it to complete.
    # whatHappened = Use this inside the interrupt handler to see what caused the interrupt.
    # hasDataISR   = Same as hasData(1) and is just for clarity.  It will greatly speed up the receive bitrate when
    #                CE and CSN share the same pin.
    def startSend(self, toRadioId, data, length, sendType = SendType.REQUIRE_ACK):
        self._prepForTx(toRadioId, sendType)
    
        # Add data to the TX FIFO buffer, with or without an ACK request.
        if (sendType == self.SendType.NO_ACK):
             self._spiTransfer(self._SpiTransferType.WRITE_OPERATION, W_TX_PAYLOAD_NO_ACK, data, length)
        else:
            self._spiTransfer(self._SpiTransferType.WRITE_OPERATION, W_TX_PAYLOAD, data, length)
        
        # Start transmission.
        if (self._cePin != self._csnPin):
            self._cePin.write(1)
            time.sleep(self._CE_TRANSMISSION_MICROS/1000000)
            self._cePin.write(0)

    def whatHappened(self, txOk, txFail, rxReady):
        statusReg = self._readRegisterData(STATUS_NRF)
    
        txOk = statusReg & (1 << TX_DS)
        txFail = statusReg & (1 << MAX_RT)
        rxReady = statusReg & (1 << RX_DR)
        
        # When we need to see interrupt flags, we disable the logic here which clears them.
        # Programs that have an interrupt handler for the radio's IRQ pin will use 'whatHappened'
        # and if we don't disable this logic, it's not possible for us to check these flags.
        if (self._resetInterruptFlags):
            self._writeRegisterData(STATUS_NRF, statusReg | (1 << TX_DS) | (1 << MAX_RT) | (1 << RX_DR))

    def hasDataISR(self):
        # This method can be used inside an interrupt handler for the radio's IRQ pin to bypass
        # the limit on how often the radio can be checked for data.  This optimization greatly increases
        # the receiving bitrate when CE and CSN share the same pin.
        return self.hasData(1) # usingInterrupts = 1

    # Privates

    class _SpiTransferType:
        READ_OPERATION = 0
        WRITE_OPERATION = 1
    
    def _getPipeOfFirstRxPacket(self):
        # The pipe number is bits 3, 2, and 1.  So B1110 masks them and we shift right by 1 to get the pipe number.
        # 000-101 = Data Pipe Number
        #     110 = Not Used
        #     111 = RX FIFO Empty
        return (self._readRegisterData(STATUS_NRF) & 0B1110) >> 1

    def _getRxPacketLength(self):
        # Read the length of the first data packet sitting in the RX FIFO buffer.
        dataLength = 0
        dataLength = self._spiTransfer(self._SpiTransferType.READ_OPERATION, R_RX_PL_WID, [dataLength], 1)[0]

        # Verify the data length is valid (0 - 32 bytes).
        if (dataLength > 32):
            self._spiTransfer(self._SpiTransferType.WRITE_OPERATION, FLUSH_RX, None, 0) # Clear invalid data in the RX FIFO buffer.
            self._writeRegisterData(STATUS_NRF, self._readRegisterData(STATUS_NRF) | (1 << TX_DS) | (1 << MAX_RT) | (1 << RX_DR))
            return 0
        else:
            return dataLength

    def _prepForRx(self, radioId, bitrate, channel):
        self._resetInterruptFlags = 1
        time.sleep(self._OFF_TO_POWERDOWN_MILLIS/1000)
        if channel > 125:
            channel = 125
        
        self._writeRegisterData(RF_CH, channel)

        # Transmission speed, retry times, and output power setup.
        # For 2 Mbps or 1 Mbps operation, a 500 uS retry time is necessary to support the max ACK packet size.
        # For 250 Kbps operation, a 1500 uS retry time is necessary.
        # '_allowedDataCheckIntervalMicros' is used to limit how often the radio can be checked to determine if data
        # has been received when CE and CSN share the same pin.  If we don't limit how often the radio is checked,
        # the radio may never be given the chance to receive a packet.  More info about this in the 'hasData' method.
        # '_allowedDataCheckIntervalMicros' was determined by maximizing the transfer bitrate between two 16 MHz ATmega328's
        # using 32 byte payloads and sending back 32 byte ACK packets.

        if (bitrate == self.BitRates.BITRATE2MBPS):
            self._writeRegisterData(RF_SETUP, 0B00001110)   # 2 Mbps, 0 dBm output power
            self._writeRegisterData(SETUP_RETR, 0B00011111) # 0001 =  500 uS between retries, 1111 = 15 retries
            self._maxHasDataIntervalMicros = 600   
            self._transmissionRetryWaitMicros = 250
        elif (bitrate == self.BitRates.BITRATE1MBPS):                                        
            self._writeRegisterData(RF_SETUP, 0B00000110)   # 1 Mbps, 0 dBm output power
            self._writeRegisterData(SETUP_RETR, 0B00011111) # 0001 =  500 uS between retries, 1111 = 15 retries
            self._maxHasDataIntervalMicros = 1200     
            _transmissionRetryWaitMicros = 1000  
        else:
            self._writeRegisterData(RF_SETUP, 0B00100110)   # 250 Kbps, 0 dBm output power
            self._writeRegisterData(SETUP_RETR, 0B01011111) # 0101 = 1500 uS between retries, 1111 = 15 retries
            self._maxHasDataIntervalMicros = 8000     
            self._transmissionRetryWaitMicros = 1500  

        # Assign this radio's address to RX pipe 1.  When another radio sends us data, this is the address
        # it will use.  We use RX pipe 1 to store our address since the address in RX pipe 0 is reserved
        # for use with auto-acknowledgment packets.
        address = [ 1, 2, 3, 4, radioId ]
        address = self._writeRegister(RX_ADDR_P1, address, 5)

        # Enable dynamically sized packets on the 2 RX pipes we use, 0 and 1.
        # RX pipe address 1 is used to for normal packets from radios that send us data.
        # RX pipe address 0 is used to for auto-acknowledgment packets from radios we transmit to.
        self._writeRegisterData(DYNPD, (1 << DPL_P0) | (1 << DPL_P1))

        # Enable dynamically sized payloads, ACK payloads, and TX support with or without an ACK request.
        self._writeRegisterData(FEATURE, (1 << EN_DPL) | (1 << EN_ACK_PAY) | (1 << EN_DYN_ACK))

        # Ensure RX FIFO and TX FIFO buffers are empty.  Each buffer can hold 3 packets.
        self._spiTransfer(self._SpiTransferType.WRITE_OPERATION, FLUSH_RX, None, 0)
        self._spiTransfer(self._SpiTransferType.WRITE_OPERATION, FLUSH_TX, None, 0)

        # Clear any interrupts.
        statusReg = self._readRegisterData(STATUS_NRF)
        self._writeRegisterData(STATUS_NRF, statusReg | (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT))

        # Power on the radio and start listening, delaying to allow startup to complete.
        newConfigReg = (1 << PWR_UP) | (1 << PRIM_RX) | (1 << EN_CRC)
        self._writeRegisterData(CONFIG, newConfigReg)
        self._cePin.write(1)
        time.sleep(self._POWERDOWN_TO_RXTX_MODE_MICROS/1000000)

        # Return success if the update we made to the CONFIG register was successful.
        return self._readRegisterData(CONFIG) == newConfigReg

    def _prepForTx(self, toRadioId, sendType):
        # TX pipe address sets the destination radio for the data.
        # RX pipe 0 is special and needs the same address in order to receive ACK packets from the destination radio.
        address = [ 1, 2, 3, 4, toRadioId ]
        address = self._writeRegister(TX_ADDR, address, 5)
        address = self._writeRegister(RX_ADDR_P0, address, 5)

        # Ensure radio is powered on and ready for TX operation.
        originalConfigReg = self._readRegisterData(CONFIG)
        newConfigReg = originalConfigReg & (~(1 << PRIM_RX)) | (1 << PWR_UP)

        if originalConfigReg != newConfigReg:
            if (originalConfigReg & (1 << PRIM_RX)) and (originalConfigReg & (1<< PWR_UP)):
                if self._cePin.read():
                    self._cePin.write(0)
            self._writeRegisterData(CONFIG, newConfigReg)
            time.sleep(self._POWERDOWN_TO_RXTX_MODE_MICROS/1000000)

        # If RX FIFO buffer is full and we require an ACK, clear it so we can receive the ACK response.
        fifoReg = self._readRegisterData(FIFO_STATUS)
        if (fifoReg & (1 << RX_FULL)) and (sendType == self.SendType.REQUIRE_ACK):
            self._spiTransfer(self._SpiTransferType.WRITE_OPERATION, FLUSH_RX, None, 0)

        # If TX FIFO buffer is full, we'll attempt to send all the packets it contains.
        if (fifoReg & (1 << FIFO_FULL)):
            # Disable interrupt flag reset logic in 'whatHappened' so we can react to the flags here.
            self._resetInterruptFlags = 0
            statusReg = None
            
            # While the TX FIFO buffer is not empty...
            while (not(fifoReg & (1 << TX_EMPTY))):
                # Try sending a packet.
                self._cePin.write(1)
                time.sleep(self._CE_TRANSMISSION_MICROS/1000000)
                self._cePin.write(0)
                
                time.sleep(self._transmissionRetryWaitMicros/1000000)
                statusReg = self._readRegisterData(STATUS_NRF)
                
                if (statusReg & (1 << TX_DS)):
                    self._writeRegisterData(STATUS_NRF, statusReg | (1 << TX_DS))   # Clear TX success flag.
                elif (statusReg & (1 << MAX_RT)):
                    self._spiTransfer(self._SpiTransferType.WRITE_OPERATION, FLUSH_TX, None, 0) # Clear TX FIFO buffer.
                    self._writeRegisterData(STATUS_NRF, statusReg | (1 << MAX_RT))  # Clear flag which indicates max retries has been reached.
                fifoReg = self._readRegisterData(FIFO_STATUS)

            self._resetInterruptFlags = 1

    def _readRegisterData(self, regName):
        return self._readRegister(regName, [0] ,1)[0]

    def _readRegister(self, regName, data, length):
        return self._spiTransfer(self._SpiTransferType.READ_OPERATION, (R_REGISTER | (REGISTER_MASK & regName)), data, length)

    def _writeRegisterData(self, regName, data):
        return self._writeRegister(regName, [data], 1)[0]

    def _writeRegister(self, regName, data, length):
        return self._spiTransfer(self._SpiTransferType.WRITE_OPERATION, (W_REGISTER | (REGISTER_MASK & regName)), data, length)

    def _spiTransfer(self, transferType, regName, data, length):
        self._csnPin.write(0)
        # print("Reg: ", hex(regName))
        SPI.writeByte(regName)
        for i in range(length):
            # print("Data: ", data[i])
            newData = SPI.writeByte(data[i])
            if transferType == self._SpiTransferType.READ_OPERATION :
                data[i] = newData
        self._csnPin.write(1)
        return data
