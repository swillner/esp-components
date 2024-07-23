#ifndef MAX3421E_INTERNAL_H
#define MAX3421E_INTERNAL_H

enum {
    REG_RCVFIFO = 1,  // Receive FIFO
    REG_SNDFIFO = 2,  // Send FIFO
    REG_SUDFIFO = 4,  // Set Up Data FIFO Register
    REG_RCVBC = 6,    // Receive FIFO Byte Count Register
    REG_SNDBC = 7,    // Send FIFO Byte Count Register
    REG_USBIRQ = 13,
    REG_USBIEN = 14,
    REG_USBCTL = 15,
    REG_CPUCTL = 16,
    REG_PINCTL = 17,
    REG_REVISION = 18,  // MAX3421E Revision Number
    REG_IOPINS1 = 20,
    REG_IOPINS2 = 21,
    REG_GPINIRQ = 22,
    REG_GPINIEN = 23,
    REG_GPINPOL = 24,
    REG_HIRQ = 25,
    REG_HIEN = 26,
    REG_MODE = 27,
    REG_PERADDR = 28,  // Peripheral Address to which packets are to be sent
    REG_HCTL = 29,
    REG_HXFR = 30,
    REG_HRSL = 31,
};

#define CMD_READ (0 << 1)
#define CMD_WRITE (1 << 1)

#define BIT_USBIRQ_VBUSIRQ (1 << 6)    // V_BUS Detect Interrupt Request
#define BIT_USBIRQ_NOVBUSIRQ (1 << 5)  // V_BUS Absence Interrupt Request
#define BIT_USBIRQ_OSCOKIRQ (1 << 0)   // Oscillator OK Interrupt Request
#define BIT_USBIEN_VBUSIE (1 << 6)     // V_BUS Detect Interrupt Enable
#define BIT_USBIEN_NOVBUSIE (1 << 5)   // V_BUS Absence Interrupt Enable
#define BIT_USBIEN_OSCOKIE (1 << 0)    // Oscillator OK Interrupt Enable

#define BIT_USBCTL_CHIPRES (1 << 5)  // Chip Reset
#define BIT_USBCTL_PWRDOWN (1 << 4)  // Power Down the MAX3421E

#define BIT_CPUCTL_PULSEWID1 (1 << 7)  // These two bits set the IRQ inactive time in edge mode
#define BIT_CPUCTL_PULSEWID0 (1 << 6)  // These two bits set the IRQ inactive time in edge mode
#define BIT_CPUCTL_IE (1 << 0)         // Enable the INT pin

#define BIT_PINCTL_FDUPSPI (1 << 4)   // Full-Duplex SPI port operation
#define BIT_PINCTL_INTLEVEL (1 << 3)  // Sets the INT output pin to level-active (1) or edge-active (0)
#define BIT_PINCTL_POSINT (1 << 2)    // Edge polarity of the edge-active INT pin
#define BIT_PINCTL_GPXB (1 << 1)      // Two bits, GPXB and GPXA, determine the output of the GPX pin
#define BIT_PINCTL_GPXA (1 << 0)      // Two bits, GPXB and GPXA, determine the output of the GPX pin

#define BIT_IOPINS1_GPIN3 (1 << 7)   // General-Purpose Input pins 0 through 7
#define BIT_IOPINS1_GPIN2 (1 << 6)   // General-Purpose Input pins 0 through 7
#define BIT_IOPINS1_GPIN1 (1 << 5)   // General-Purpose Input pins 0 through 7
#define BIT_IOPINS1_GPIN0 (1 << 4)   // General-Purpose Input pins 0 through 7
#define BIT_IOPINS1_GPOUT3 (1 << 3)  // General-Purpose Output pins 0 through 7
#define BIT_IOPINS1_GPOUT2 (1 << 2)  // General-Purpose Output pins 0 through 7
#define BIT_IOPINS1_GPOUT1 (1 << 1)  // General-Purpose Output pins 0 through 7
#define BIT_IOPINS1_GPOUT0 (1 << 0)  // General-Purpose Output pins 0 through 7

#define BIT_IOPINS2_GPIN7 (1 << 7)   // General-Purpose Input pins 0 through 7
#define BIT_IOPINS2_GPIN6 (1 << 6)   // General-Purpose Input pins 0 through 7
#define BIT_IOPINS2_GPIN5 (1 << 5)   // General-Purpose Input pins 0 through 7
#define BIT_IOPINS2_GPIN4 (1 << 4)   // General-Purpose Input pins 0 through 7
#define BIT_IOPINS2_GPOUT7 (1 << 3)  // General-Purpose Output pins 0 through 7
#define BIT_IOPINS2_GPOUT6 (1 << 2)  // General-Purpose Output pins 0 through 7
#define BIT_IOPINS2_GPOUT5 (1 << 1)  // General-Purpose Output pins 0 through 7
#define BIT_IOPINS2_GPOUT4 (1 << 0)  // General-Purpose Output pins 0 through 7

#define BIT_GPINIRQ_GPINIRQ7 (1 << 7)  // General-Purpose IN Interrupt Request 0 through 7
#define BIT_GPINIRQ_GPINIRQ6 (1 << 6)  // General-Purpose IN Interrupt Request 0 through 7
#define BIT_GPINIRQ_GPINIRQ5 (1 << 5)  // General-Purpose IN Interrupt Request 0 through 7
#define BIT_GPINIRQ_GPINIRQ4 (1 << 4)  // General-Purpose IN Interrupt Request 0 through 7
#define BIT_GPINIRQ_GPINIRQ3 (1 << 3)  // General-Purpose IN Interrupt Request 0 through 7
#define BIT_GPINIRQ_GPINIRQ2 (1 << 2)  // General-Purpose IN Interrupt Request 0 through 7
#define BIT_GPINIRQ_GPINIRQ1 (1 << 1)  // General-Purpose IN Interrupt Request 0 through 7
#define BIT_GPINIRQ_GPINIRQ0 (1 << 0)  // General-Purpose IN Interrupt Request 0 through 7

#define BIT_GPINIEN_GPINIEN7 (1 << 7)  // General-Purpose IN Interrupt Enable 0 through 7
#define BIT_GPINIEN_GPINIEN6 (1 << 6)  // General-Purpose IN Interrupt Enable 0 through 7
#define BIT_GPINIEN_GPINIEN5 (1 << 5)  // General-Purpose IN Interrupt Enable 0 through 7
#define BIT_GPINIEN_GPINIEN4 (1 << 4)  // General-Purpose IN Interrupt Enable 0 through 7
#define BIT_GPINIEN_GPINIEN3 (1 << 3)  // General-Purpose IN Interrupt Enable 0 through 7
#define BIT_GPINIEN_GPINIEN2 (1 << 2)  // General-Purpose IN Interrupt Enable 0 through 7
#define BIT_GPINIEN_GPINIEN1 (1 << 1)  // General-Purpose IN Interrupt Enable 0 through 7
#define BIT_GPINIEN_GPINIEN0 (1 << 0)  // General-Purpose IN Interrupt Enable 0 through 7

#define BIT_GPINPOL_GPINPOL7 (1 << 7)  // General-Purpose IN Interrupt Polarity 0 though 7
#define BIT_GPINPOL_GPINPOL6 (1 << 6)  // General-Purpose IN Interrupt Polarity 0 though 7
#define BIT_GPINPOL_GPINPOL5 (1 << 5)  // General-Purpose IN Interrupt Polarity 0 though 7
#define BIT_GPINPOL_GPINPOL4 (1 << 4)  // General-Purpose IN Interrupt Polarity 0 though 7
#define BIT_GPINPOL_GPINPOL3 (1 << 3)  // General-Purpose IN Interrupt Polarity 0 though 7
#define BIT_GPINPOL_GPINPOL2 (1 << 2)  // General-Purpose IN Interrupt Polarity 0 though 7
#define BIT_GPINPOL_GPINPOL1 (1 << 1)  // General-Purpose IN Interrupt Polarity 0 though 7
#define BIT_GPINPOL_GPINPOL0 (1 << 0)  // General-Purpose IN Interrupt Polarity 0 though 7

#define BIT_HIRQ_HXFRDNIRQ (1 << 7)    // Host Transfer Done Interrupt Request
#define BIT_HIRQ_FRAMEIRQ (1 << 6)     // Frame Generator Interrupt Request
#define BIT_HIRQ_CONDETIRQ (1 << 5)    // Peripheral Connect/Disconnect Interrupt Request
#define BIT_HIRQ_SUSDNIRQ (1 << 4)     // Suspend operation Done IRQ
#define BIT_HIRQ_SNDBAVIRQ (1 << 3)    // Send Buffer Available Interrupt Request
#define BIT_HIRQ_RCVDAVIRQ (1 << 2)    // Receive FIFO Data Available Interrupt Request
#define BIT_HIRQ_RWUIRQ (1 << 1)       // Remote Wakeup Interrupt Request
#define BIT_HIRQ_BUSEVENTIRQ (1 << 0)  // One of two bus events has occurred: Bus Reset (when BUSRST 1 -> 0) or Bus Resume (when BUSRSM 1 -> 0)

#define BIT_HIEN_HXFRDNIE (1 << 7)    // Host Transfer Done Interrupt Enable
#define BIT_HIEN_FRAMEIE (1 << 6)     // Frame Generator Interrupt Enable
#define BIT_HIEN_CONDETIE (1 << 5)    // Peripheral Connect/Disconnect Interrupt Enable
#define BIT_HIEN_SUSDNIE (1 << 4)     // Suspend operation Done IE
#define BIT_HIEN_SNDBAVIE (1 << 3)    // Send Buffer Available Interrupt Enable
#define BIT_HIEN_RCVDAVIE (1 << 2)    // Receive FIFO Data Available Interrupt Enable
#define BIT_HIEN_RWUIE (1 << 1)       // Remote Wakeup Interrupt Enable
#define BIT_HIEN_BUSEVENTIE (1 << 0)  // Enable the BUSEVENTIRQ

#define BIT_MODE_DPPULLDN (1 << 7)   // Connect internal 15kΩ resistors from D+ and D- to ground
#define BIT_MODE_DMPULLDN (1 << 6)   // Connect internal 15kΩ resistors from D+ and D- to ground
#define BIT_MODE_DELAYISO (1 << 5)   // Delay data transfer to an ISOCHRONOUS endpoint until the next frame (until after the next SOF packet is transmitted)
#define BIT_MODE_SEPIRQ (1 << 4)     // Provides the GPIN IRQS on a separate pin (GPX)
#define BIT_MODE_SOFKAENAB (1 << 3)  // Enable automatic generation of full-speed SOF packets or low-speed Keep-Alive pulses
#define BIT_MODE_HUBPRE (1 << 2)     // Send the PRE PID to a LS device operating through a USB hub
#define BIT_MODE_LOWSPEED (1 << 1)   // Sets the host for low-speed operation
#define BIT_MODE_HOST (1 << 0)       // MAX3421E host or peripheral operation

#define BIT_HCTL_SNDTOG1 (1 << 7)    // Set or clear the data toggle value for a data transfer
#define BIT_HCTL_SNDTOG0 (1 << 6)    // Set or clear the data toggle value for a data transfer
#define BIT_HCTL_RCVTOG1 (1 << 5)    // Set or clear the data toggle value for a data transfer
#define BIT_HCTL_RCVTOG0 (1 << 4)    // Set or clear the data toggle value for a data transfer
#define BIT_HCTL_SIGRSM (1 << 3)     // Signal a bus resume event
#define BIT_HCTL_SAMPLEBUS (1 << 2)  // Sample the bus
#define BIT_HCTL_FRMRST (1 << 1)     // Reset the SOF frame counter
#define BIT_HCTL_BUSRST (1 << 0)     // Issue a Bus Reset to a USB peripheral

#define BIT_HXFR_HS (1 << 7)      // The CPU writes this register to launch a host transfer
#define BIT_HXFR_ISO (1 << 6)     // The CPU writes this register to launch a host transfer
#define BIT_HXFR_OUTNIN (1 << 5)  // The CPU writes this register to launch a host transfer
#define BIT_HXFR_SETUP (1 << 4)   // The CPU writes this register to launch a host transfer
#define MASK_HXFR_EP 0b00001111   // The CPU writes this register to launch a host transfer

#define BIT_HRSL_JSTATUS (1 << 7)    // Sample the state of the USB bus: Indicate the state
#define BIT_HRSL_KSTATUS (1 << 6)    // Sample the state of the USB bus: Indicate the state
#define BIT_HRSL_SNDTOGRD (1 << 5)   // SNDTOGRD and RCVTOGRD indicate the resulting data toggle values for OUT and IN transfers, respectively
#define BIT_HRSL_RCVTOGRD (1 << 4)   // SNDTOGRD and RCVTOGRD indicate the resulting data toggle values for OUT and IN transfers, respectively
#define MASK_HRSL_HRSLT3 0b00001111  // These bits indicate the results of a host transfer: HRSLT[3:0] indicate the result code

#define BIT_MISO_HXFRDNIRQ (1 << 7)
#define BIT_MISO_FRAMEIRQ (1 << 6)
#define BIT_MISO_CONNIRQ (1 << 5)
#define BIT_MISO_SUSDNIRQ (1 << 4)
#define BIT_MISO_SNDBAVIRQ (1 << 3)
#define BIT_MISO_RCVDAVIRQ (1 << 2)
#define BIT_MISO_RSMREQIRQ (1 << 1)
#define BIT_MISO_BUSEVENIRQ (1 << 0)

enum {
    VAL_HXFR_SETUP = 0x10,
    VAL_HXFR_BULK_IN = 0x00,
    VAL_HXFR_BULK_OUT = 0x20,
    VAL_HXFR_HS_IN = 0x80,
    VAL_HXFR_HS_OUT = 0xA0,
    VAL_HXFR_ISO_IN = 0x40,
    VAL_HXFR_ISO_OUT = 0x60,
};

// HRSLT[3:0] Codes
enum max3421e_result_t {
    hrSUCCESS = 0x00,   // Successful Transfer
    hrBUSY = 0x01,      // SIE is busy, transfer pending /* not needed when using interrupts */
    hrBADREQ = 0x02,    // Bad value in HXFR reg
    hrUNDEF = 0x03,     // (reserved)
    hrNAK = 0x04,       // Peripheral returned NAK
    hrSTALL = 0x05,     // Perpheral returned STALL
    hrTOGERR = 0x06,    // Toggle error/ISO over-underrun
    hrWRONGPID = 0x07,  // Received the wrong PID
    hrBADBC = 0x08,     // Bad byte count
    hrPIDERR = 0x09,    // Receive PID is corrupted
    hrPKTERR = 0x0A,    // Packet error (stuff, EOP)
    hrCRCERR = 0x0B,    // CRC error
    hrKERR = 0x0C,      // K-state instead of response
    hrJERR = 0x0D,      // J-state instead of response
    hrTIMEOUT = 0x0E,   // Device did not respond in time
    hrBABBLE = 0x0F,    // Device talked too long
};

// see https://www.beyondlogic.org/usbnutshell/usb6.shtml#StandardDeviceRequests
#define USB_REQUEST_GET_STATUS 0x00
#define USB_REQUEST_CLEAR_FEATURE 0x01
#define USB_REQUEST_SET_FEATURE 0x03
#define USB_REQUEST_SET_ADDRESS 0x05
#define USB_REQUEST_GET_DESCRIPTOR 0x06
#define USB_REQUEST_SET_DESCRIPTOR 0x07
#define USB_REQUEST_GET_CONFIGURATION 0x08
#define USB_REQUEST_SET_CONFIGURATION 0x09

#define USB_DESCRIPTOR_TYPE_DEVICE 0x0100
#define USB_DESCRIPTOR_TYPE_CONFIGURATION 0x0200
#define USB_DESCRIPTOR_TYPE_STRING 0x0300

#endif
