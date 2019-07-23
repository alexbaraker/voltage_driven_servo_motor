## voltage_driven_servo_motor

The purpose of this project is to design and create a system that could sample an analog voltage using an ADC, and drive a servo motor that would respond to the amplitude of that voltage by having it move like the needle of a traditional analog voltmeter. The system is implemented on two separate devices, which communicate with each other via a wired interface. On one end is the QNX system, which serves its purpose by reading voltage using the onboard ADC. On the other end of the system is the STM32 eval board, which serves its purpose by generating the PWM signals necessary to drive the servo motor. The interface between the two boards is a 2-wire serial interface that is modeled heavily after the standard I2C protocol.


## Analysis/Design

![](https://i.imgur.com/1ghAA0w.png)

**Fig 1:** Hardware diagram, showing how the hardware is connected together.

&nbsp;

![](https://i.imgur.com/sqQ1SUX.png)

**Fig 2:** Transmission timing diagram. The CLK line is free-running. The DAT line idles high. Transmission begins with a start pulse (low), followed by 8 bits of data (LSB first). After the final bit is sent, the DAT line is pulled low by the STM32 board to acknowledge reception of the data, then the DAT returns to idle state.

&nbsp;

![](https://i.imgur.com/fovzi5e.png)

**Fig 3:** QNX system state transmission diagram.

&nbsp;

![](https://i.imgur.com/MhCYObq.png)

**Fig 4:** STM32 board state transition diagram.

&nbsp;

![](https://i.imgur.com/GXAzDuq.png)

**Fig 5:** The voltage signals gets read by the ADC which is stored in a buffer ready to be sent over to the microcontroller. The microcontroller reads the data into the buffer, which gets copied over to data when the transmission complete. The data is then used to set the appropriate duty cycle to drive the servo.


## Operation

Approximately every 100ms, the QNX system samples the analog signal present at the Vin pin, and maps this voltage from the full-scale input range of the ADC (-5V to +5V) into a degree measurement (0 to ~150°), and stores the inverse of the result of this calculation in a buffer. The clock line is toggled with every iteration through the main loop, and is always running. Every other iteration through the main loop, the inverse of the LSB of the buffer is written to the DAT line. When in the Idle state, the buffer will have a value of 0, thus the DAT line will stay at 1. When a sample is taken with the ADC, the inverse of this sample, along with a trailing ‘1’, are written into the buffer. As this data is written onto the DAT line, the trailing ‘1’ becomes a ‘0’, which is the start pulse, and is used to indicate that the following 8 bits are legitimate data. The DAT line is updated on the falling edge of the CLK line, so that it is definitely stable on each rising edge, when it is sampled. After all 8 bits have been transmitted, the QNX system goes into the ACK state, where it is waiting for the acknowledgement from the STM32 board that data has been received. To do this, the DAT line is switched to an input on the QNX system so that its status can be monitored. When this line is pulled low, this is the acknowledgement that the data was received, and the QNX system returns to the Idle state, and starts driving the DAT line as an output again.

From the STM32 board’s side, at every rising edge of the CLK line, the input buffer is shifted to the left 1 bit, and the value of DAT line is written to the LSB of the input buffer. When the buffer’s value is 0x1FE, this means that the DAT line has been sitting in the idle state for the full length of a transmission, and has just received a start pulse. This means that the next 8 bits on the DAT line are the data being transmitted. After counting that 8 bits have been read on the DAT line, the STM board moves into the ACK state, where it switches the DAT line to an output, and writes a ‘0’ to it. This signifies to the QNX system that the data were successfully received, and then both boards return to the Idle state.


## Test Plan

Sent data value of 0b00101001 --> (bit reversal) --> 0x94 = 0d148 = 148°

Visible features: 
- Start pulse low before actual data; Beginning of data is separated from idle signal state by start pulse
- Signal is idle high after end of data; End of data visible when signal goes high

![](https://i.imgur.com/wrRfT8Z.png)

**Fig 6:** Transmission of the value 0x94, which tells the STM32 board to move the servo to 148°. Ch. 1 (Yellow) is the CLK line, ch. 2 (blue) is the DAT line. Note the start pulse (the first low bit on DAT following the idle period), and that data is sampled at each rising edge of the CLK.
