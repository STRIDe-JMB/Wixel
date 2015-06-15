/* wireless_serial app:
 * This app allows you to connect two Wixels together to make a wireless,
 * bidirectional, lossless serial link.  
 * See description.txt or the Wixel User's Guide for more information.
 */

/*
 * TODO: To avoid damage, don't enable nDTR and nRTS outputs by default.
 * TODO: use LEDs to give feedback about sending/receiving bytes.
 * TODO: UART flow control.
 * TODO: Obey CDC-ACM Set Line Coding commands:
 *       In USB-RADIO mode, bauds 0-255 would correspond to radio channels.
 * TODO: shut down radio when we are in a different serial mode
 * TODO: make the heartbeat blinks on the Wixels be synchronized (will require
 *       major changes to the radio_link library)
 * TODO: turn on red LED or flash it if the Wixel is in a mode that requires USB
 *       but has not reached the USB Configured State (this avoids the problem of
 *       having 0 LEDs on when the Wixel is in USB-UART mode and self powered)
 */

/** Dependencies **************************************************************/
#include <wixel.h>

#include <usb.h>
#include <usb_com.h>

#include <radio_com.h>
#include <radio_link.h>

#include <uart0.h>
#include <uart1.h>


/** Parameters ****************************************************************/
#define SERIAL_MODE_AUTO        0
#define SERIAL_MODE_USB_RADIO   1
#define SERIAL_MODE_UART_RADIO  2
#define SERIAL_MODE_USB_UART    3

#define SERVO_FR	5
#define SERVO_FL	2
#define SERVO_MR	4
#define SERVO_ML	1
#define SERVO_RR	3
#define SERVO_RL	0

#define MOTOR_FR	0
#define MOTOR_FL	1
#define MOTOR_MR	2
#define MOTOR_ML	3
#define MOTOR_RR	4
#define MOTOR_RL	5

#define SERVO_SET_OTHER 1500
#define SERVO_SET_FR 1500
#define SERVO_SET_FL 1500
#define SERVO_SET_MR 1500
#define SERVO_SET_ML 1500
#define SERVO_SET_RR 1500
#define SERVO_SET_RL 1500
#define SERVO_OFFSET 0

#define LOOP_TIME 10


#define TICKS_PER_Hz 1516//.2524654832347140039447731755

#define MICROSEC_PER_DEG 11
#define FORWARD 1
#define BACKWARD -1

int32 CODE param_serial_mode = SERIAL_MODE_UART_RADIO;
int32 CODE param_baud_rate = 38400;

int32 CODE param_nDTR_pin = 10;
int32 CODE param_nRTS_pin = 11;
int32 CODE param_nDSR_pin = 12;
int32 CODE param_nCD_pin = 13;

int32 CODE param_DTR_pin = -1;
int32 CODE param_RTS_pin = -1;
int32 CODE param_DSR_pin = -1;
int32 CODE param_CD_pin = -1;
int32 CODE param_arduino_DTR_pin = 0;

uint8 mode = 0;
uint32 time = 0;
float SWEEP_ANGLE = 60;
float FREQUENCY_Hz = 1.0;


// Approximate number of milliseconds to disable UART's receiver for after a
// framing error is encountered.
// Valid values are 0-250.
// A value of 0 disables the feature (the UART's receiver will not be disabled).
// The actual number of milliseconds that the receiver is disabled for will be
// between param_framing_error_ms and param_framing_error_ms + 1.
int32 CODE param_framing_error_ms = 0;

/** Global Variables **********************************************************/

// This bit is 1 if the UART's receiver has been disabled due to a framing error.
// This bit should be equal to !U1CSR.RE, but we need this variable because we
// don't want to be reading U1CSR in the main loop, because reading it might
// cause the FE or ERR bits to be cleared and then the ISR
// would not receive notice of those errors.
BIT uartRxDisabled = 0;

uint8 DATA currentSerialMode;

BIT framingErrorActive = 0;

BIT errorOccurredRecently = 0;
uint8 lastErrorTime;

/** Functions *****************************************************************/

void updateLeds()
{
    static BIT dimYellowLed = 0;
    static uint16 lastRadioActivityTime;
    uint16 now;

    usbShowStatusWithGreenLed();

    now = (uint16)getMs();

    if (currentSerialMode == SERIAL_MODE_USB_UART)
    {
        // The radio is not being used, so turn off the yellow LED.
        LED_YELLOW(0);
    }
    else if (!radioLinkConnected())
    {
        // We have not connected to another device wirelessly yet, so do a
        // 50% blink with a period of 1024 ms.
        LED_YELLOW(now & 0x200 ? 1 : 0);
    }
    else
    {
        // We have connected.

        if ((now & 0x3FF) <= 20)
        {
            // Do a heartbeat every 1024ms for 21ms.
            LED_YELLOW(1);
        }
        else if (dimYellowLed)
        {
            static uint8 DATA count;
            count++;
            LED_YELLOW((count & 0x7)==0);
        }
        else
        {
            LED_YELLOW(0);
        }
    }

    if (radioLinkActivityOccurred)
    {
        radioLinkActivityOccurred = 0;
        dimYellowLed ^= 1;
        //dimYellowLed = 1;
        lastRadioActivityTime = now;
    }

    if ((uint16)(now - lastRadioActivityTime) > 32)
    {
        dimYellowLed = 0;
    }

    if ((uint8)(now - lastErrorTime) > 100)
    {
        errorOccurredRecently = 0;
    }

    LED_RED(errorOccurredRecently || uartRxDisabled);
}

/* Returns the logical values of the input control signal pins.
   Bit 0 is DSR.
   Bit 1 is CD. */
uint8 ioRxSignals()
{
    uint8 signals = 0;

    if ((param_CD_pin >= 0 && isPinHigh(param_CD_pin)) ||
            (param_nCD_pin >= 0 && !isPinHigh(param_nCD_pin)))
    {
        signals |= 2;
    }

    if ((param_DSR_pin >= 0 && isPinHigh(param_DSR_pin)) ||
            (param_nDSR_pin >= 0 && !isPinHigh(param_nDSR_pin)))
    {
        signals |= 1;
    }

    return signals;
}

/* Sets the logical values of the output control signal pins.
   This should be called frequently (not just when the values change).
   Bit 0 is DTR.
   Bit 1 is RTS. */
void ioTxSignals(uint8 signals)
{
    static uint8 nTrstPulseStartTime;
    static uint8 lastSignals;

    // Inverted outputs
    setDigitalOutput(param_nDTR_pin, (signals & ACM_CONTROL_LINE_DTR) ? 0 : 1);
    setDigitalOutput(param_nRTS_pin, (signals & ACM_CONTROL_LINE_RTS) ? 0 : 1);

    // Non-inverted outputs.
    setDigitalOutput(param_DTR_pin, (signals & ACM_CONTROL_LINE_DTR) ? 1 : 0);
    setDigitalOutput(param_RTS_pin, (signals & ACM_CONTROL_LINE_RTS) ? 1 : 0);

    // Arduino DTR pin.
    if (!(lastSignals & ACM_CONTROL_LINE_DTR) && (signals & ACM_CONTROL_LINE_DTR))
    {
        // We just made a falling edge on the nDTR line, so start a 1-2ms high pulse
        // on the nTRST line.
        setDigitalOutput(param_arduino_DTR_pin, HIGH);
        nTrstPulseStartTime = getMs();
    }
    else if ((uint8)(getMs() - nTrstPulseStartTime) >= 2)
    {
        setDigitalOutput(param_arduino_DTR_pin, LOW);
    }

    lastSignals = signals;
}

void errorOccurred()
{
    lastErrorTime = (uint8)getMs();
    errorOccurredRecently = 1;
}

void errorService()
{
    static uint8 lastRxLowTime;

    if (uart1RxBufferFullOccurred)
    {
        uart1RxBufferFullOccurred = 0;
        errorOccurred();
    }

    if (uart1RxFramingErrorOccurred)
    {
        uart1RxFramingErrorOccurred = 0;

        // A framing error occurred.
        framingErrorActive = 1;
        errorOccurred();

        if (param_framing_error_ms > 0)
        {
            // Disable the UART's receiver.
            U1CSR &= ~0x40;    // U1CSR.RE = 0.  Disables reception of bytes on the UART.
            uartRxDisabled = 1;
            lastRxLowTime = (uint8)getMs();  // Initialize lastRxLowTime even if the line isn't low right now.
        }
    }

    if (framingErrorActive)
    {
        if (!isPinHigh(17))
        {
            errorOccurred();
        }
        else
        {
            framingErrorActive = 0;
        }
    }

    if (uartRxDisabled)
    {
        if (!isPinHigh(17))
        {
            // The line is low.
            lastRxLowTime = (uint8)getMs();
        }
        else if ((uint8)(getMs() - lastRxLowTime) > param_framing_error_ms)
        {
            // The line has been high for long enough, so re-enable the receiver.
            U1CSR |= 0x40;
            uartRxDisabled = 0;
        }
    }
}

void updateSerialMode()
{
    if ((uint8)param_serial_mode > 0 && (uint8)param_serial_mode <= 3)
    {
        currentSerialMode = (uint8)param_serial_mode;
        return;
    }

    if (usbPowerPresent())
    {
        if (vinPowerPresent())
        {
            currentSerialMode = SERIAL_MODE_USB_UART;
        }
        else
        {
            currentSerialMode = SERIAL_MODE_USB_RADIO;
        }
    }
    else
    {
        currentSerialMode = SERIAL_MODE_UART_RADIO;
    }
}

void usbToRadioService()
{
    uint8 signals;

    // Data
    while(usbComRxAvailable() && radioComTxAvailable())
    {
        radioComTxSendByte(usbComRxReceiveByte());
    }

    while(radioComRxAvailable() && usbComTxAvailable())
    {
        usbComTxSendByte(radioComRxReceiveByte());
    }

    // Control Signals

    radioComTxControlSignals(usbComRxControlSignals() & 3);

    // Need to switch bits 0 and 1 so that DTR pairs up with DSR.
    signals = radioComRxControlSignals();
    usbComTxControlSignals( ((signals & 1) ? 2 : 0) | ((signals & 2) ? 1 : 0));
}

void uartToRadioService()
{
    // Data
    while(uart1RxAvailable() && radioComTxAvailable())
    {
        radioComTxSendByte(uart1RxReceiveByte());
    }

    while(radioComRxAvailable() && uart1TxAvailable())
    {
        uart1TxSendByte(radioComRxReceiveByte());
    }

    // Control Signals.
    ioTxSignals(radioComRxControlSignals());
    radioComTxControlSignals(ioRxSignals());
}

void usbToUartService()


{
    uint8 signals;

    // Data
    while(usbComRxAvailable() && uart1TxAvailable())
    {
        uart1TxSendByte(usbComRxReceiveByte());
    }

    while(uart1RxAvailable() && usbComTxAvailable())
    {
        usbComTxSendByte(uart1RxReceiveByte());
    }

    ioTxSignals(usbComRxControlSignals());

    // Need to switch bits 0 and 1 so that DTR pairs up with DSR.
    signals = ioRxSignals();
    usbComTxControlSignals( ((signals & 1) ? 2 : 0) | ((signals & 2) ? 1 : 0));

    // TODO: report framing, parity, and overrun errors to the USB host here
}

void sendMasterString(char *message)
{
	int ind;

	for (ind = 0; message[ind] != '\0'; ind++){
		radioComTxSendByte(message[ind]);
	}

	radioComTxSendByte(13);
	radioComTxSendByte(10);
}

//void sendUint32(uint32 num)
//{
//	radioComTxSendByte((num >> 24)&0xff);
//	radioComTxSendByte((num >> 16)&0xff);
//	radioComTxSendByte((num >> 8)&0xff);
//	radioComTxSendByte((num >> 0)&0xff);
//
//}


void initMotors()
{
	char ind;

	uint8 rbcNum;
	uint8 mtrNum;
	uint8 chksum;

	uint32 QPPS = 20000;
	uint32 KP 	= 0x003F0000;
	uint32 KI 	= 0x00190000;
	uint32 KD 	= 0x00690000;
//	uint32 KP 	= 0x00690000;
//	uint32 KI 	= 0x002A0000;
//	uint32 KD 	= 0x00AF0000;


	for(ind = MOTOR_FR; ind <= MOTOR_RL; ind++){

		rbcNum = ( ind >> 1 ) + 129;
		mtrNum = ind & 1;
		chksum = ( rbcNum
				+ ( mtrNum + 28 )
				+ ( KD >> 24 ) + ( KD >> 16 ) + ( KD >> 8 ) + KD
				+ ( KP >> 24 ) + ( KP >> 16 ) + ( KP >> 8 ) + KP
				+ ( KI >> 24 ) + ( KI >> 16 ) + ( KI >> 8 ) + KI
				+ ( QPPS >> 24 ) + ( QPPS >> 16 ) + ( QPPS >> 8 ) + QPPS )
				& 0x7F;

		// Set velocity PID constants

		uart1TxSendByte(rbcNum);
		uart1TxSendByte(mtrNum + 28);
		uart1TxSendByte(KD >> 24);
		uart1TxSendByte(KD >> 16);
		uart1TxSendByte(KD >> 8);
		uart1TxSendByte(KD);
		uart1TxSendByte(KP >> 24);
		uart1TxSendByte(KP >> 16);
		uart1TxSendByte(KP >> 8);
		uart1TxSendByte(KP);
		uart1TxSendByte(KI >> 24);
		uart1TxSendByte(KI >> 16);
		uart1TxSendByte(KI >> 8);
		uart1TxSendByte(KI);
		uart1TxSendByte(QPPS >> 24);
		uart1TxSendByte(QPPS >> 16);
		uart1TxSendByte(QPPS >> 8);
		uart1TxSendByte(QPPS);
		uart1TxSendByte(chksum);

//		radioComTxSendByte(rbcNum);
//		radioComTxSendByte(mtrNum + 28);
//		radioComTxSendByte(KD >> 24);
//		radioComTxSendByte(KD >> 16);
//		radioComTxSendByte(KD >> 8);
//		radioComTxSendByte(KD);
//		radioComTxSendByte(KP >> 24);
//		radioComTxSendByte(KP >> 16);
//		radioComTxSendByte(KP >> 8);
//		radioComTxSendByte(KP);
//		radioComTxSendByte(KI >> 24);
//		radioComTxSendByte(KI >> 16);
//		radioComTxSendByte(KI >> 8);
//		radioComTxSendByte(KI);
//		radioComTxSendByte(QPPS >> 24);
//		radioComTxSendByte(QPPS >> 16);
//		radioComTxSendByte(QPPS >> 8);
//		radioComTxSendByte(QPPS);
//		radioComTxSendByte(chksum);

		delayMs(5);

		// Set position PID constants

		// ***TO DO LATER***

		// Reset Decoders

		chksum = ( rbcNum + 20 ) & 0x7F;

		uart1TxSendByte(rbcNum);
		uart1TxSendByte(20);
		uart1TxSendByte(chksum);

		delayMs(5);

	}

	sendMasterString("Motor Initialization Complete...");
}

//void initServos()
//{
//	char ind;
//
//	for(ind = SERVO_FL; ind <= SERVO_RR; ind++){
//		uart0TxSendByte(128);
//		uart0TxSendByte(1);
//		uart0TxSendByte(0);
//		uart0TxSendByte(ind);
//		if((ind & 1) == 1){
//			uart0TxSendByte(84);
//		}
//		else{
//			uart0TxSendByte(116);
//		}
//		delayMs(5);
//	}
//
//	delayMs(2000);
//
//	sendMasterString("Servo Initialization Complete...");
//}

void setServoSpd(uint8 servoNum, uint16 servoSpd)
{
	uint8 spd1 = (servoSpd) & 0x7f;
	uint8 spd2 = ((servoSpd) >> 7) & 0x7f;

	uart0TxSendByte(170);
	uart0TxSendByte(12);
	uart0TxSendByte(7);
	uart0TxSendByte(servoNum);
	uart0TxSendByte(spd1);
	uart0TxSendByte(spd2);
}

void setServoPos_2(uint8 servoNum, uint16 servoPos)
{

	uint8 pos1 = (4*servoPos) & 0x7f;
	uint8 pos2 = ((4*servoPos) >> 7) & 0x7f;

	uart0TxSendByte(170);		//Servo CTR Address
	uart0TxSendByte(12);		//Device Number (12 Default)
	uart0TxSendByte(4);			//Command 4 Move to Position
	uart0TxSendByte(servoNum);
	uart0TxSendByte(pos1);
	uart0TxSendByte(pos2);

}

void sweepServo(uint8 servoNum, int dir)
{
	uint16 init_Pos = 1500;
	int16 Pos_change = 0;

	switch(servoNum){
	case SERVO_FR:
		init_Pos = SERVO_SET_FR;
		dir = -dir;
		break;
	case SERVO_FL:
		init_Pos = SERVO_SET_FL;
		break;
	case SERVO_MR:
		init_Pos = SERVO_SET_MR;
		dir = -dir;
		break;
	case SERVO_ML:
		init_Pos = SERVO_SET_ML;
		break;
	case SERVO_RR:
		init_Pos = SERVO_SET_RR;
		dir = -dir;
		break;
	case SERVO_RL:
		init_Pos = SERVO_SET_RL;
		break;
	default:
		break;
	}
	Pos_change = dir*(SWEEP_ANGLE/2.0)*MICROSEC_PER_DEG;
	//sendUint32(Pos_change);
	setServoPos_2(servoNum, (init_Pos+Pos_change));


}

void initServos()
{
	char ind = SERVO_RL;
	//Servo Speed set based on the angle traveled per half period
	//Units are ([DEG]*[1/sec])*([microsec]/[DEG])*([sec]/[millisec])
	//This gives [microsec/millisec]
	uint16 SERVO_SPEED = (int)(40*(2.5*SWEEP_ANGLE*MICROSEC_PER_DEG*FREQUENCY_Hz)/1000.0);

	//Set Speed Values
	//while(((ind%12) <= SERVO_FL) || ((ind%12)>=SERVO_RR)){
	while(ind <= SERVO_FR){
		setServoSpd((ind%12), SERVO_SPEED);
		ind++;
	}

	//ind = SERVO_RR;
	ind = SERVO_RL;
	while(ind <= SERVO_FR)
	{
		sweepServo((ind%12), 0);
		ind+=2;

	}
	ind = SERVO_ML;

	while(ind <= SERVO_FR)
	{
		sweepServo((ind%12), 0);
		ind+=2;
	}

//	//Set Initial Positions for all servos
//	while(((ind%12) <= SERVO_FL) || ((ind%12)>=SERVO_RR)){
//		if((ind%12) == SERVO_RL){
//			setServoPos_2((ind%12), SERVO_SET_RL);
//		}
//		else if((ind%12) == SERVO_RR){
//			setServoPos_2((ind%12), SERVO_SET_RR);
//		}
//		else{
//			setServoPos_2((ind%12), SERVO_SET_OTHER);
//		}
//		ind++;
//	}
//	ind = SERVO_RR;

	delayMs(2000);

	sendMasterString("Servo Initialization Complete...");
}



void setMotorPos()//uint8 motorNum, uint8 motorPos)
{
	// ***TO DO LATER***
}

void setMotorSpd(uint8 motorNum, uint32 motorSpd)
{
	uint8 rbcNum;
	uint8 mtrNum;
	uint8 chksum;

	rbcNum = ( motorNum >> 1 ) + 129;
	mtrNum = motorNum & 1;

	chksum = ( rbcNum
			+ ( mtrNum + 35 )
			+ ( motorSpd >> 24 ) + ( motorSpd >> 16 ) + ( motorSpd >> 8 ) + motorSpd )
			& 0x7F;

	uart1TxSendByte(rbcNum);
	uart1TxSendByte(mtrNum + 35);
	uart1TxSendByte(motorSpd >> 24);
	uart1TxSendByte(motorSpd >> 16);
	uart1TxSendByte(motorSpd >> 8);
	uart1TxSendByte(motorSpd);
	uart1TxSendByte(chksum);

//	radioComTxSendByte(rbcNum);
//	radioComTxSendByte(mtrNum + 35);
//	radioComTxSendByte(motorSpd >> 24);
//	radioComTxSendByte(motorSpd >> 16);
//	radioComTxSendByte(motorSpd >> 8);
//	radioComTxSendByte(motorSpd);
//	radioComTxSendByte(chksum);
}

void setServoPos(uint8 servoNum, uint8 servoPos)
{
	uart0TxSendByte(128);
	uart0TxSendByte(1);
	uart0TxSendByte(2);
	uart0TxSendByte(servoNum);
	uart0TxSendByte(servoPos);
}

void main()
{
	char ind;
	uint16 ctn = 0;
	//int ind_3 = 0;
	int8 direction = 1;
	uint32 time = 0;
//	uint32 max = 0;
//	//uint32 min = 0;

//	uint32 timeArray[10] = {0};


	systemInit();

    setDigitalOutput(param_arduino_DTR_pin, LOW);
    ioTxSignals(0);

    usbInit();

    uart0Init();
    uart0SetBaudRate(param_baud_rate);
    uart1Init();
    uart1SetBaudRate(param_baud_rate);

    timeInit();



    if (param_serial_mode != SERIAL_MODE_USB_UART)
    {
        radioComRxEnforceOrdering = 1;
        radioComInit();
    }

    // Set up P1_5 to be the radio's TX debug signal.
    //P1DIR |= (1<<5);
    //IOCFG0 = 0b011011; // P1_5 = PA_PD (TX mode)

    while(1)
    {
    	time = getMs();
        updateSerialMode();
        boardService();
        updateLeds();
        errorService();

        if (param_serial_mode != SERIAL_MODE_USB_UART)
        {
            radioComTxService();
        }

        usbComService();

        switch(currentSerialMode)
        {
        case SERIAL_MODE_USB_RADIO:  usbToRadioService();  break;
        case SERIAL_MODE_UART_RADIO: uartToRadioService(); break;
        case SERIAL_MODE_USB_UART:   usbToUartService();   break;
        }

//        while(radioComRxAvailable() && uart0TxAvailable() && uart1TxAvailable()){
//
//        	switch(radioComRxReceiveByte()){
//
//        	case 0:		// STOP
//        		// Stop motors
//        		for(ind = MOTOR_FR; ind <= MOTOR_RL; ind++){
//        			setMotorSpd(ind,0);
//        		}
//
//        		// Set servo speeds to zero
//        		mode = 0;
//        		break;
//        	case 1:		// Initialize
//        		// Initialize servos speed and initial pos
//        		initServos();
//
//        		// Initialize motors
//        		initMotors();
//
//        		// Set legs to starting positions
//
//        		// TO DO LATER (assume in correct position for now)
//
//        		sendMasterString("Robot Initialization Complete...");
//        		time = getMs();
//        		break;
//        	case 2:		// Walk (prismatic)
//        		// Set servo positions
//
//        		// Start legs at desired speed
//        		for(ind = MOTOR_FR; ind <= MOTOR_RL; ind++){
//        			setMotorSpd(ind,500);
//        		}
//        		sendMasterString("Sent Motor SPD ...");
//
//        		break;
//        	case 3:		// Walk (torsional)
//        		// Set servo positions
//        		ind = SERVO_RL;
//        		while(ind <= SERVO_FR)
//        		{
//        			sweepServo(ind, FORWARD);
//        			ind+=2;
//
//        		}
//        		ind = SERVO_ML;
//
//        		while(ind <= SERVO_FR)
//        		{
//        			sweepServo(ind, BACKWARD);
//        			ind+=2;
//        		}
//        		// Set leg positions
//
//        		// Set servo speeds
//        		sendMasterString("Sent SERVO Pos...");
//        		break;
//        	case 4:		// Walk (hybrid)
//        		// Sweep servo positions
//        		mode = 4;
//        		ctn = 0;
////        		ind = SERVO_RR;
////        		while(((ind%12) <= SERVO_FL) || ((ind%12)>=SERVO_RR)){
////        			sweepServo((ind%12), direction);
////        			ind+=2;
////        		}
////        		ind = SERVO_MR;
////        		while(((ind%12) <= SERVO_FL) || ((ind%12)>=SERVO_RR)){
////        			sweepServo((ind%12), -direction);
////        			ind+=2;
////        		}
////        		if((ctn%(50/FREQUENCY_Hz)) == 0)
////        		{
////        			direction = -direction;
////        		}
//
//        		// Set leg positions
//        		for(ind = MOTOR_FR; ind <= MOTOR_RL; ind++){
//        			setMotorSpd(ind,FREQUENCY_Hz*TICKS_PER_Hz);
//        		}
//        		// Set servo speeds
//        		sendMasterString("Sent SERVO Pos2...");
//
//
//        		// Set leg positions
//
//        		// Set servo speeds
//
//        		// Start legs at desired speed
//
//        		break;
////        	case 5:		// Send Values
////        		max = timeArray[0];
////        		//min = timeArray[0];
////
////        		for(ind = 0; ind < 10; ind++)
////        		{
////
////					if(timeArray[ind] > max)
////					{
////						max = timeArray[ind];
////					}
//////					if(timeArray[ind] < min)
//////					{
//////						min = timeArray[ind];
//////					}
////        		}
////        		sendUint32(max);
////        		//sendUint32(min);
////
////        		break;
//
//        	case 255:
//        		setServoPos(1,10);
//        		setServoPos(2,10);
//        		setServoPos(3,10);
//        		setServoPos(4,10);
//        		setServoPos(5,10);
//        		setServoPos(6,10);
//
//        		delayMs(500);
//
//        		setServoPos(1,100);
//        		setServoPos(2,100);
//        		setServoPos(3,100);
//        		setServoPos(4,100);
//        		setServoPos(5,100);
//        		setServoPos(6,100);
//
//            	uart1TxSendByte(129);
//				uart1TxSendByte(0);
//				uart1TxSendByte(25);
//				uart1TxSendByte(26);
//
//				delayMs(1000);
//
//            	uart1TxSendByte(129);
//            	uart1TxSendByte(0);
//            	uart1TxSendByte(0);
//            	uart1TxSendByte(1);
//            	break;
//        	}
//        }
//
//        switch(mode){
//
//        case 0:		// Stop
//        	break;
//        case 1:		// Initialize
//        	break;
//        case 2:		// Walk (prismatic)
//        	break;
//        case 3:		// Walk (torsional)
//        	// Determine if servo positions should be updated
//
//        	// Update servo positions (if necessary)
//
//        	break;
//        case 4:		// Walk (hybrid)
//        	// Determine if servo positions should be updated
//
//        	// Update servo positions (if necessary)
//
//        	break;
//        default:
//        	break;
//        }
//
//
//		if(((ctn%((int)(50/FREQUENCY_Hz))) == SERVO_OFFSET) && (mode == 4))
//		{
//			direction = -direction;
//			ind = SERVO_RL;
//			while(ind <= SERVO_FR)
//			{
//				sweepServo((ind%12), direction);
//				ind+=2;
//
//			}
//			ind = SERVO_ML;
//			while(ind <= SERVO_FR)
//			{
//				sweepServo((ind%12), -direction);
//				ind+=2;
//			}
//		}
//
//
//		// Control Signals.
//		ioTxSignals(radioComRxControlSignals());
//		radioComTxControlSignals(ioRxSignals());
//
//
////		timeArray[ind_3%10] = (getMs() - time);
////
////		if((ind_3 % 10) == 0){
////			for(ind = 0; ind < 10; ind++){
////				sendUint32(timeArray[ind]);
////			}
////		}
////		ind_3++;
//		//sendUint32(getMs() - time);
//		//sendUint32(time);
//		ctn++;
//		while((getMs() - time) < LOOP_TIME){
//		}

    }

}
