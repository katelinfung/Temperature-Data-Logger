/*
Temperature-Data-Logger.c
Date: March 5th 2021
Video: https://youtu.be/ryyEJxsVl8I


Features:
1. Displays temperature on LCD screen
2. Three LEDs turn on depending on current temperature
	a. If temperature < 15C :		 Blue LED turns on
	b. If 15C < temperature < 20C :  Green LED turns on
	c. If 20C < temperature :		 Red LED turns on
3. When Boot button is pressed, plays audio:
	a. If temperature < 15C, 		 "Cold"
	b. If 15C < temperature < 20C,   "Normal"
	c. If 20C < temperature, 	     "Hot"

A Python Script displays Temperature vs Time on a Stripchart
	
References:
	Lab4-data_logging_using_Python_2021_EFM8.pdf
	EFM8LB1_Receiver.c
	lcd.c
	Blinky.c
	Hello_World_SoC-8052.c
	ADC_EFM8.c
	Lecture Slides February 26 (Lab4-RS232-Temperature.pdf)
	serial_in.py
	stripchart_sinewave.py
*/

#include <stdio.h>
#include <stdlib.h>
#include <EFM8LB1.h>
#include <math.h>
#include <string.h>

// ~C51~  

#define SYSCLK 72000000L
#define BAUDRATE 115200L
#define SARCLK 18000000L
#define TIMER_2_FREQ 22050L  // Must match the frequency of the wav file store in external SPI flash
#define F_SCK_MAX 20000000L  // Max SPI SCK freq (Hz) 
#define CS P0_3

#define LCD_RS P2_0
#define LCD_RW P1_7
#define LCD_E P1_6
#define LCD_D4 P1_1
#define LCD_D5 P1_0
#define LCD_D6 P0_7
#define LCD_D7 P0_6
#define CHARS_PER_LINE 16

// Flash memory commands
#define WRITE_ENABLE     0x06  // Address:0 Dummy:0 Num:0 fMax: 25MHz
#define WRITE_DISABLE    0x04  // Address:0 Dummy:0 Num:0 fMax: 25MHz
#define READ_STATUS      0x05  // Address:0 Dummy:0 Num:1 to infinite fMax: 32MHz
#define READ_BYTES       0x03  // Address:3 Dummy:0 Num:1 to infinite fMax: 20MHz
#define READ_SILICON_ID  0xab  // Address:0 Dummy:3 Num:1 to infinite fMax: 32MHz
#define FAST_READ        0x0b  // Address:3 Dummy:1 Num:1 to infinite fMax: 40MHz
#define WRITE_STATUS     0x01  // Address:0 Dummy:0 Num:1 fMax: 25MHz
#define WRITE_BYTES      0x02  // Address:3 Dummy:0 Num:1 to 256 fMax: 25MHz
#define ERASE_ALL        0xc7  // Address:0 Dummy:0 Num:0 fMax: 25MHz
#define ERASE_BLOCK      0xd8  // Address:3 Dummy:0 Num:0 fMax: 25MHz
#define READ_DEVICE_ID   0x9f  // Address:0 Dummy:2 Num:1 to infinite fMax: 25MHz

// SPI Flash Memory connections:
// 	P0.0: SCK  connected to pin 6
// 	P0.1: MISO connected to pin 2
// 	P0.2: MOSI connected to pin 5
//  P0.3: CS*  connected to pin 1
//  3.3V: connected to pins 3, 7, and 8
//  GND:  connected to pin 4

volatile unsigned long int playcnt=0;

union long_bytes
{
    long l;
    unsigned char b[4];
};


char _c51_external_startup (void)
{
	// Disable Watchdog with key sequence
	SFRPAGE = 0x00;
	WDTCN = 0xDE; //First key
	WDTCN = 0xAD; //Second key
  
	VDM0CN=0x80;       // enable VDD monitor
	RSTSRC=0x02|0x04;  // Enable reset on missing clock detector and VDD

	#if (SYSCLK == 48000000L)	
		SFRPAGE = 0x10;
		PFE0CN  = 0x10; // SYSCLK < 50 MHz.
		SFRPAGE = 0x00;
	#elif (SYSCLK == 72000000L)
		SFRPAGE = 0x10;
		PFE0CN  = 0x20; // SYSCLK < 75 MHz.
		SFRPAGE = 0x00;
	#endif
	
	#if (SYSCLK == 12250000L)
		CLKSEL = 0x10;
		CLKSEL = 0x10;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 24500000L)
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 48000000L)	
		// Before setting clock to 48 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x07;
		CLKSEL = 0x07;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 72000000L)
		// Before setting clock to 72 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x03;
		CLKSEL = 0x03;
		while ((CLKSEL & 0x80) == 0);
	#else
		#error SYSCLK must be either 12250000L, 24500000L, 48000000L, or 72000000L
	#endif
	
	P0MDOUT |= 0x10; // Enable UART0 TX as push-pull output
	XBR0     = 0x01; // Enable UART0 on P0.4(TX) and P0.5(RX)                     
	XBR1     = 0X00;
	XBR2     = 0x40; // Enable crossbar and weak pull-ups

	// Configure Uart 0
	#if (((SYSCLK/BAUDRATE)/(2L*12L))>0xFFL)
		#error Timer 0 reload value is incorrect because (SYSCLK/BAUDRATE)/(2L*12L) > 0xFF
	#endif
	SCON0 = 0x10;
	TH1 = 0x100-((SYSCLK/BAUDRATE)/(2L*12L));
	TL1 = TH1;      // Init Timer1
	TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit auto-reload
	TMOD |=  0x20;                       
	TR1 = 1; // START Timer1
	TI = 1;  // Indicate TX0 ready
	
	
	// Configure SPI
	// Initialize the pin used by DAC0 (P3.0 in the LQFP32 package)
	// 1. Clear the bit associated with the pin in the PnMDIN register to 0. This selects analog mode for the pin.
    // 2. Set the bit associated with the pin in the Pn register to 1.
    // 3. Skip the bit associated with the pin in the PnSKIP register to ensure the crossbar does not attempt to assign a function to the pin.
	P3MDIN&=0b_1111_1110;
	P3|=0b_0000_0001;
	//P3SKIP|=0b_0000_0001; // P3 Pins Not Available on Crossbar
	
	P0MDOUT  = 0b_0001_1101;//SCK, MOSI, P0.3, TX0 are puspull, all others open-drain
	XBR0     = 0b_0000_0011;//SPI0E=1, URT0E=1
	XBR1     = 0X00;
	XBR2     = 0x40; // Enable crossbar and weak pull-ups

	#if ( ((SYSCLK/BAUDRATE)/(12L*2L)) > 0x100)
		#error Can not configure baudrate using timer 1 
	#endif
	// Configure Uart 0
	SCON0 = 0x10;
	TH1 = 0x100-((SYSCLK/BAUDRATE)/(12L*2L));
	TL1 = TH1;      // Init Timer1
	TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit auto-reload
	TMOD |=  0x20;                       
	TR1 = 1; // START Timer1
	TI = 1;  // Indicate TX0 ready
	
  	// Initialize DAC
	SFRPAGE = 0x30; 
  	DACGCF0=0b_1000_1000; // 1:D23REFSL(VCC) 1:D3AMEN(NORMAL) 2:D3SRC(DAC3H:DAC3L) 1:D01REFSL(VCC) 1:D1AMEN(NORMAL) 1:D1SRC(DAC1H:DAC1L)
  	DACGCF1=0b_0000_0000;
  	DACGCF2=0b_0010_0010; // Reference buffer gain 1/3 for all channels
  	DAC0CF0=0b_1000_0000; // Enable DAC 0
  	DAC0CF1=0b_0000_0010; // DAC gain is 3.  Therefore the overall gain is 1.
  	DAC0=0x80<<4;

	SFRPAGE = 0x00; 

	// SPI inititialization
	SPI0CKR = (SYSCLK/(2*F_SCK_MAX))-1;
	SPI0CFG = 0b_0100_0000; //SPI in master mode
	SPI0CN0 = 0b_0000_0001; //SPI enabled and in three wire mode
	CS=1;

	// Initialize timer 2 for periodic interrupts
	TMR2CN0=0x00;   // Stop Timer2; Clear TF2;
	CKCON0|=0b_0001_0000; // Timer 2 uses the system clock
	TMR2RL=(0x10000L-(SYSCLK/TIMER_2_FREQ)); // Initialize reload value
	TMR2=0xffff;   // Set to reload immediately
	ET2=1;         // Enable Timer2 interrupts
	TR2=1;         // Start Timer2 (TMR2CN is bit addressable)

	EA=1; // Enable interrupts
	
  	
	return 0;
}

void InitADC (void)
{
	SFRPAGE = 0x00;
	ADEN=0; // Disable ADC
	
	ADC0CN1=
		(0x2 << 6) | // 0x0: 10-bit, 0x1: 12-bit, 0x2: 14-bit
        (0x0 << 3) | // 0x0: No shift. 0x1: Shift right 1 bit. 0x2: Shift right 2 bits. 0x3: Shift right 3 bits.		
		(0x0 << 0) ; // Accumulate n conversions: 0x0: 1, 0x1:4, 0x2:8, 0x3:16, 0x4:32
	
	ADC0CF0=
	    ((SYSCLK/SARCLK) << 3) | // SAR Clock Divider. Max is 18MHz. Fsarclk = (Fadcclk) / (ADSC + 1)
		(0x0 << 2); // 0:SYSCLK ADCCLK = SYSCLK. 1:HFOSC0 ADCCLK = HFOSC0.
	
	ADC0CF1=
		(0 << 7)   | // 0: Disable low power mode. 1: Enable low power mode.
		(0x1E << 0); // Conversion Tracking Time. Tadtk = ADTK / (Fsarclk)
	
	ADC0CN0 =
		(0x0 << 7) | // ADEN. 0: Disable ADC0. 1: Enable ADC0.
		(0x0 << 6) | // IPOEN. 0: Keep ADC powered on when ADEN is 1. 1: Power down when ADC is idle.
		(0x0 << 5) | // ADINT. Set by hardware upon completion of a data conversion. Must be cleared by firmware.
		(0x0 << 4) | // ADBUSY. Writing 1 to this bit initiates an ADC conversion when ADCM = 000. This bit should not be polled to indicate when a conversion is complete. Instead, the ADINT bit should be used when polling for conversion completion.
		(0x0 << 3) | // ADWINT. Set by hardware when the contents of ADC0H:ADC0L fall within the window specified by ADC0GTH:ADC0GTL and ADC0LTH:ADC0LTL. Can trigger an interrupt. Must be cleared by firmware.
		(0x0 << 2) | // ADGN (Gain Control). 0x0: PGA gain=1. 0x1: PGA gain=0.75. 0x2: PGA gain=0.5. 0x3: PGA gain=0.25.
		(0x0 << 0) ; // TEMPE. 0: Disable the Temperature Sensor. 1: Enable the Temperature Sensor.

	ADC0CF2= 
		(0x0 << 7) | // GNDSL. 0: reference is the GND pin. 1: reference is the AGND pin.
		(0x1 << 5) | // REFSL. 0x0: VREF pin (external or on-chip). 0x1: VDD pin. 0x2: 1.8V. 0x3: internal voltage reference.
		(0x1F << 0); // ADPWR. Power Up Delay Time. Tpwrtime = ((4 * (ADPWR + 1)) + 2) / (Fadcclk)
	
	ADC0CN2 =
		(0x0 << 7) | // PACEN. 0x0: The ADC accumulator is over-written.  0x1: The ADC accumulator adds to results.
		(0x0 << 0) ; // ADCM. 0x0: ADBUSY, 0x1: TIMER0, 0x2: TIMER2, 0x3: TIMER3, 0x4: CNVSTR, 0x5: CEX5, 0x6: TIMER4, 0x7: TIMER5, 0x8: CLU0, 0x9: CLU1, 0xA: CLU2, 0xB: CLU3

	ADEN=1; // Enable ADC
}

// Uses Timer3 to delay <us> micro-seconds. 
void Timer3us(unsigned char us)
{
	unsigned char i;               // usec counter
	
	// The input for Timer 3 is selected as SYSCLK by setting T3ML (bit 6) of CKCON0:
	CKCON0|=0b_0100_0000;
	
	TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
	TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow
	
	TMR3CN0 = 0x04;                 // Sart Timer3 and clear overflow flag
	for (i = 0; i < us; i++)       // Count <us> overflows
	{
		while (!(TMR3CN0 & 0x80));  // Wait for overflow
		TMR3CN0 &= ~(0x80);         // Clear overflow indicator
	}
	TMR3CN0 = 0 ;                   // Stop Timer3 and clear overflow flag
}

void waitms (unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0; j<ms; j++)
		for (k=0; k<4; k++) Timer3us(250);
}

#define VDD 3.3035 // The measured value of VDD in volts

void InitPinADC (unsigned char portno, unsigned char pin_num)
{
	unsigned char mask;
	
	mask=1<<pin_num;

	SFRPAGE = 0x20;
	switch (portno)
	{
		case 0:
			P0MDIN &= (~mask); // Set pin as analog input
			P0SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 1:
			P1MDIN &= (~mask); // Set pin as analog input
			P1SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 2:
			P2MDIN &= (~mask); // Set pin as analog input
			P2SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		default:
		break;
	}
	SFRPAGE = 0x00;
}

unsigned int ADC_at_Pin(unsigned char pin)
{
	ADC0MX = pin;   // Select input from pin
	ADINT = 0;
	ADBUSY = 1;     // Convert voltage at the pin
	while (!ADINT); // Wait for conversion to complete
	return (ADC0);
}

float Volts_at_Pin(unsigned char pin)
{
	 return ((ADC_at_Pin(pin)*VDD)/16383.0);
}

//=========================LCD Functions===================
void LCD_pulse(void)
{
	LCD_E=1; 
	Timer3us(40);
	LCD_E=0;
}

void LCD_byte(unsigned char x)
{
	// The accumulator in the C8051Fxxx is bit addressable!
	ACC=x; // Send high nible 
	LCD_D7=ACC_7;
	LCD_D6=ACC_6;
	LCD_D5=ACC_5;
	LCD_D4=ACC_4;
	LCD_pulse();
	Timer3us(40);
	ACC=x; // Send low nible 
	LCD_D7=ACC_3;
	LCD_D6=ACC_2;
	LCD_D5=ACC_1;
	LCD_D4=ACC_0;
	LCD_pulse();
}

void WriteData (unsigned char x) 
{
	LCD_RS=1; 
	LCD_byte(x);
	waitms(2);
}

void WriteCommand (unsigned char x)
{
	LCD_RS=0;
	LCD_byte(x);
	waitms(5);
}

void LCD_4BIT (void) 
{
	LCD_E=0; // Resting state of LCD's enable is zero
	LCD_RW=0; // We are only writing to the LCD in this program
	waitms(20); 
	// First make sure the LCD is in 8-bit mode and then change to 4-bit modeWriteCommand(0x33);
	WriteCommand(0x33);
	WriteCommand(0x33);
	WriteCommand(0x32); // Change to 4-bit mode
	
	// Configure the LCD 
	WriteCommand(0x28);
	WriteCommand(0x0c); 
	WriteCommand(0x01); // Clear screen command(takes some time)
	waitms(20); // Wait for clear screen command to finish. 
}

void LCDprint(char * string, unsigned char line, bit clear)
{
	int j;

	WriteCommand(line==2?0xc0:0x80);
	waitms(5);
	for(j=0; string[j]!=0; j++)	WriteData(string[j]);// Write the message
	if(clear) for(; j<CHARS_PER_LINE; j++) WriteData(' '); // Clear the rest of the line
}

int getsn (char * buff, int len)
{
	int j;
	char c;
	
	for(j=0; j<(len-1); j++)
	{
		c=getchar();
		if ( (c=='\n') || (c=='\r') )
		{
			buff[j]=0;
			return j;
		}
		else
		{
			buff[j]=c;
		}
	}
	buff[j]=0;
	return len;
}

//=========================SPI Functions===================

unsigned char getbyte (void)
{
	char c;
	while (!RI);
	RI=0;
	c=SBUF;
	return c;
}

void putbyte (char c)
{
	while (!TI);
	TI=0;
	SBUF=c;
}

unsigned char SPIWrite (unsigned char x)
{
   SPI0DAT=x;
   while(!SPIF);
   SPIF=0;
   return SPI0DAT;
}

void Timer2_ISR (void) interrupt INTERRUPT_TIMER2
{
	unsigned char x;
	
	SFRPAGE=0x0;
	TF2H = 0; // Clear Timer2 interrupt flag

	if(playcnt==0)
	{
		CS=1; // Done playing
		TR2=0;
	}
	else
	{
		x=SPIWrite(0x55);
		SFRPAGE = 0x30;
		DAC0=x<<4;
		playcnt--;
	}
}

void Start_Playback (unsigned long int address, unsigned long int numb)
{
	CS=1;
	TR2=0;
	TMR2=0xffff; // Set to reload immediately
    CS=0;
    SPIWrite(READ_BYTES);
    SPIWrite((unsigned char)((address>>16)&0xff));
    SPIWrite((unsigned char)((address>>8)&0xff));
    SPIWrite((unsigned char)(address&0xff));
    playcnt=numb;
    TR2=1;
}

void Enable_Write (void)
{
    CS=0;
    SPIWrite(WRITE_ENABLE);
    CS=1;
}

void Check_WIP (void)
{
	unsigned char c;
	do
	{
	    CS=0;
	    SPIWrite(READ_STATUS);
	    c=SPIWrite(0x55);
	    CS=1;
	} while (c&0x01);
}


//========================================================



void main (void)
{
	float v[4];
	char buff[17];
	float temp;
	//unsigned char c;
    //unsigned int j, n;
    volatile union long_bytes start, nbytes;
    
    CS=1;
	TR2=0;
	

    waitms(500); // Give PuTTy a chance to start before sending

	
	InitPinADC(2, 2); // Configure P2.2 as analog input

    InitADC();
    
    // Configure the LCD 
	LCD_4BIT();
	
	// Display Temperature on LCD 
	LCDprint("Temperature (C):", 1, 1);


	while(1)
	{
		
	    // Read 14-bit value from the pins configured as analog inputs
		v[0] = Volts_at_Pin(QFP32_MUX_P2_2);

		printf ("%7.5f\r\r\n", v[0]);
		
		temp = 100.0*(v[0]-2.73);
				
		sprintf (buff,"%7.5f" , temp);
		LCDprint(buff, 2, 1);	
		waitms(500);
		
		// Turn on LEDs
	     if (temp < 15)
		 {
			 // Turn on Blue LED
		 	P2_1 = 0; // on
		 	P2_3 = 1; // off
		 	P2_4 = 1; // off

		 }
		 else if (temp < 20)
		 {
			 // Turn on Green LED
		 	P2_1 = 1; // off
		 	P2_3 = 0; // on
		 	P2_4 = 1; // off

		 }
		 else 
		 {
			 // Turn on Red LED
		 	P2_1 = 1; // off
		 	P2_3 = 1; // off
		 	P2_4 = 0; // on
		 }
		 
		 
		 
		 // Play Sound
		 if(P3_7==0)
		{
			while(P3_7==0);
			
			if (temp < 15)
		 {
			// Play Cold
			Start_Playback(0x100L, 0x400000L-0x100L);
			start.b[3]=0x000;
			start.b[2]=0x000;
			start.b[1]=0x00;
			start.b[0]=0x2d;
			
			// Get the number of bytes to playback
			nbytes.b[3]=0x0;
			nbytes.b[2]=0x0;
			nbytes.b[1]=0x2a;
			nbytes.b[0]=0xdf;

		 }
		 else if (temp < 20)
		 {
		 	// Play Normal
			//Start_Playback(0x100L, 0x400000L-0x100L);
			start.b[3]=0x000;
			start.b[2]=0x000;
			start.b[1]=0x2b;
			start.b[0]=0x0c;
			
			// Get the number of bytes to playback
			nbytes.b[3]=0x0;
			nbytes.b[2]=0x0;
			nbytes.b[1]=0x4d;
			nbytes.b[0]=0xf9;

		 }
		 else 
		 {
			// Play Hot
			//Start_Playback(0x100L, 0x400000L-0x100L);
			start.b[3]=0x000;
			start.b[2]=0x000;
			start.b[1]=0x79;
			start.b[0]=0x05;
			
			// Get the number of bytes to playback
			nbytes.b[3]=0x0;
			nbytes.b[2]=0x0;
			nbytes.b[1]=0x3a;
			nbytes.b[0]=0x35;
		 }
			
						
			Start_Playback(start.l, nbytes.l);
			
		}
	 
	 }  
}	
