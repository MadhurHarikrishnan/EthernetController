// Ethernet Example
// Madhurkiran Harikrishnan

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL w/ ENC28J60
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// ENC28J60 Ethernet controller
//   MOSI (SSI2Tx) on PB7
//   MISO (SSI2Rx) on PB6
//   SCLK (SSI2Clk) on PB4
//   ~CS connected to PB1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "enc28j60.h"
#include "wait.h"

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))



//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

uint8_t data[1600];

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(uint8_t c)
{
	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
	int i;
    for (i = 0; i < strlen(str); i++)
	  putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
	while (UART0_FR_R & UART_FR_RXFE);
	return UART0_DR_R & 0xFF;
}


// Initialize Hardware
void initHw()
{
	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port B and E peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOF;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x0E;  // bits 1-3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0E; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x1E;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R = 0x1E;  // enable internal pull-up for push button

    GPIO_PORTD_DIR_R = 0x0;  // bits 1-3 are outputs, other pins are inputs
    GPIO_PORTD_DEN_R = 0x0;  // enable LEDs and pushbuttons

    // Configure ~CS for ENC28J60
    GPIO_PORTB_DIR_R = 0x02;  // make bit 1 an output
    GPIO_PORTB_DR2R_R = 0x02; // set drive strength to 2mA
    GPIO_PORTB_DEN_R = 0x02;  // enable bits 1 for digital

    // Configure SSI2 pins for SPI configuration
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;           // turn-on SSI2 clocking
    GPIO_PORTB_DIR_R |= 0x90;                        // make bits 4 and 7 outputs
    GPIO_PORTB_DR2R_R |= 0x90;                       // set drive strength to 2mA
	GPIO_PORTB_AFSEL_R |= 0xD0;                      // select alternative functions for MOSI, MISO, SCLK pins
    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB6_SSI2RX | GPIO_PCTL_PB4_SSI2CLK; // map alt fns to SSI2
    GPIO_PORTB_DEN_R |= 0xD0;                        // enable digital operation on TX, RX, CLK pins

    // Configure the SSI2 as a SPI master, mode 3, 8bit operation, 1 MHz bit rate
    SSI2_CR1_R &= ~SSI_CR1_SSE;                      // turn off SSI2 to allow re-configuration
    SSI2_CR1_R = 0;                                  // select master mode
    SSI2_CC_R = 0;                                   // select system clock as the clock source
    SSI2_CPSR_R = 40;                                // set bit rate to 1 MHz (if SR=0 in CR0)
    SSI2_CR0_R = SSI_CR0_FRF_MOTO | SSI_CR0_DSS_8;   // set SR=0, mode 0 (SPH=0, SPO=0), 8-bit
    SSI2_CR1_R |= SSI_CR1_SSE;                       // turn on SSI2

    // Configure UART0 pins
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
	GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

   	// Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
	UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module


}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
   // uint8_t* udpData;
    uint8_t* tcpData;
    uint8_t p=0;
    uint16_t i=0;
    Seq_number=0xfb0983f7;
    Ack_number=0x00000;
    myport=0xb921;
    id=0x1229;
    uint8_t* http;

    // init controller
    initHw();

    putsUart0("\r\nWeather forecast\r\n");
    // init ethernet interface
    etherInit(ETHER_UNICAST | ETHER_BROADCAST | ETHER_HALFDUPLEX);
    etherSetIpAddress(192,168,137,199);

    // flash phy leds
    etherWritePhy(PHLCON, 0x0880);
    RED_LED = 1;

    waitMicrosecond(500000);
    etherWritePhy(PHLCON, 0x0990);
    RED_LED = 0;
    waitMicrosecond(500000);

    sendSynWeather(data);

    // message loop
    while (1)
    {
        if (etherKbhit())
        {
            if (etherIsOverflow())
            {
                RED_LED = 1;
                waitMicrosecond(100000);
                RED_LED = 0;
            }
            // get packet
            etherGetPacket(data, 1500);
            // handle arp request

            if (etherIsArp(data))
            {
                etherSendArpResp(data);
                RED_LED = 1;
               // GREEN_LED = 1;
                waitMicrosecond(50000);
                RED_LED = 0;
                GREEN_LED = 0;
            }

            // handle ip datagram

            if (etherIsIp(data))
            {
            	if (etherIsIpUnicast(data))
            	{
            		// handle icmp ping request

            		if (etherIsPingReq(data))
					{
					  etherSendPingResp(data);
					  GREEN_LED = 1;
					  waitMicrosecond(50000);
					  GREEN_LED = 0;

					}

					if(etherIsTcp(data))
					{
						tcpData = etherGetTcpData(data);
						if(etherisHandshake1(data))
						{
							etherRespondtoHandshake1(data);
							RED_LED = 1;
							//GREEN_LED=1;
							BLUE_LED=1;
							waitMicrosecond(100000);
							RED_LED = 0;
							//GREEN_LED=0;
							BLUE_LED=0;
							break;
						}

					}


                }
            }
        }
    }

    sendGETRequest(data);

    while(1)
    {
    	if (etherKbhit())
    	{
    		etherGetPacket(data, 1500);
			if(etherIsTcp(data))
			{

				RED_LED = 1;
				//GREEN_LED=1;
				BLUE_LED=1;
				waitMicrosecond(100000);
				RED_LED = 0;
				//GREEN_LED=0;
				BLUE_LED=0;

				if(p<=7)
					{
					if(tcp_DataCount>0)
						etherRespondToData(data);
					}


				//if(p>=2)
				{
					//p=0;
					tcpData = etherGetTcpData(data);
					http=tcpData+9;
					if((*http==0x32))//|(*http==0x35))
					{
						tcpData=tcpData+299;
						for(i=0;i<(tcp_DataCount-299);i++)
						{
						putcUart0(*tcpData);
						tcpData++;
						}
						if(*http==0x32)
								break;
					}
					//break;
					//sendFinRequest(data);

				}
				p++;
			}
    	}

    }
    while(1);

    return 0;
}
