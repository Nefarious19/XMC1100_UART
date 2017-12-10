/*----------------------------------------------------------------------------
 *      main Template for CMSIS RTE C/C++ Project
 *----------------------------------------------------------------------------
 *      Name:    main.c
 *      Purpose: Generic main program body including main() function
 *      Rev.:    1.0.0
 *----------------------------------------------------------------------------*/
/*******************************************************************************
* Copyright (c) 2015 ARM Ltd. and others
* All rights reserved. This program and the accompanying materials
* are made available under the terms of the Eclipse Public License v1.0
* which accompanies this distribution, and is available at
* http://www.eclipse.org/legal/epl-v10.html
*
* Contributors:
* ARM Ltd and ARM Germany GmbH - file template
*******************************************************************************/

#ifdef _RTE_
  #include "RTE_Components.h"             // Component selection
#endif

#include "xmc_uart.h"
#include "xmc_gpio.h"
#include "xmc_scu.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define RX_SIZE 64
#define TX_SIZE 64


volatile uint32_t timer1;
volatile uint8_t ascii_lines;


char RX_buffer[RX_SIZE] = { 0 };
char tmp_buffer[RX_SIZE] = { 0 };
uint8_t rx_tail, rx_head;


void uart_putchar (char data)
{
	XMC_UART_CH_Transmit(XMC_UART0_CH0 , data);
}

char uart_getC (void)
{
	char data = 0;
	if(rx_tail != rx_head)
	{
		data = RX_buffer[rx_tail];
		rx_tail = (rx_tail+1) & (RX_SIZE-1);
		if(data == 0x0D) ascii_lines--;
	}
	return data;
}

char * uart_getS ( char * buf )
{
	char znak;
	char * wsk = buf;
	while( (znak=uart_getC()) ) *wsk++ = znak;
	*wsk = 0;
	return buf;
}

void uart_putS (char * data)
{
	char znak = 0;
	while( (znak = *data++) ) uart_putchar(znak);
}

void SysTick_Handler (void)
{
	if(timer1) timer1--;
}

void USIC0_0_IRQHandler (void)
{
	char data;
	uint8_t tmp_rx_head;

	if( (XMC_UART_CH_GetStatusFlag(XMC_UART0_CH0) &
		( XMC_UART_CH_STATUS_FLAG_RECEIVE_INDICATION ||
		XMC_UART_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION)) == 1)
	{
		data = (char) XMC_UART_CH_GetReceivedData(XMC_UART0_CH0);
		if(data == 0x0D) ascii_lines++;
		tmp_rx_head = (rx_head + 1) & (RX_SIZE-1);
		if(tmp_rx_head == rx_tail) NVIC_SystemReset();
		RX_buffer[tmp_rx_head] = data;
		rx_head = tmp_rx_head;
	}
}

/* main function */
int main(void)
{
//	SystemInit();
//	SystemCoreClockUpdate();

	XMC_GPIO_CONFIG_t TX_pin =
	{
		.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT6,
		.output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH
	};

	XMC_GPIO_CONFIG_t RX_pin =
	{
		.mode = XMC_GPIO_MODE_INPUT_PULL_UP,
		.input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
	};
	XMC_GPIO_CONFIG_t LED1 =
	{
		.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
		.output_level = XMC_GPIO_OUTPUT_LEVEL_LOW
	};
	XMC_GPIO_CONFIG_t LED2 =
	{
		.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
		.output_level = XMC_GPIO_OUTPUT_LEVEL_LOW
	};

	XMC_UART_CH_CONFIG_t UART_board =
	{
			.baudrate  = 9600UL,
			.data_bits = 8UL,
			.stop_bits = 1UL,
			.parity_mode = XMC_USIC_CH_PARITY_MODE_NONE,

	};



	XMC_USIC_CH_t *uart_0 = XMC_UART0_CH0;



	XMC_UART_CH_Init(uart_0, &UART_board);
	XMC_UART_CH_SetInputSource(uart_0, XMC_USIC_CH_INPUT_DX3, USIC0_C0_DX3_P2_2);
	USIC0_CH0->DX0CR &= ~(USIC_CH_DX0CR_DSEL_Msk);
	USIC0_CH0->DX0CR |= 6 << USIC_CH_DX0CR_DSEL_Pos;
	XMC_UART_CH_Start(uart_0);
	USIC0_CH0->INPR &= ~(USIC_CH_INPR_RINP_Msk);
	XMC_UART_CH_ClearStatusFlag(uart_0, XMC_UART_CH_STATUS_FLAG_RECEIVE_INDICATION);
	XMC_UART_CH_EnableEvent(uart_0, XMC_UART_CH_EVENT_STANDARD_RECEIVE);
	NVIC_ClearPendingIRQ(USIC0_0_IRQn);
	NVIC_EnableIRQ(USIC0_0_IRQn);


	XMC_GPIO_Init( P2_1, &TX_pin );
	XMC_GPIO_Init( P2_2, &RX_pin );
	XMC_GPIO_Init( P1_0, &LED1 );
	XMC_GPIO_Init( P1_1, &LED2 );

	SysTick_Config(SystemCoreClock/1000);



	uint8_t licznik = 1;
	char itoa_buf[10];
  /* Infinite loop */
  while (1)
  {

	  if(!timer1)
	  {
		  	timer1 = 500;
			XMC_GPIO_ToggleOutput(P1_0);
			XMC_GPIO_ToggleOutput(P1_1);
			uart_putS(itoa(licznik++, itoa_buf, 10));
			uart_putS("\r\n");
	  }

	  if(ascii_lines){
		  uart_putS(uart_getS(tmp_buffer));
		  uart_putS("\r\n");
	  }

  }

}

