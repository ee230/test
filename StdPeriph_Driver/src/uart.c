/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 * 
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//   MSP430F22x4 Demo - USCI_A0, Ultra-Low Pwr UART 9600 RX/TX, 32kHz ACLK
//
//   Description: This program demonstrates a full-duplex 9600-baud UART using
//   USCI_A0 and a 32kHz crystal.  The program will wait in LPM3, and receive
//   a string1 into RAM, and echo back the complete string.
//   ACLK = BRCLK = LFXT1 = 32768Hz, MCLK = SMCLK = DCO ~1.2MHz
//   Baud rate divider with 32768Hz XTAL @9600 = 32768Hz/9600 = 3.41
//   //* An external watch crystal is required on XIN XOUT for ACLK *//
//
//                MSP430F22x4
//             -----------------
//         /|\|              XIN|-
//          | |                 | 32kHz
//          --|RST          XOUT|-
//            |                 |
//            |     P3.4/UCA0TXD|------------>
//            |                 | 9600 - 8N1
//            |     P3.5/UCA0RXD|<------------
//
//
//   A. Dannenberg
//   Texas Instruments Inc.
//   October 2006
//   Built with CCE Version: 3.2.0 and IAR Embedded Workbench Version: 3.41A
//******************************************************************************
#include <msp430.h>

char string1[8];
char i;
char j = 0;

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  P1DIR = 0xFF;                             // All P1.x outputs
  P1OUT = 0;                                // All P1.x reset
  P2DIR = 0xFF;                             // All P2.x outputs
  P2OUT = 0;                                // All P2.x reset
  P3SEL = 0x30;                             // P3.4,5 = USCI_A0 TXD/RXD
  P3DIR = 0xFF;                             // All P3.x outputs
  P3OUT = 0;                                // All P3.x reset
  P4DIR = 0xFF;                             // All P4.x outputs
  P4OUT = 0;                                // All P4.x reset
  UCA0CTL1 |= UCSSEL_1;                     // CLK = ACLK
  UCA0BR0 = 0x03;                           // 32kHz/9600 = 3.41
  UCA0BR1 = 0x00;                           //
  UCA0MCTL = UCBRS1 + UCBRS0;               // Modulation UCBRSx = 3
  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt

  __bis_SR_register(LPM3_bits + GIE);       // Enter LPM3, interrupts enabled
}

// USCI A0/B0 Transmit ISR
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0TX_VECTOR))) USCI0TX_ISR (void)
#else
#error Compiler not supported!
#endif
{
  UCA0TXBUF = string1[i++];                 // TX next character

  if (i == sizeof string1)                  // TX over?
    IE2 &= ~UCA0TXIE;                       // Disable USCI_A0 TX interrupt
}

// USCI A0/B0 Receive ISR
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCI0RX_ISR (void)
#else
#error Compiler not supported!
#endif
{
  string1[j++] = UCA0RXBUF;
  if (j > sizeof string1 - 1)
  {
    i = 0;
    j = 0;
    IE2 |= UCA0TXIE;                        // Enable USCI_A0 TX interrupt
    UCA0TXBUF = string1[i++];
  }
}
