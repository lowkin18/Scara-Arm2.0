#include "scara_robo_header.h"


void uart_init(void)
{
  PTRxBuffer = RXBuffer;
  RXCount = 0;
  rx_flag = 1;
  P4SEL = BIT4+BIT5;                        // P3.4,5 = USCI_A0 TXD/RXD
  UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
  UCA1CTL1 |= UCSSEL_2;                     // CLK = ACLK
  UCA1BR0 = 0xD9;                           // 2400 (see User's Guide)
  UCA1BR1 = 0x00;                           //
  UCA1MCTL |= UCBRS_6+UCBRF_0;              // Modulation UCBRSx=6, UCBRFx=0
  UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  UCA1IE |= UCRXIE + UCTXIE;                         // Enable USCI_A1 RX interrupt

  __bis_SR_register(GIE);       // Enter LPM3, interrupts enabled
  __no_operation();                         // For debugger
}

// Echo back RXed character, confirm TX buffer is ready first
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A1_VECTOR))) USCI_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(UCA1IV,4))
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG
    while (!(UCA0IFG&UCTXIFG));
   *PTRxBuffer = UCA1RXBUF; 				// TX -> RXed character

   	PTRxBuffer++;
   	RXCount ++;
   	if(RXCount == 6)
   	{

   	PTRxBuffer = RXBuffer;
   	RXCount = 0;
   	rx_flag = 0;
   	UCA1IFG = 0x00;
   	break;
   	}


    break;
  case 4:break;                             // Vector 4 - TXIFG
  default: break;
  }
}
