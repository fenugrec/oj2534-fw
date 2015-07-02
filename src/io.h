#ifndef IO_H
#define IO_H
/** i/o pin and periph defs **/

/* ISO9141/14230 *
Needs two UART pins for K line (TX + RX), + 1 pin for L

*/

#define ISO_UART	USART3
#define ISO_IRQH	USART3_4_IRQHandler
#define ISO_URCC	RCC_APB1Periph_USART3	//(verify APB1/APB2 if changing USART)

#define ISO_GRCC	RCC_AHBPeriph_GPIOC
#define ISO_GPIO	GPIOC
	//warning : some hard-coded GPIO stuff in iso_tx.c !
#define ISO_KTX	10	//PC10 (AF1) == TX, PP out, GPIO / AF1 selon init
#define ISO_KRX	11	//PC11 (AF1) = RX: I?, PU
#define ISO_L	12	//PC12 (GPIO): L; PP out


#endif // IO_H
