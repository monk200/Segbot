#ifndef F28379DSERIAL_H_
#define F28379DSERIAL_H_
#include <buffer.h>


#define PLL_IMULT           0x28        //40
#define OSCCLK_KHZ          10000L  //10 MHz
#define SYSCLKOUT_KHZ   (OSCCLK_KHZ*PLL_IMULT/((ClkCfgRegs.SYSCLKDIVSEL.bit.PLLSYSCLKDIV == 0) ? 1 : (ClkCfgRegs.SYSCLKDIVSEL.bit.PLLSYSCLKDIV*2)))
//LSPCLKDIV shouldn't be changed, so then should still be default /4
//so 200Mhz/4 = 50Mhz
#define LSPCLK_KHZ (SYSCLKOUT_KHZ/((ClkCfgRegs.LOSPCP.bit.LSPCLKDIV == 0) ? 1 : (ClkCfgRegs.LOSPCP.bit.LSPCLKDIV*2)))

#define LSPCLK_HZ       (LSPCLK_KHZ*1000L)

typedef struct serial_s {
	volatile struct buffer_s TX;
	//volatile buffer_t TX;
	volatile struct SCI_REGS *sci;
	void (*got_data)(struct serial_s *s, char data);
} serial_t;


extern serial_t SerialA;
//extern serial_t SerialB;
extern serial_t SerialC;
extern serial_t SerialD;

uint16_t init_serial(serial_t *s, Uint32 baud, void (*got_func)(serial_t *s, char data));
void uninit_serial(serial_t *s);
uint16_t serial_send(serial_t *s, char *data, Uint16 len);
uint16_t serial_printf(serial_t *s, char *fmt, ...);
void UART_printfLine(unsigned char line, char *format, ...);
void UART_vprintfLine(unsigned char line, char *format, va_list ap);
__interrupt void TXAINT_data_sent(void);
//__interrupt void TXBINT_data_sent(void);
__interrupt void TXCINT_data_sent(void);
__interrupt void TXDINT_data_sent(void);
__interrupt void RXAINT_recv_ready(void);
//__interrupt void RXBINT_recv_ready(void);
__interrupt void RXCINT_recv_ready(void);
__interrupt void RXDINT_recv_ready(void);

#endif /* F28379DSERIAL_H_ */
