all:
	sed -e 's/@/0/g' -e 's/\(uart\)/\10/i' uart.h.in > uart0.h
	sed -e 's/@/1/g' -e 's/\(uart\)/\11/i' uart.h.in > uart1.h
	sed -e 's/@/2/g' -e 's/\(uart\)/\12/i' uart.h.in > uart2.h
	sed -e 's/@/3/g' -e 's/\(uart\)/\13/i' uart.h.in > uart3.h
	sed -e 's/@/0/g' -e 's/\(uart\)/\10/i' uart.c.in > uart0.c
	sed -e 's/@/1/g' -e 's/\(uart\)/\11/i' uart.c.in > uart1.c
	sed -e 's/@/2/g' -e 's/\(uart\)/\12/i' uart.c.in > uart2.c
	sed -e 's/@/3/g' -e 's/\(uart\)/\13/i' uart.c.in > uart3.c

default:
	sed -e 's/@/0/g' -e 's/USART0/USART/i' uart.h.in > ../uart.h
	sed -e 's/@/0/g' -e 's/USART0/USART/i' uart.c.in > ../uart.c
