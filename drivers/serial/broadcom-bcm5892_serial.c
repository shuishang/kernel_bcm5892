/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL"). 
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/



/*
 *  Driver for AMBA serial ports on BCM5892 board
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *  Based on drivers/serial/amba.c, by Deep Blue Solutions Ltd.
 *
 * This is a driver for ARM AMBA-type serial ports on BCM5892 board. They
 * have a lot of 16550-like features, but are not register compatible.
 * Note that although they do have CTS, DCD and DSR inputs, they do
 * not have an RI input, nor do they have DTR or RTS outputs.  If
 * required, these have to be supplied via some other means (eg, GPIO)
 * and hooked into this driver.
 */

#if defined(CONFIG_SERIAL_BCM5892_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/amba/bus.h>
#include <linux/amba/serial.h>
#include <linux/version.h>

#include <asm/io.h>
#include <asm/sizes.h>
#include <mach/hardware.h>

#define UART_NR			4

#define SERIAL_AMBA_MAJOR	4
#define SERIAL_AMBA_MINOR	64
#define SERIAL_AMBA_NR		UART_NR

#define AMBA_ISR_PASS_LIMIT	256

#define UART_DR_ERROR		(UART011_DR_OE|UART011_DR_BE|UART011_DR_PE|UART011_DR_FE)
#define UART_DUMMY_DR_RX	(1 << 16)

/*
 * We wrap our port structure around the generic uart_port.
 */
struct uart_port_bcm5892 {
	struct uart_port	port;
	unsigned int		im;	/* interrupt mask */
	unsigned int		old_status;
};

static void bcm5892uart_stop_tx(struct uart_port *port)
{
	struct uart_port_bcm5892 *uap = (struct uart_port_bcm5892 *)port;

	uap->im &= ~UART011_TXIM;
	writew(uap->im, uap->port.membase + UART011_IMSC);
}

static void bcm5892uart_start_tx(struct uart_port *port)
{
	struct uart_port_bcm5892 *uap = (struct uart_port_bcm5892 *)port;

	uap->im |= UART011_TXIM;
	writew(uap->im, uap->port.membase + UART011_IMSC);
}

static void bcm5892uart_stop_rx(struct uart_port *port)
{
	struct uart_port_bcm5892 *uap = (struct uart_port_bcm5892 *)port;

	uap->im &= ~(UART011_RXIM|UART011_RTIM|UART011_FEIM|
		     UART011_PEIM|UART011_BEIM|UART011_OEIM);
	writew(uap->im, uap->port.membase + UART011_IMSC);
}

static void bcm5892uart_enable_ms(struct uart_port *port)
{
	struct uart_port_bcm5892 *uap = (struct uart_port_bcm5892 *)port;

	uap->im |= UART011_RIMIM|UART011_CTSMIM|UART011_DCDMIM|UART011_DSRMIM;
	writew(uap->im, uap->port.membase + UART011_IMSC);
}

static void bcm5892uart_rx_chars(struct uart_port_bcm5892 *uap)
{

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
	struct tty_struct *tty = uap->port.state->port.tty;
#else
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,26)
	struct tty_struct *tty = uap->port.info->port.tty;
#else
	struct tty_struct *tty = uap->port.info->tty;
#endif
#endif
	unsigned int status, ch, flag, max_count = 256;

	status = readw(uap->port.membase + UART01x_FR);
	while ((status & UART01x_FR_RXFE) == 0 && max_count--) {
		ch = readw(uap->port.membase + UART01x_DR) | UART_DUMMY_DR_RX;
		flag = TTY_NORMAL;
		uap->port.icount.rx++;

		/*
		 * Note that the error handling code is
		 * out of the main execution path
		 */
		if (unlikely(ch & UART_DR_ERROR)) {
			if (ch & UART011_DR_BE) {
				ch &= ~(UART011_DR_FE | UART011_DR_PE);
				uap->port.icount.brk++;
				if (uart_handle_break(&uap->port))
					goto ignore_char;
			} else if (ch & UART011_DR_PE)
				uap->port.icount.parity++;
			else if (ch & UART011_DR_FE)
				uap->port.icount.frame++;
			if (ch & UART011_DR_OE)
				uap->port.icount.overrun++;

			ch &= uap->port.read_status_mask;

			if (ch & UART011_DR_BE)
				flag = TTY_BREAK;
			else if (ch & UART011_DR_PE)
				flag = TTY_PARITY;
			else if (ch & UART011_DR_FE)
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(&uap->port, ch & 255))
			goto ignore_char;

		uart_insert_char(&uap->port, ch, UART011_DR_OE, ch, flag);

	ignore_char:
		status = readw(uap->port.membase + UART01x_FR);
	}
	spin_unlock(&uap->port.lock);
	tty_flip_buffer_push(tty);
	spin_lock(&uap->port.lock);
}

static void bcm5892uart_tx_chars(struct uart_port_bcm5892 *uap)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
	struct circ_buf *xmit = &uap->port.state->xmit;
#else
	struct circ_buf *xmit = &uap->port.info->xmit;
#endif
	int count;

	if (uap->port.x_char) {
		writew(uap->port.x_char, uap->port.membase + UART01x_DR);
		uap->port.icount.tx++;
		uap->port.x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(&uap->port)) {
		bcm5892uart_stop_tx(&uap->port);
		return;
	}

	count = uap->port.fifosize >> 1;
	do {
		writew(xmit->buf[xmit->tail], uap->port.membase + UART01x_DR);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		uap->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&uap->port);

	if (uart_circ_empty(xmit))
		bcm5892uart_stop_tx(&uap->port);
}

static void bcm5892uart_modem_status(struct uart_port_bcm5892 *uap)
{
	unsigned int status, delta;

	status = readw(uap->port.membase + UART01x_FR) & UART01x_FR_MODEM_ANY;

	delta = status ^ uap->old_status;
	uap->old_status = status;

	if (!delta)
		return;

	if (delta & UART01x_FR_DCD)
		uart_handle_dcd_change(&uap->port, status & UART01x_FR_DCD);

	if (delta & UART01x_FR_DSR)
		uap->port.icount.dsr++;

	if (delta & UART01x_FR_CTS)
		uart_handle_cts_change(&uap->port, status & UART01x_FR_CTS);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
	wake_up_interruptible(&uap->port.state->port.delta_msr_wait);
#else
	wake_up_interruptible(&uap->port.info->port.delta_msr_wait);
#endif
}

static irqreturn_t bcm5892uart_int(int irq, void *dev_id)
{
	struct uart_port_bcm5892 *uap = dev_id;
	unsigned int status, pass_counter = AMBA_ISR_PASS_LIMIT;
	int handled = 0;

	spin_lock(&uap->port.lock);

	status = readw(uap->port.membase + UART011_MIS);
	if (status) {
		do {
			writew(status & ~(UART011_TXIS|UART011_RTIS|
					  UART011_RXIS),
			       uap->port.membase + UART011_ICR);

			if (status & (UART011_RTIS|UART011_RXIS))
				bcm5892uart_rx_chars(uap);
			if (status & (UART011_DSRMIS|UART011_DCDMIS|
				      UART011_CTSMIS|UART011_RIMIS))
				bcm5892uart_modem_status(uap);
			if (status & UART011_TXIS)
				bcm5892uart_tx_chars(uap);

			if (pass_counter-- == 0)
				break;

			status = readw(uap->port.membase + UART011_MIS);
		} while (status != 0);
		handled = 1;
	}

	spin_unlock(&uap->port.lock);

	return IRQ_RETVAL(handled);
}

static unsigned int bcm5892uart_tx_empty(struct uart_port *port)
{
	struct uart_port_bcm5892 *uap = (struct uart_port_bcm5892 *)port;
	unsigned int status = readw(uap->port.membase + UART01x_FR);
	return status & (UART01x_FR_BUSY|UART01x_FR_TXFF) ? 0 : TIOCSER_TEMT;
}

static unsigned int bcm5892uart_get_mctrl(struct uart_port *port)
{
	struct uart_port_bcm5892 *uap = (struct uart_port_bcm5892 *)port;
	unsigned int result = 0;
	unsigned int status = readw(uap->port.membase + UART01x_FR);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
#define TIOCMBIT(uartbit, tiocmbit)		\
	if (status & uartbit)		\
		result |= tiocmbit

	TIOCMBIT(UART01x_FR_DCD, TIOCM_CAR);
	TIOCMBIT(UART01x_FR_DSR, TIOCM_DSR);
	TIOCMBIT(UART01x_FR_CTS, TIOCM_CTS);
	TIOCMBIT(UART011_FR_RI, TIOCM_RNG);
#undef TIOCMBIT
#else
#define BIT(uartbit, tiocmbit)          \
 	        if (status & uartbit)           \
 	                result |= tiocmbit
 	
	BIT(UART01x_FR_DCD, TIOCM_CAR);
	BIT(UART01x_FR_DSR, TIOCM_DSR);
	BIT(UART01x_FR_CTS, TIOCM_CTS);
	BIT(UART011_FR_RI, TIOCM_RNG);
#undef BIT
#endif	
	return result;
}

static void bcm5892uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_port_bcm5892 *uap = (struct uart_port_bcm5892 *)port;
	unsigned int cr;

	cr = readw(uap->port.membase + UART011_CR);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
#define	TIOCMBIT(tiocmbit, uartbit)		\
	if (mctrl & tiocmbit)		\
		cr |= uartbit;		\
	else				\
		cr &= ~uartbit

	TIOCMBIT(TIOCM_RTS, UART011_CR_RTS);
	TIOCMBIT(TIOCM_DTR, UART011_CR_DTR);
	TIOCMBIT(TIOCM_OUT1, UART011_CR_OUT1);
	TIOCMBIT(TIOCM_OUT2, UART011_CR_OUT2);
	TIOCMBIT(TIOCM_LOOP, UART011_CR_LBE);
#undef TIOCMBIT
#else
#define	BIT(tiocmbit, uartbit)		\
	if (mctrl & tiocmbit)		\
		cr |= uartbit;		\
	else				\
		cr &= ~uartbit

	BIT(TIOCM_RTS, UART011_CR_RTS);
	BIT(TIOCM_DTR, UART011_CR_DTR);
	BIT(TIOCM_OUT1, UART011_CR_OUT1);
	BIT(TIOCM_OUT2, UART011_CR_OUT2);
	BIT(TIOCM_LOOP, UART011_CR_LBE);
 	
#undef BIT
#endif	

	writew(cr, uap->port.membase + UART011_CR);
}

static void bcm5892uart_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_port_bcm5892 *uap = (struct uart_port_bcm5892 *)port;
	unsigned long flags;
	unsigned int lcr_h;

	spin_lock_irqsave(&uap->port.lock, flags);
	lcr_h = readw(uap->port.membase + UART011_LCRH);
	if (break_state == -1)
		lcr_h |= UART01x_LCRH_BRK;
	else
		lcr_h &= ~UART01x_LCRH_BRK;
	writew(lcr_h, uap->port.membase + UART011_LCRH);
	spin_unlock_irqrestore(&uap->port.lock, flags);
}

static int bcm5892uart_startup(struct uart_port *port)
{
	struct uart_port_bcm5892 *uap = (struct uart_port_bcm5892 *)port;
	unsigned int cr;
	int retval;

	/*
	 * Allocate the IRQ
	 */
	retval = request_irq(uap->port.irq, bcm5892uart_int, 0, "uart", uap);
	if (retval)
		return retval;

	writew(UART011_IFLS_RX4_8|UART011_IFLS_TX4_8,
	       uap->port.membase + UART011_IFLS);

	/*
	 * Provoke TX FIFO interrupt into asserting.
	 */
	cr = UART01x_CR_UARTEN | UART011_CR_TXE | UART011_CR_LBE;
	writew(cr, uap->port.membase + UART011_CR);
	writew(0, uap->port.membase + UART011_FBRD);
	writew(1, uap->port.membase + UART011_IBRD);
	writew(0, uap->port.membase + UART011_LCRH);
	writew(0, uap->port.membase + UART01x_DR);
	while (readw(uap->port.membase + UART01x_FR) & UART01x_FR_BUSY)
		barrier();

	cr = UART011_CR_CTSEN | UART011_CR_RTSEN;
	cr |= UART01x_CR_UARTEN | UART011_CR_RXE | UART011_CR_TXE;
	writew(cr, uap->port.membase + UART011_CR);

	/*
	 * initialise the old status of the modem signals
	 */
	uap->old_status = readw(uap->port.membase + UART01x_FR) & UART01x_FR_MODEM_ANY;

	/*
	 * Finally, enable interrupts
	 */
	spin_lock_irq(&uap->port.lock);
	uap->im = UART011_RXIM | UART011_RTIM;
	writew(uap->im, uap->port.membase + UART011_IMSC);
	spin_unlock_irq(&uap->port.lock);

	return 0;
}

static void bcm5892uart_shutdown(struct uart_port *port)
{
	struct uart_port_bcm5892 *uap = (struct uart_port_bcm5892 *)port;
	unsigned long val;

	/*
	 * disable all interrupts
	 */
	spin_lock_irq(&uap->port.lock);
	uap->im = 0;
	writew(uap->im, uap->port.membase + UART011_IMSC);
	writew(0xffff, uap->port.membase + UART011_ICR);
	spin_unlock_irq(&uap->port.lock);

	/*
	 * Free the interrupt
	 */
	free_irq(uap->port.irq, uap);

	/*
	 * disable the port
	 */
	writew(UART01x_CR_UARTEN | UART011_CR_TXE, uap->port.membase + UART011_CR);

	/*
	 * disable break condition and fifos
	 */
	val = readw(uap->port.membase + UART011_LCRH);
	val &= ~(UART01x_LCRH_BRK | UART01x_LCRH_FEN);
	writew(val, uap->port.membase + UART011_LCRH);
}

static void
bcm5892uart_set_termios(struct uart_port *port, struct ktermios *termios,
		     struct ktermios *old)
{
	unsigned int lcr_h, old_cr;
	unsigned long flags;
	unsigned int baud, quot;

	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16);
	quot = port->uartclk * 4 / baud;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr_h = UART01x_LCRH_WLEN_5;
		break;
	case CS6:
		lcr_h = UART01x_LCRH_WLEN_6;
		break;
	case CS7:
		lcr_h = UART01x_LCRH_WLEN_7;
		break;
	default: /* CS8 */
		lcr_h = UART01x_LCRH_WLEN_8;
		break;
	}
	if (termios->c_cflag & CSTOPB)
		lcr_h |= UART01x_LCRH_STP2;
	if (termios->c_cflag & PARENB) {
		lcr_h |= UART01x_LCRH_PEN;
		if (!(termios->c_cflag & PARODD))
			lcr_h |= UART01x_LCRH_EPS;
	}
	if (port->fifosize > 1)
		lcr_h |= UART01x_LCRH_FEN;

	spin_lock_irqsave(&port->lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	port->read_status_mask = UART011_DR_OE | 255;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UART011_DR_FE | UART011_DR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= UART011_DR_BE;

	/*
	 * Characters to ignore
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= UART011_DR_FE | UART011_DR_PE;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= UART011_DR_BE;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= UART011_DR_OE;
	}

	/*
	 * Ignore all characters if CREAD is not set.
	 */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= UART_DUMMY_DR_RX;

	if (UART_ENABLE_MS(port, termios->c_cflag))
		bcm5892uart_enable_ms(port);

	/* first, disable everything */
	old_cr = readw(port->membase + UART011_CR);
	writew(0, port->membase + UART011_CR);

	/* Set baud rate */
	writew(quot & 0x3f, port->membase + UART011_FBRD);
	writew(quot >> 6, port->membase + UART011_IBRD);

	/*
	 * ----------v----------v----------v----------v-----
	 * NOTE: MUST BE WRITTEN AFTER UARTLCR_M & UARTLCR_L
	 * ----------^----------^----------^----------^-----
	 */
	writew(lcr_h, port->membase + UART011_LCRH);
	writew(old_cr, port->membase + UART011_CR);

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *bcm5892uart_type(struct uart_port *port)
{
	return port->type == PORT_AMBA ? "AMBA/BCM5892UART" : NULL;
}

/*
 * Release the memory region(s) being used by 'port'
 */
static void pl010_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase, SZ_4K);
}

/*
 * Request the memory region(s) being used by 'port'
 */
static int pl010_request_port(struct uart_port *port)
{
	return request_mem_region(port->mapbase, SZ_4K, "uart-bcm5892uart")
			!= NULL ? 0 : -EBUSY;
}

/*
 * Configure/autoconfigure the port.
 */
static void pl010_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_AMBA;
		pl010_request_port(port);
	}
}

/*
 * verify the new serial_struct (for TIOCSSERIAL).
 */
static int pl010_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_AMBA)
		ret = -EINVAL;
	if (ser->irq < 0 || ser->irq >= NR_IRQS)
		ret = -EINVAL;
	if (ser->baud_base < 9600)
		ret = -EINVAL;
	return ret;
}

static struct uart_ops amba_bcm5892uart_pops = {
	.tx_empty	= bcm5892uart_tx_empty,
	.set_mctrl	= bcm5892uart_set_mctrl,
	.get_mctrl	= bcm5892uart_get_mctrl,
	.stop_tx	= bcm5892uart_stop_tx,
	.start_tx	= bcm5892uart_start_tx,
	.stop_rx	= bcm5892uart_stop_rx,
	.enable_ms	= bcm5892uart_enable_ms,
	.break_ctl	= bcm5892uart_break_ctl,
	.startup		= bcm5892uart_startup,
	.shutdown	= bcm5892uart_shutdown,
	.set_termios	= bcm5892uart_set_termios,
	.type		= bcm5892uart_type,
	.release_port	= pl010_release_port,
	.request_port	= pl010_request_port,
	.config_port	= pl010_config_port,
	.verify_port	= pl010_verify_port,
};

static struct uart_port_bcm5892 amba_ports[UART_NR] = {
	{
		.port = {
			.membase	= (char*)(IO_ADDRESS(URT0_REG_BASE_ADDR)),
			.mapbase	= URT0_REG_BASE_ADDR,
			.iotype		= SERIAL_IO_MEM,
			.irq		= IRQ_OUART0,
			.uartclk		= 24000000,
			/*
			 *.uartclk	= 10000000,
			 *.uartclk	= 5000000,
			 */
			.fifosize		= 16,
			.ops		= &amba_bcm5892uart_pops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 0,
		},
	},
	{
		.port = {
			.membase	= (char*)(IO_ADDRESS(URT1_REG_BASE_ADDR)),
			.mapbase	= URT1_REG_BASE_ADDR,
			.iotype		= SERIAL_IO_MEM,
			.irq		= IRQ_OUART1,
			.uartclk		= 24000000,
			/*
			 *.uartclk	= 10000000,
			 *.uartclk	= 5000000,
			 */
			.fifosize		= 16,
			.ops		= &amba_bcm5892uart_pops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 1,
		},
	},
	{
		.port = {
			.membase        = (char*)(IO_ADDRESS(URT2_REG_BASE_ADDR)),
			.mapbase	= URT2_REG_BASE_ADDR,
			.iotype 		= SERIAL_IO_MEM,
			.irq            	= IRQ_OUART2,
			.uartclk		= 24000000,
			/*
			 *.uartclk      = 10000000,
			 *.uartclk        = 5000000,
			 */
			.fifosize		= 16,
			.ops		= &amba_bcm5892uart_pops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 2,
		},
	},
	{
		.port = {
			.membase        = (char*)(IO_ADDRESS(URT3_REG_BASE_ADDR)),
			.mapbase	= URT3_REG_BASE_ADDR,
			.iotype 		= SERIAL_IO_MEM,
			.irq            	= IRQ_OUART3,
			.uartclk		= 24000000,
			/*
			 *.uartclk	= 10000000,
			 *.uartclk	= 5000000,
			 */
			.fifosize		= 16,
			.ops		= &amba_bcm5892uart_pops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 3,
		},
	},
};

#ifdef CONFIG_SERIAL_BCM5892_CONSOLE

static void bcm5892uart_console_putchar(struct uart_port *port, int ch)
{
	struct uart_port_bcm5892 *uap = (struct uart_port_bcm5892 *)port;

	while (readw(uap->port.membase + UART01x_FR) & UART01x_FR_TXFF)
		barrier();
	writew(ch, uap->port.membase + UART01x_DR);
}

static void
bcm5892uart_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_port_bcm5892 *uap = &amba_ports[co->index];
	unsigned int status, old_cr, new_cr;

	/*
	 *	First save the CR then disable the interrupts
	 */
	old_cr = readw(uap->port.membase + UART011_CR);
	new_cr = old_cr & ~UART011_CR_CTSEN;
	new_cr |= UART01x_CR_UARTEN | UART011_CR_TXE;
	writew(new_cr, uap->port.membase + UART011_CR);

	uart_console_write(&uap->port, s, count, bcm5892uart_console_putchar);

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore the TCR
	 */
	do {
		status = readw(uap->port.membase + UART01x_FR);
	} while (status & UART01x_FR_BUSY);
	writew(old_cr, uap->port.membase + UART011_CR);

}

static void __init
bcm5892uart_console_get_options(struct uart_port_bcm5892 *uap, int *baud,
			     int *parity, int *bits)
{
	if (readw(uap->port.membase + UART011_CR) & UART01x_CR_UARTEN) {
		unsigned int lcr_h, ibrd, fbrd;

		lcr_h = readw(uap->port.membase + UART011_LCRH);

		*parity = 'n';
		if (lcr_h & UART01x_LCRH_PEN) {
			if (lcr_h & UART01x_LCRH_EPS)
				*parity = 'e';
			else
				*parity = 'o';
		}

		if ((lcr_h & 0x60) == UART01x_LCRH_WLEN_7)
			*bits = 7;
		else
			*bits = 8;

		ibrd = readw(uap->port.membase + UART011_IBRD);
		fbrd = readw(uap->port.membase + UART011_FBRD);

		*baud = uap->port.uartclk * 4 / (64 * ibrd + fbrd);
	}
}

static int __init bcm5892uart_console_setup(struct console *co, char *options)
{
	struct uart_port_bcm5892 *uap;
	int baud = 38400;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index >= UART_NR)
		co->index = 0;
	uap = &amba_ports[co->index];
	if (!uap)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		bcm5892uart_console_get_options(uap, &baud, &parity, &bits);

	return uart_set_options(&uap->port, co, baud, parity, bits, flow);
}

static struct uart_driver amba_reg;
static struct console amba_console = {
	.name		= "ttyS",
	.write		= bcm5892uart_console_write,
	.device		= uart_console_device,
	.setup		= bcm5892uart_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &amba_reg,
};

/* Added to get called from console_init */
static int __init bcm5892uart_console_init(void)
{
	register_console(&amba_console);
	return 0;
}

console_initcall(bcm5892uart_console_init);

#define AMBA_CONSOLE	(&amba_console)
#else
#define AMBA_CONSOLE	NULL
#endif

static struct uart_driver amba_reg = {
	.owner			= THIS_MODULE,
	.driver_name		= "ttyS",
	.dev_name		= "ttyS",
	.major			= SERIAL_AMBA_MAJOR,
	.minor			= SERIAL_AMBA_MINOR,
	.nr			= UART_NR,
	.cons			= AMBA_CONSOLE,
};

/* For 5892, all init is done in struct amba_ports[] */
static int bcm5892uart_probe(struct amba_device *dev, void *id)
{
	return 0;  
}
static int bcm5892uart_remove(struct amba_device *dev)
{
	return 0;
}

static struct amba_id bcm5892uart_ids[] __initdata = {
	{
		.id	= 0x00041011,
		.mask	= 0x000fffff,
	},
	{ 0, 0 },
};

static struct amba_driver bcm5892uart_driver = {
	.drv = {
		.name	= "bcm5892 uart",
	},
	.id_table	= bcm5892uart_ids,
	.probe		= bcm5892uart_probe,
	.remove		= bcm5892uart_remove,
};

static int __init bcm5892uart_init(void)
{
	int ret;
	int i;

	printk(KERN_INFO "Serial: AMBA 5892 BCM5892UART UART driver\n");

	ret = uart_register_driver(&amba_reg);
	if (ret == 0) {
		ret = amba_driver_register(&bcm5892uart_driver);
		if (ret)
			uart_unregister_driver(&amba_reg);

		for (i = 0; i < UART_NR; i++)
		  uart_add_one_port(&amba_reg,
				    &amba_ports[i].port);

	}
	return ret;
}

static void __exit bcm5892uart_exit(void)
{
	amba_driver_unregister(&bcm5892uart_driver);
	uart_unregister_driver(&amba_reg);
}

module_init(bcm5892uart_init);
module_exit(bcm5892uart_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("BROADCOM BCM5892 serial port driver");
MODULE_LICENSE("GPL");
