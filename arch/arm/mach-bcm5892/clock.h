/*
 *  linux/arch/arm/mach-realview/clock.h
 *
 *  Copyright (C) 2004 ARM Limited.
 *  Written by Deep Blue Solutions Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
struct module;

struct clk {
	struct list_head		node;
	unsigned long		rate;
	struct module		*owner;
	const char		*name;
	void			*data;
};

int clk_register(struct clk *clk);
void clk_unregister(struct clk *clk);
