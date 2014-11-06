/*
 * rmobile reset definition
 *
 * Copyright (C) 2014  Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef PLATSMP_RST_H
#define PLATSMP_RST_H

struct rcar_rst_config {
	unsigned int rescnt;
	unsigned int rescnt_magic;
};

extern void r8a779x_deassert_reset(unsigned int cpu);
extern void r8a779x_assert_reset(unsigned int cpu);
extern void r8a779x_init_reset(struct rcar_rst_config *rst_config);

#endif /* PLATSMP_RST_H */
