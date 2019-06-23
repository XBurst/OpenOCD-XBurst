/***************************************************************************
 *   Copyright (C) 2018 by TianyangLiu                                     *
 *   rick.tyliu@ingenic.com                                                *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_MIPS_XBURST_H
#define OPENOCD_TARGET_MIPS_XBURST_H

struct target;

#define MIPS_XBURST_COMMON_MAGIC	0xB321B321

struct mips_xburst_common {
	uint32_t common_magic;
	struct mips32_common mips32;
};

static inline struct mips_xburst_common *
target_to_xburst(struct target *target)
{
	return container_of(target->arch_info,
			struct mips_xburst_common, mips32);
}

extern const struct command_registration mips_xburst_command_handlers[];

#endif /* OPENOCD_TARGET_MIPS_XBURST_H */
