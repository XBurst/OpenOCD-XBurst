/***************************************************************************
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by David T.L. Wong                                 *
 *                                                                         *
 *   Copyright (C) 2009 by David N. Claffey <dnclaffey@gmail.com>          *
 *                                                                         *
 *   Copyright (C) 2011 by Drasko DRASKOVIC                                *
 *   drasko.draskovic@gmail.com                                            *
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

/*
 * This version has optimized assembly routines for 32 bit operations:
 * - read word
 * - write word
 * - write array of words
 *
 * One thing to be aware of is that the MIPS32 cpu will execute the
 * instruction after a branch instruction (one delay slot).
 *
 * For example:
 *  LW $2, ($5 +10)
 *  B foo
 *  LW $1, ($2 +100)
 *
 * The LW $1, ($2 +100) instruction is also executed. If this is
 * not wanted a NOP can be inserted:
 *
 *  LW $2, ($5 +10)
 *  B foo
 *  NOP
 *  LW $1, ($2 +100)
 *
 * or the code can be changed to:
 *
 *  B foo
 *  LW $2, ($5 +10)
 *  LW $1, ($2 +100)
 *
 * The original code contained NOPs. I have removed these and moved
 * the branches.
 *
 * These changes result in a 35% speed increase when programming an
 * external flash.
 *
 * More improvement could be gained if the registers do no need
 * to be preserved but in that case the routines should be aware
 * OpenOCD is used as a flash programmer or as a debug tool.
 *
 * Nico Coesel
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/time_support.h>
#include <sys/stat.h>

#include "mips32.h"
#include "mips32_pracc.h"

static int wait_for_pracc_rw(struct mips_ejtag *ejtag_info)
{
	int64_t then = timeval_ms();

	/* wait for the PrAcc to become "1" */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);

	while (1) {
		ejtag_info->pa_ctrl = ejtag_info->ejtag_ctrl;
		int retval = mips_ejtag_drscan_32(ejtag_info, &ejtag_info->pa_ctrl);
		if (retval != ERROR_OK)
			return retval;

		if (ejtag_info->pa_ctrl & EJTAG_CTRL_PRACC)
			break;

		int64_t timeout = timeval_ms() - then;
		if (timeout > 1000) {
			LOG_DEBUG("DEBUGMODULE: No memory access in progress!");
			return ERROR_JTAG_DEVICE_ERROR;
		}
	}

	return ERROR_OK;
}

/* Shift in control and address for a new processor access, save them in ejtag_info */
static int mips32_pracc_read_ctrl_addr(struct mips_ejtag *ejtag_info)
{
	int retval = wait_for_pracc_rw(ejtag_info);
	if (retval != ERROR_OK)
		return retval;

	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS);

	ejtag_info->pa_addr = 0;
	return mips_ejtag_drscan_32(ejtag_info, &ejtag_info->pa_addr);
}

static int mips32_pracc_try_read_ctrl_addr(struct mips_ejtag *ejtag_info)
{
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
	ejtag_info->pa_ctrl = ejtag_info->ejtag_ctrl;
	int retval = mips_ejtag_drscan_32(ejtag_info, &ejtag_info->pa_ctrl);
	if (retval != ERROR_OK)
		return retval;

	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS);

	ejtag_info->pa_addr = 0;
	return mips_ejtag_drscan_32(ejtag_info, &ejtag_info->pa_addr);
}

/* Finish processor access */
static int mips32_pracc_finish(struct mips_ejtag *ejtag_info)
{
	uint32_t ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_PRACC;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
	mips_ejtag_drscan_32_out(ejtag_info, ctrl);

	return jtag_execute_queue();
}

int mips32_pracc_clean_text_jump(struct mips_ejtag *ejtag_info)
{
	uint32_t jt_code = MIPS32_J((0x0FFFFFFF & MIPS32_PRACC_TEXT) >> 2);
	uint32_t data;

	/* do 3 0/nops to clean pipeline before a jump to pracc text, NOP in delay slot */
	for (int i = 0; i < 128; i++) {
		/* Wait for pracc */
		int retval = wait_for_pracc_rw(ejtag_info);
		if (retval != ERROR_OK)
			return retval;

		/* Data or instruction out */
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
		if (i < 3)
			data = MIPS32_NOP;
		else if (i % 2)
			data = jt_code;
		else
			data = MIPS32_NOP;
		mips_ejtag_drscan_32_out(ejtag_info, data);

		/* finish pa */
		retval = mips32_pracc_finish(ejtag_info);
		if (retval != ERROR_OK)
			return retval;

		mips32_pracc_read_ctrl_addr(ejtag_info);
		if (ejtag_info->pa_addr == MIPS32_PRACC_TEXT)
			return ERROR_OK;
	}

	LOG_DEBUG("Can not back to MIPS32_PRACC_TEXT");
	return ERROR_FAIL;
}

int mips32_pracc_exec(struct mips_ejtag *ejtag_info, struct pracc_queue_info *ctx, uint32_t *param_out)
{
	int code_count = 0;
	int store_pending = 0;		/* increases with every store instruction at dmseg, decreases with every store pa */
	uint32_t abandoned_count = 0;
	uint32_t wait_dret_count = 0;
	uint32_t max_store_addr = 0;	/* for store pa address testing */
	uint32_t instr = 0;
	bool final_check = 0;		/* set to 1 if in final checks after function code shifted out */
	int index = 0;
	int retval;

	mips32_pracc_read_ctrl_addr(ejtag_info);
	if (ejtag_info->pa_addr != MIPS32_PRACC_TEXT) { /* restart */
		retval = mips32_pracc_clean_text_jump(ejtag_info);		
		if (retval != ERROR_OK)
			return retval;
	}

	while (1) {
		retval = mips32_pracc_read_ctrl_addr(ejtag_info);		/* update current pa info: control and address */
		if (retval != ERROR_OK)
			return retval;

		/* Check for read or write access */
		if (ejtag_info->pa_ctrl & EJTAG_CTRL_PRNW) {						/* write/store access */
			/* Check for pending store from a previous store instruction at dmseg */
			if (store_pending == 0) {
				LOG_DEBUG("unexpected write at address %" PRIx32, ejtag_info->pa_addr);
				return ERROR_JTAG_DEVICE_ERROR;
			} else {
				/* check address */
				if (ejtag_info->pa_addr < MIPS32_PRACC_PARAM_OUT || ejtag_info->pa_addr > max_store_addr) {
					LOG_DEBUG("writing at unexpected address %" PRIx32, ejtag_info->pa_addr);
					return ERROR_JTAG_DEVICE_ERROR;
				}
			}
			/* read data */
			uint32_t data = 0;
			mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
			retval = mips_ejtag_drscan_32(ejtag_info, &data);
			if (retval != ERROR_OK)
				return retval;

			/* store data at param out, address based offset */
			param_out[(ejtag_info->pa_addr - MIPS32_PRACC_PARAM_OUT) / 4] = data;
			store_pending--;
		} else {					/* read/fetch access */
			if ((code_count != 0) && (ejtag_info->pa_addr == MIPS32_PRACC_TEXT) && (final_check == 0)) {
				final_check = 1;
				code_count = 0;
			}
			if (!final_check) {			/* executing function code */
				/* check address */
				index = (ejtag_info->pa_addr - MIPS32_PRACC_TEXT) / 4;
				if ((code_count == 0) && (ejtag_info->pa_addr != MIPS32_PRACC_TEXT)) {
					LOG_DEBUG("reading at unexpected address %" PRIx32 ", expected %x",
							ejtag_info->pa_addr, MIPS32_PRACC_TEXT);
					return ERROR_JTAG_DEVICE_ERROR;
				}
				if (index < ctx->code_count) {
					instr = ctx->pracc_list[index].instr;
					/* check for store instruction at dmseg */
					uint32_t store_addr = ctx->pracc_list[index].addr;
					if (store_addr != 0) {
						if (store_addr > max_store_addr)
							max_store_addr = store_addr;
						store_pending++;
					}
				} else { /* for fix LSU store delay */
					instr = MIPS32_NOP;
					abandoned_count++;
				}
				code_count++;
				if (code_count > PRACC_MAX_EXEC_CODE_COUNT) {
					LOG_DEBUG("max exec code count is %d", PRACC_MAX_EXEC_CODE_COUNT);
					return ERROR_JTAG_DEVICE_ERROR;
				}
			} else {	/* final check after function code shifted out */
				if (store_pending == 0) {
					if (ejtag_info->pa_addr != MIPS32_PRACC_TEXT) {
						instr = MIPS32_B(NEG16(code_count + 1));
						do {
							mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
							mips_ejtag_drscan_32_out(ejtag_info, instr);
							mips32_pracc_finish(ejtag_info);
							instr = MIPS32_NOP;
							mips32_pracc_read_ctrl_addr(ejtag_info);
						} while (ejtag_info->pa_addr != MIPS32_PRACC_TEXT);
					}
					return ERROR_OK;
				} else { /* for fix LSU store delay */
					instr = MIPS32_NOP;
					abandoned_count++;
					code_count++;
				}
			}
			if (abandoned_count > 256) {
				LOG_DEBUG("execution abandoned, store pending: %d", store_pending);
				return ERROR_JTAG_DEVICE_ERROR;
			}
			/* Send instruction out */
			mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
			mips_ejtag_drscan_32_out(ejtag_info, instr);
		}
		/* finish processor access, let the processor eat! */
		retval = mips32_pracc_finish(ejtag_info);
		if (retval != ERROR_OK)
			return retval;

		if (instr == MIPS32_DRET) {	/* after leaving debug mode and make sure the DRET finish */
			while(1) {
				mips32_pracc_try_read_ctrl_addr(ejtag_info);	/* update current pa info: control and address */
				if (((ejtag_info->pa_ctrl & EJTAG_CTRL_BRKST) == 0) ||
						((ejtag_info->pa_ctrl & EJTAG_CTRL_PRACC) &&
						(ejtag_info->pa_addr == MIPS32_PRACC_TEXT))) {
					return ERROR_OK;
				} else if ((ejtag_info->pa_addr != MIPS32_PRACC_TEXT) &&
						(ejtag_info->pa_ctrl & EJTAG_CTRL_PRACC)) {
					mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
					mips_ejtag_drscan_32_out(ejtag_info, MIPS32_NOP);
					mips32_pracc_finish(ejtag_info);
				}
				wait_dret_count++;
				if (wait_dret_count > 64) {
					LOG_DEBUG("mips32_pracc_finish failed");
					return ERROR_FAIL;
				}
			}
		}
	}
}

inline void pracc_queue_init(struct pracc_queue_info *ctx)
{
	ctx->retval = ERROR_OK;
	ctx->code_count = 0;
	ctx->store_count = 0;
	ctx->max_code = 0;
	ctx->pracc_list = NULL;
}

void pracc_add(struct pracc_queue_info *ctx, uint32_t addr, uint32_t instr)
{
	if (ctx->retval != ERROR_OK)	/* On previous out of memory, return */
		return;
	if (ctx->code_count == ctx->max_code) {
		void *p = realloc(ctx->pracc_list, sizeof(pa_list) * (ctx->max_code + PRACC_BLOCK));
		if (p) {
			ctx->max_code += PRACC_BLOCK;
			ctx->pracc_list = p;
		} else {
			ctx->retval = ERROR_FAIL;	/* Out of memory */
			return;
		}
	}
	ctx->pracc_list[ctx->code_count].instr = instr;
	ctx->pracc_list[ctx->code_count++].addr = addr;
	if (addr)
		ctx->store_count++;
}

void pracc_add_li32(struct pracc_queue_info *ctx, uint32_t reg_num, uint32_t data, bool optimize)
{
	if (LOWER16(data) == 0 && optimize)
		pracc_add(ctx, 0, MIPS32_LUI(reg_num, UPPER16(data)));	/* load only upper value */
	else if (UPPER16(data) == 0 && optimize)
		pracc_add(ctx, 0, MIPS32_ORI(reg_num, 0, LOWER16(data)));	/* load only lower */
	else {
		pracc_add(ctx, 0, MIPS32_LUI(reg_num, UPPER16(data)));		/* load upper and lower */
		pracc_add(ctx, 0, MIPS32_ORI(reg_num, reg_num, LOWER16(data)));
	}
}

inline void pracc_queue_free(struct pracc_queue_info *ctx)
{
	if (ctx->pracc_list != NULL)
		free(ctx->pracc_list);
}

int mips32_pracc_queue_exec(struct mips_ejtag *ejtag_info, struct pracc_queue_info *ctx, uint32_t *buf)
{
	if (ctx->retval != ERROR_OK) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	if (ejtag_info->mode == 0)
		return mips32_pracc_exec(ejtag_info, ctx, buf);

	union scan_in {
		uint8_t scan_96[12];
		struct {
			uint8_t ctrl[4];
			uint8_t data[4];
			uint8_t addr[4];
		} scan_32;

	} *scan_in = malloc(sizeof(union scan_in) * (ctx->code_count + ctx->store_count));
	if (scan_in == NULL) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	unsigned num_clocks =
		((uint64_t)(ejtag_info->scan_delay) * jtag_get_speed_khz() + 500000) / 1000000;

	uint32_t ejtag_ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_PRACC;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ALL);

	int scan_count = 0;
	for (int i = 0; i != ctx->code_count; i++) {
		jtag_add_clocks(num_clocks);
		mips_ejtag_add_scan_96(ejtag_info, ejtag_ctrl, ctx->pracc_list[i].instr,
				       scan_in[scan_count++].scan_96);

		/* Check store address from previous instruction, if not the first */
		if (i > 0 && ctx->pracc_list[i - 1].addr) {
			jtag_add_clocks(num_clocks);
			mips_ejtag_add_scan_96(ejtag_info, ejtag_ctrl, 0, scan_in[scan_count++].scan_96);
		}
	}

	int retval = jtag_execute_queue();		/* execute queued scans */
	if (retval != ERROR_OK)
		goto exit;

	uint32_t fetch_addr = MIPS32_PRACC_TEXT;		/* start address */
	scan_count = 0;
	for (int i = 0; i != ctx->code_count; i++) {				/* verify every pracc access */
		/* check pracc bit */
		ejtag_ctrl = buf_get_u32(scan_in[scan_count].scan_32.ctrl, 0, 32);
		uint32_t addr = buf_get_u32(scan_in[scan_count].scan_32.addr, 0, 32);
		if (!(ejtag_ctrl & EJTAG_CTRL_PRACC)) {
			LOG_ERROR("Error: access not pending  count: %d", scan_count);
			retval = ERROR_FAIL;
			goto exit;
		}
		if (ejtag_ctrl & EJTAG_CTRL_PRNW) {
			LOG_ERROR("Not a fetch/read access, count: %d", scan_count);
			retval = ERROR_FAIL;
			goto exit;
		}
		if (addr != fetch_addr) {
			LOG_ERROR("Fetch addr mismatch, read: %" PRIx32 " expected: %" PRIx32 " count: %d",
					  addr, fetch_addr, scan_count);
			retval = ERROR_FAIL;
			goto exit;
		}
		fetch_addr += 4;
		scan_count++;

		/* check if previous intrucction is a store instruction at dmesg */
		if (i > 0 && ctx->pracc_list[i - 1].addr) {
			uint32_t store_addr = ctx->pracc_list[i - 1].addr;
			ejtag_ctrl = buf_get_u32(scan_in[scan_count].scan_32.ctrl, 0, 32);
			addr = buf_get_u32(scan_in[scan_count].scan_32.addr, 0, 32);

			if (!(ejtag_ctrl & EJTAG_CTRL_PRNW)) {
				LOG_ERROR("Not a store/write access, count: %d", scan_count);
				retval = ERROR_FAIL;
				goto exit;
			}
			if (addr != store_addr) {
				LOG_ERROR("Store address mismatch, read: %" PRIx32 " expected: %" PRIx32 " count: %d",
							      addr, store_addr, scan_count);
				retval = ERROR_FAIL;
				goto exit;
			}
			int buf_index = (addr - MIPS32_PRACC_PARAM_OUT) / 4;
			buf[buf_index] = buf_get_u32(scan_in[scan_count].scan_32.data, 0, 32);
			scan_count++;
		}
	}
exit:
	free(scan_in);
	return retval;
}

int mips32_pracc_read_u32(struct mips_ejtag *ejtag_info, uint32_t addr, uint32_t *buf)
{
	struct pracc_queue_info ctx;
	pracc_queue_init(&ctx);

	pracc_add(&ctx, 0, MIPS32_LUI(15, PRACC_UPPER_BASE_ADDR));			/* $15 = MIPS32_PRACC_BASE_ADDR */
	pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16((addr + 0x8000))));		/* load  $8 with modified upper address */
	pracc_add(&ctx, 0, MIPS32_LW(8, LOWER16(addr), 8));				/* lw $8, LOWER16(addr)($8) */
	pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT,
				MIPS32_SW(8, PRACC_OUT_OFFSET, 15));			/* sw $8,PRACC_OUT_OFFSET($15) */
	pracc_add_li32(&ctx, 8, ejtag_info->reg8, 0);				/* restore $8 */
	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));					/* jump to start */
	pracc_add(&ctx, 0, MIPS32_MFC0(15, 31, 0));					/* move COP0 DeSave to $15 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, buf);
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_pracc_read_mem(struct mips_ejtag *ejtag_info, uint32_t addr, int size, int count, void *buf)
{
	if (count == 1 && size == 4)
		return mips32_pracc_read_u32(ejtag_info, addr, (uint32_t *)buf);

	uint32_t *data = NULL;
	struct pracc_queue_info ctx;
	pracc_queue_init(&ctx);

	if (size != 4) {
		data = malloc(256 * sizeof(uint32_t));
		if (data == NULL) {
			LOG_ERROR("Out of memory");
			goto exit;
		}
	}

	uint32_t *buf32 = buf;
	uint16_t *buf16 = buf;
	uint8_t *buf8 = buf;

	while (count) {
		ctx.code_count = 0;
		ctx.store_count = 0;
		int this_round_count = (count > 256) ? 256 : count;
		uint32_t last_upper_base_addr = UPPER16((addr + 0x8000));

		pracc_add(&ctx, 0, MIPS32_LUI(15, PRACC_UPPER_BASE_ADDR));			/* $15 = MIPS32_PRACC_BASE_ADDR */
		pracc_add(&ctx, 0, MIPS32_LUI(9, last_upper_base_addr));		/* load the upper memory address in $9 */

		for (int i = 0; i != this_round_count; i++) {			/* Main code loop */
			uint32_t upper_base_addr = UPPER16((addr + 0x8000));
			if (last_upper_base_addr != upper_base_addr) {			/* if needed, change upper address in $9 */
				pracc_add(&ctx, 0, MIPS32_LUI(9, upper_base_addr));
				last_upper_base_addr = upper_base_addr;
			}

			if (size == 4)
				pracc_add(&ctx, 0, MIPS32_LW(8, LOWER16(addr), 9));		/* load from memory to $8 */
			else if (size == 2)
				pracc_add(&ctx, 0, MIPS32_LHU(8, LOWER16(addr), 9));
			else
				pracc_add(&ctx, 0, MIPS32_LBU(8, LOWER16(addr), 9));

			pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + i * 4,
					  MIPS32_SW(8, PRACC_OUT_OFFSET + i * 4, 15));		/* store $8 at param out */
			addr += size;
		}
		pracc_add_li32(&ctx, 8, ejtag_info->reg8, 0);				/* restore $8 */
		pracc_add_li32(&ctx, 9, ejtag_info->reg9, 0);				/* restore $9 */

		pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));				/* jump to start */
		pracc_add(&ctx, 0, MIPS32_MFC0(15, 31, 0));					/* restore $15 from DeSave */

		if (size == 4) {
			ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, buf32);
			if (ctx.retval != ERROR_OK)
				goto exit;
			buf32 += this_round_count;
		} else {
			ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, data);
			if (ctx.retval != ERROR_OK)
				goto exit;

			uint32_t *data_p = data;
			for (int i = 0; i != this_round_count; i++) {
				if (size == 2)
					*buf16++ = *data_p++;
				else
					*buf8++ = *data_p++;
			}
		}
		count -= this_round_count;
	}
exit:
	pracc_queue_free(&ctx);
	if (data != NULL)
		free(data);
	return ctx.retval;
}

int mips32_cp0_read(struct mips_ejtag *ejtag_info, uint32_t *val, uint32_t cp0_reg, uint32_t cp0_sel)
{
	struct pracc_queue_info ctx;
	pracc_queue_init(&ctx);

	pracc_add(&ctx, 0, MIPS32_LUI(15, PRACC_UPPER_BASE_ADDR));			/* $15 = MIPS32_PRACC_BASE_ADDR */
	pracc_add(&ctx, 0, MIPS32_MFC0(8, cp0_reg, cp0_sel));			/* move cp0 reg / sel to $8 */
	pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT,
				MIPS32_SW(8, PRACC_OUT_OFFSET, 15));			/* store $8 to pracc_out */
	pracc_add(&ctx, 0, MIPS32_MFC0(15, 31, 0));					/* restore $15 from DeSave */
	pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16(ejtag_info->reg8)));		/* restore upper 16 bits  of $8 */
	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));					/* jump to start */
	pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16(ejtag_info->reg8)));		/* restore lower 16 bits of $8 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, val);
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_cp0_write(struct mips_ejtag *ejtag_info, uint32_t val, uint32_t cp0_reg, uint32_t cp0_sel)
{
	struct pracc_queue_info ctx;
	pracc_queue_init(&ctx);

	pracc_add_li32(&ctx, 15, val, 0);				/* Load val to $15 */

	pracc_add(&ctx, 0, MIPS32_MTC0(15, cp0_reg, cp0_sel));			/* write $15 to cp0 reg / sel */
	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));		/* jump to start */
	pracc_add(&ctx, 0, MIPS32_MFC0(15, 31, 0));				/* restore $15 from DeSave */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL);
	pracc_queue_free(&ctx);
	return ctx.retval;
}

/**
 * \b mips32_pracc_sync_cache
 *
 * Synchronize Caches to Make Instruction Writes Effective
 * (ref. doc. MIPS32 Architecture For Programmers Volume II: The MIPS32 Instruction Set,
 *  Document Number: MD00086, Revision 2.00, June 9, 2003)
 *
 * When the instruction stream is written, the SYNCI instruction should be used
 * in conjunction with other instructions to make the newly-written instructions effective.
 *
 * Explanation :
 * A program that loads another program into memory is actually writing the D- side cache.
 * The instructions it has loaded can't be executed until they reach the I-cache.
 *
 * After the instructions have been written, the loader should arrange
 * to write back any containing D-cache line and invalidate any locations
 * already in the I-cache.
 *
 * If the cache coherency attribute (CCA) is set to zero, it's a write through cache, there is no need
 * to write back.
 *
 * In the latest MIPS32/64 CPUs, MIPS provides the synci instruction,
 * which does the whole job for a cache-line-sized chunk of the memory you just loaded:
 * That is, it arranges a D-cache write-back (if CCA = 3) and an I-cache invalidate.
 *
 * The line size is obtained with the rdhwr SYNCI_Step in release 2 or from cp0 config 1 register in release 1.
 */
static int mips32_pracc_synchronize_cache(struct mips_ejtag *ejtag_info,
					 uint32_t start_addr, uint32_t end_addr, int cached, int rel)
{
	struct pracc_queue_info ctx;
	pracc_queue_init(&ctx);

	/** Find cache line size in bytes */
	uint32_t clsiz;
	/* Ingenic XBurst1 does not support RDHWR, so we can not use it get CACHE_LINE_SIZE */
	if (rel && (ejtag_info->core_type != MIPS_INGENIC_XBURST1)) {	/* Release 2 (rel = 1) */
		pracc_add(&ctx, 0, MIPS32_LUI(15, PRACC_UPPER_BASE_ADDR));	/* $15 = MIPS32_PRACC_BASE_ADDR */

		pracc_add(&ctx, 0, MIPS32_RDHWR(8, MIPS32_SYNCI_STEP));		/* load synci_step value to $8 */

		pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT,
				MIPS32_SW(8, PRACC_OUT_OFFSET, 15));				/* store $8 to pracc_out */

		pracc_add_li32(&ctx, 8, ejtag_info->reg8, 0);				/* restore $8 */

		pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));	/* jump to start */
		pracc_add(&ctx, 0, MIPS32_MFC0(15, 31, 0));					/* move COP0 DeSave to $15 */

		ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, &clsiz);
		if (ctx.retval != ERROR_OK)
			goto exit;

	} else {			/* Release 1 (rel = 0) */
		uint32_t conf;
		ctx.retval = mips32_cp0_read(ejtag_info, &conf, 16, 1);
		if (ctx.retval != ERROR_OK)
			goto exit;

		uint32_t dl = (conf & MIPS32_CONFIG1_DL_MASK) >> MIPS32_CONFIG1_DL_SHIFT;

		/* dl encoding : dl=1 => 4 bytes, dl=2 => 8 bytes, etc... max dl=6 => 128 bytes cache line size */
		clsiz = 0x2 << dl;
		if (dl == 0)
			clsiz = 0;
	}

	if (clsiz == 0)
		goto exit;  /* Nothing to do */

	/* make sure clsiz is power of 2 */
	if (clsiz & (clsiz - 1)) {
		LOG_DEBUG("clsiz must be power of 2");
		ctx.retval = ERROR_FAIL;
		goto exit;
	}

	/* make sure start_addr and end_addr have the same offset inside de cache line */
	start_addr |= clsiz - 1;
	end_addr |= clsiz - 1;

	ctx.code_count = 0;
	int count = 0;
	uint32_t last_upper_base_addr = UPPER16((start_addr + 0x8000));

	pracc_add(&ctx, 0, MIPS32_LUI(15, last_upper_base_addr));		/* load upper memory base address to $15 */

	while (start_addr <= end_addr) {						/* main loop */
		uint32_t upper_base_addr = UPPER16((start_addr + 0x8000));
		if (last_upper_base_addr != upper_base_addr) {				/* if needed, change upper address in $15 */
			pracc_add(&ctx, 0, MIPS32_LUI(15, upper_base_addr));
			last_upper_base_addr = upper_base_addr;
		}
		if (rel)
			pracc_add(&ctx, 0, MIPS32_SYNCI(LOWER16(start_addr), 15));		/* synci instruction, offset($15) */

		else {
			if (cached == 3)
				pracc_add(&ctx, 0, MIPS32_CACHE(MIPS32_CACHE_D_HIT_WRITEBACK,
							LOWER16(start_addr), 15));		/* cache Hit_Writeback_D, offset($15) */

			pracc_add(&ctx, 0, MIPS32_CACHE(MIPS32_CACHE_I_HIT_INVALIDATE,
							LOWER16(start_addr), 15));		/* cache Hit_Invalidate_I, offset($15) */
		}
		start_addr += clsiz;
		count++;
		if (count == 256 && start_addr <= end_addr) {				/* more ?, then execute code list */
			pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));		/* jump to start */
			pracc_add(&ctx, 0, MIPS32_NOP);						/* nop in delay slot */

			ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL);
			if (ctx.retval != ERROR_OK)
				goto exit;

			ctx.code_count = 0;
			count = 0;
		}
	}
	pracc_add(&ctx, 0, MIPS32_SYNC);
	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));					/* jump to start */
	pracc_add(&ctx, 0, MIPS32_MFC0(15, 31, 0));					/* restore $15 from DeSave*/

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL);
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_pracc_invalidate_cache(struct target *target, struct mips_ejtag *ejtag_info, int cache)
{
	uint32_t bpl;
	uint32_t conf;
	struct pracc_queue_info ctx;
	pracc_queue_init(&ctx);

	uint32_t inv_inst_cache[] = {
		/* Determine how big the I$ is */
		MIPS32_MFC0(15, 16, 1),				/* C0_Config1 */  	
		MIPS32_ADDIU(9, 15, 0),
		MIPS32_SRL(9, 15, MIPS32_CONFIG1_IS_SHIFT),
		MIPS32_ANDI(9, 9, 0x7),
		MIPS32_ADDIU(8, 0, 64),				/* li t0, 64 */
		MIPS32_SLLV(9, 8, 9),				/* I$ Sets per way */

		MIPS32_SRL(15, 15, MIPS32_CONFIG1_IA_SHIFT),
		MIPS32_ANDI(15, 15, 0x7),
		MIPS32_ADDIU(15, 15, 1),
		MIPS32_MUL(9, 9, 15),				/* Total number of sets */

		/* Clear TagLo/TagHi registers */
		MIPS32_MTC0(0, 28, 0),				/* C0_ITagLo */
		MIPS32_MTC0(0, 29, 0),				/* C0_ITagHi */
		MIPS32_MFC0(15, 16, 1),				/* Re-read C0_Config1 */

		/* Isolate I$ Line Size */
		MIPS32_ADDIU(8, 0, 2),				/* li a2, 2 */
		MIPS32_SRL(15, 15, MIPS32_CONFIG1_IL_SHIFT),
		MIPS32_ANDI(15, 15, 0x7),

		MIPS32_SLLV(15, 8, 15),				/* Now have true I$ line size in bytes */
		MIPS32_LUI(8, 0x8000),				/* Get a KSeg0 address for cacheops */

		MIPS32_CACHE(MIPS32_CACHE_I_INDEX_STORE_TAG, 0, 8),
		MIPS32_ADDI(9, 9,NEG16(1)),			/* Decrement set counter */
		MIPS32_BNE(9, 0, NEG16(3)),
		MIPS32_ADDU(8, 8, 15),
	};
	uint32_t inv_data_cache[] = {
		MIPS32_MFC0(15, 16, 1),				/* read C0_Config1 */

		MIPS32_SRL(9, 15, MIPS32_CONFIG1_DS_SHIFT),	/* extract DS */
		MIPS32_ANDI(9, 9, 0x7),
		MIPS32_ADDIU(8, 0, 64),				/* li t0, 64 */
		MIPS32_SLLV(9, 8, 9),				/* D$ Sets per way */

		MIPS32_SRL(15, 15, MIPS32_CONFIG1_DA_SHIFT),	/* extract DA */
		MIPS32_ANDI(15, 15, 0x7),
		MIPS32_ADDIU(15, 15, 1),
		MIPS32_MUL(9, 9, 15),				/* Total number of sets */

		/* Clear TagLo/TagHi registers */
		MIPS32_MTC0(0, 28, 0),				/* write C0_TagLo */
		MIPS32_MTC0(0, 29, 0),				/* write C0_TagHi */
		MIPS32_MTC0(0, 28, 2),				/* write C0_DTagLo */
		MIPS32_MTC0(0, 29, 2),				/* write C0_DTagHi */

		/* Isolate D$ Line Size */
		MIPS32_MFC0(15, 16, 1),				/* Re-read C0_Config1 */
		MIPS32_ADDIU(8, 0, 2),				/* li a2, 2 */

		MIPS32_SRL(15, 15, MIPS32_CONFIG1_DL_SHIFT),	/* extract DL */
		MIPS32_ANDI(15, 15, 0x7),

		MIPS32_SLLV(15, 8, 15),				/* Now have true I$ line size in bytes */

		MIPS32_LUI(8, 0x8000)				/* Get a KSeg0 address for cacheops */
	};
	uint32_t inv_L2_cache[] = {
		MIPS32_MFC0(15, 16, 2),				/* read C0_Config2 */

		MIPS32_SRL (9, 15, MIPS32_CONFIG2_SS_SHIFT),	/* extract SS */
		MIPS32_ANDI(9, 9, 0xf),
		MIPS32_ADDIU(8, 0, 64),				/* li t0, 64 */
		MIPS32_SLLV(9, 8, 9),				/* D$ Sets per way */

		MIPS32_SRL(15, 15, MIPS32_CONFIG2_SA_SHIFT),	/* extract DA */
		MIPS32_ANDI(15, 15, 0xf),
		MIPS32_ADDIU(15, 15, 1),
		MIPS32_MUL(9, 9, 15),				/* Total number of sets */

		/* Clear TagLo/TagHi registers */
		MIPS32_MTC0(0, 28, 0),				/* write C0_TagLo */
		MIPS32_MTC0(0, 29, 0),				/* write C0_TagHi */
		MIPS32_MTC0(0, 28, 2),				/* write C0_DTagLo */
		MIPS32_MTC0(0, 29, 2),				/* write C0_DTagHi */

		/* Isolate D$ Line Size */
		MIPS32_MFC0(15, 16, 2),				/* Re-read C0_Config1 */
		MIPS32_ADDIU(8, 0, 2),				/* li a2, 2 */

		MIPS32_SRL(15, 15, MIPS32_CONFIG2_SL_SHIFT),	/* extract DL */
		MIPS32_ANDI(15, 15, 0xf),

		MIPS32_SLLV(15, 8, 15),				/* Now have true I$ line size in bytes */

		MIPS32_LUI(8, 0x8000)				/* Get a KSeg0 address for cacheops */
	};
	uint32_t done[] = {
		MIPS32_LUI(15, UPPER16(MIPS32_PRACC_TEXT)),
		MIPS32_ORI(15, 15, LOWER16(MIPS32_PRACC_TEXT)),
		MIPS32_JR(15),						/* jr start */
		MIPS32_MFC0(15, 31, 0)				/* move COP0 DeSave to $15 */
	};

	/* Read Config1 Register to retrieve cache info */
	if (cache == INSTNOWB || cache == DATA || cache == DATANOWB)
		mips32_cp0_read(ejtag_info, &conf, 16, 1);
	else if (cache == L2 || cache == L2NOWB)
		mips32_cp0_read(ejtag_info, &conf, 16, 2);

	switch (cache) {
		case INSTNOWB:
			/* Extract cache line size */
			bpl = (conf >> MIPS32_CONFIG1_IL_SHIFT) & 7; /* bit 21:19 */

			/* Core configured with Instruction cache */
			if (bpl == 0) {
				LOG_USER("no instructure cache configured");
				ctx.retval = ERROR_OK;
				goto exit;
			}

			for (unsigned i = 0; i < ARRAY_SIZE(inv_inst_cache); i++)
				pracc_add(&ctx, 0, inv_inst_cache[i]);

			break;
		case DATA:
		case DATANOWB:
			/* Extract cache line size */
			bpl = (conf >>  MIPS32_CONFIG1_DL_SHIFT) & 7; /* bit 12:10 */

			/* Core configured with Instruction cache */
			if (bpl == 0) {
				LOG_USER("no data cache configured");
				ctx.retval = ERROR_OK;
				goto exit;
			}

			/* Write exit code */
			for (unsigned i = 0; i < ARRAY_SIZE(inv_data_cache); i++)
				pracc_add(&ctx, 0, inv_data_cache[i]);

			if (cache == DATA)
				pracc_add(&ctx, 0, MIPS32_CACHE(MIPS32_CACHE_D_INDEX_WRITEBACK_INV, 0, 8));
			else if ((cache == ALLNOWB) || (cache == DATANOWB))
				pracc_add(&ctx, 0, MIPS32_CACHE(MIPS32_CACHE_D_INDEX_STORE_TAG, 0, 8));

			pracc_add(&ctx, 0, MIPS32_ADDI(9, 9,NEG16(1)));	/* Decrement set counter */
			pracc_add(&ctx, 0, MIPS32_BNE(9, 0, NEG16(3)));
			pracc_add(&ctx, 0, MIPS32_ADDU(8, 8, 15));
			break;
		case L2:
		case L2NOWB:
			/* Extract cache line size */
			bpl = (conf >>  MIPS32_CONFIG2_SL_SHIFT) & 15; /* bit 7:4 */

			/* Core configured with L2 cache */
			if (bpl == 0) {
				LOG_USER("no L2 cache configured");
				ctx.retval = ERROR_OK;
				goto exit;
 			}

			/* Write exit code */
			for (unsigned i = 0; i < ARRAY_SIZE(inv_L2_cache); i++)
				pracc_add(&ctx, 0, inv_L2_cache[i]);

			if (cache == L2)
				pracc_add(&ctx, 0, MIPS32_CACHE(MIPS32_CACHE_S_INDEX_WRITEBACK_INV, 0, 8));
			else if ((cache == ALLNOWB) || (cache == L2NOWB))
				pracc_add(&ctx, 0, MIPS32_CACHE(MIPS32_CACHE_S_INDEX_STORE_TAG, 0, 8));

			pracc_add(&ctx, 0, MIPS32_ADDI(9, 9,NEG16(1)));// Decrement set counter
			pracc_add(&ctx, 0, MIPS32_BNE(9, 0, NEG16(3)));
			pracc_add(&ctx, 0, MIPS32_ADDU(8, 8, 15));
			break;
	}

	/* Write exit code */
	for (unsigned i = 0; i < ARRAY_SIZE(done); i++)
		pracc_add(&ctx, 0, done[i]);

	/* Start code execution */
	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL);
	if (ctx.retval != ERROR_OK)
		LOG_DEBUG("mips32_pracc_queue_exec failed - ctx.retval: %d", ctx.retval);

exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

static int mips32_pracc_write_mem_generic(struct mips_ejtag *ejtag_info,
		uint32_t addr, int size, int count, const void *buf)
{
	struct pracc_queue_info ctx;
	pracc_queue_init(&ctx);

	const uint32_t *buf32 = buf;
	const uint16_t *buf16 = buf;
	const uint8_t *buf8 = buf;

	while (count) {
		ctx.code_count = 0;
		ctx.store_count = 0;
		int this_round_count = (count > 128) ? 128 : count;
		uint32_t last_upper_base_addr = UPPER16((addr + 0x8000));

		pracc_add(&ctx, 0, MIPS32_LUI(15, last_upper_base_addr));		/* load $15 with memory base address */

		for (int i = 0; i != this_round_count; i++) {
			uint32_t upper_base_addr = UPPER16((addr + 0x8000));
			if (last_upper_base_addr != upper_base_addr) {
				pracc_add(&ctx, 0, MIPS32_LUI(15, upper_base_addr));	/* if needed, change upper address in $15*/
				last_upper_base_addr = upper_base_addr;
			}

			if (size == 4) {
				pracc_add_li32(&ctx, 8, *buf32, 1);			/* load with li32, optimize */
				pracc_add(&ctx, 0, MIPS32_SW(8, LOWER16(addr), 15));		/* store word to memory */
				buf32++;

			} else if (size == 2) {
				pracc_add(&ctx, 0, MIPS32_ORI(8, 0, *buf16));		/* load lower value */
				pracc_add(&ctx, 0, MIPS32_SH(8, LOWER16(addr), 15));	/* store half word to memory */
				buf16++;

			} else {
				pracc_add(&ctx, 0, MIPS32_ORI(8, 0, *buf8));		/* load lower value */
				pracc_add(&ctx, 0, MIPS32_SB(8, LOWER16(addr), 15));	/* store byte to memory */
				buf8++;
			}
			addr += size;
		}

		pracc_add_li32(&ctx, 8, ejtag_info->reg8, 0);				/* restore $8 */

		pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));				/* jump to start */
		pracc_add(&ctx, 0, MIPS32_MFC0(15, 31, 0));				/* restore $15 from DeSave */

		ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL);
		if (ctx.retval != ERROR_OK)
			goto exit;
		count -= this_round_count;
	}
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_pracc_write_mem(struct mips_ejtag *ejtag_info, uint32_t addr, int size, int count, const void *buf)
{
	int retval = mips32_pracc_write_mem_generic(ejtag_info, addr, size, count, buf);
	if (retval != ERROR_OK)
		return retval;

	/*
	 * If we are in the cacheable region and cache is activated,
	 * we must clean D$ (if Cache Coherency Attribute is set to 3) + invalidate I$ after we did the write,
	 * so that changes do not continue to live only in D$ (if CCA = 3), but to be
	 * replicated in I$ also (maybe we wrote the istructions)
	 */
	uint32_t conf = 0;
	int cached = 0;

	if ((KSEGX(addr) == KSEG1) || ((addr >= 0xff200000) && (addr <= 0xff3fffff)))
		return retval; /*Nothing to do*/

	mips32_cp0_read(ejtag_info, &conf, 16, 0);

	switch (KSEGX(addr)) {
		case KUSEG:
			cached = (conf & MIPS32_CONFIG0_KU_MASK) >> MIPS32_CONFIG0_KU_SHIFT;
			break;
		case KSEG0:
			cached = (conf & MIPS32_CONFIG0_K0_MASK) >> MIPS32_CONFIG0_K0_SHIFT;
			break;
		case KSEG2:
		case KSEG3:
			cached = (conf & MIPS32_CONFIG0_K23_MASK) >> MIPS32_CONFIG0_K23_SHIFT;
			break;
		default:
			/* what ? */
			break;
	}

	/*
	 * Check cacheablity bits coherency algorithm
	 * is the region cacheable or uncached.
	 * If cacheable we have to synchronize the cache
	 *
	 * CCA Encoding Description:
	 * 0   000	Cacheable, write-throngh, write-allocate
	 * 1   001	Uncacheable, write accelerated
	 * 2   010	Uncacheable
	 * 3   011	Cacheable, write-back, write-allocate
	 * 4   100	Cacheable, write-throngh, write-allocate, Streaming
	 * 5   101	Cacheable, write-back, write-allocate, Streaming
	 * 6   110	Reserved
	 * 7   111	Reserved
	 */
	if (cached == 0 || cached == 3 || cached == 4 || cached == 5) {
		uint32_t start_addr = addr;
		uint32_t end_addr = addr + count * size;
		uint32_t rel = (conf & MIPS32_CONFIG0_AR_MASK) >> MIPS32_CONFIG0_AR_SHIFT;
		if (rel > 1) {
			LOG_DEBUG("Unknown release in cache code");
			return ERROR_FAIL;
		}
		retval = mips32_pracc_synchronize_cache(ejtag_info, start_addr, end_addr, cached, rel);
	}

	return retval;
}

int mips32_pracc_write_regs(struct mips_ejtag *ejtag_info, uint32_t *regs)
{
	static const uint32_t cp0_write_code[] = {
		MIPS32_MTC0(1, 12, 0),							/* move $1 to status */
		MIPS32_MTLO(1),									/* move $1 to lo */
		MIPS32_MTHI(1),									/* move $1 to hi */
		MIPS32_MTC0(1, 8, 0),							/* move $1 to badvaddr */
		MIPS32_MTC0(1, 13, 0),							/* move $1 to cause*/
		MIPS32_MTC0(1, 24, 0),							/* move $1 to depc (pc) */
	};

	struct pracc_queue_info ctx;
	pracc_queue_init(&ctx);

	/* load registers 2 to 31 with li32, optimize */
	for (int i = 2; i < 32; i++)
		pracc_add_li32(&ctx, i, regs[i], 1);

	for (int i = 0; i != 6; i++) {
		pracc_add_li32(&ctx, 1, regs[i + 32], 0);		/* load CPO value in $1 */
		pracc_add(&ctx, 0, cp0_write_code[i]);			/* write value from $1 to CPO register */
	}
	pracc_add(&ctx, 0, MIPS32_MTC0(15, 31, 0));					/* load $15 in DeSave */
	pracc_add(&ctx, 0, MIPS32_LUI(1, UPPER16((regs[1]))));		/* load upper half word in $1 */
	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));	/* jump to start */
	pracc_add(&ctx, 0, MIPS32_ORI(1, 1, LOWER16((regs[1]))));	/* load lower half word in $1 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL);

	ejtag_info->reg8 = regs[8];
	ejtag_info->reg9 = regs[9];
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_pracc_read_regs(struct mips_ejtag *ejtag_info, uint32_t *regs)
{
	static int cp0_read_code[] = {
		MIPS32_MFC0(8, 12, 0),							/* move status to $8 */
		MIPS32_MFLO(8),									/* move lo to $8 */
		MIPS32_MFHI(8),									/* move hi to $8 */
		MIPS32_MFC0(8, 8, 0),							/* move badvaddr to $8 */
		MIPS32_MFC0(8, 13, 0),							/* move cause to $8 */
		MIPS32_MFC0(8, 24, 0),							/* move depc (pc) to $8 */
	};

	struct pracc_queue_info ctx;
	pracc_queue_init(&ctx);

	pracc_add(&ctx, 0, MIPS32_MTC0(1, 31, 0));					/* move $1 to COP0 DeSave */
	pracc_add(&ctx, 0, MIPS32_LUI(1, PRACC_UPPER_BASE_ADDR));	/* $1 = MIP32_PRACC_BASE_ADDR */

	for (int i = 2; i != 32; i++)						/* store GPR's 2 to 31 */
		pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + (i * 4),
				  MIPS32_SW(i, PRACC_OUT_OFFSET + (i * 4), 1));

	for (int i = 0; i != 6; i++) {
		pracc_add(&ctx, 0, cp0_read_code[i]);					/* load COP0 needed registers to $8 */
		pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + (i + 32) * 4,	/* store $8 at PARAM OUT */
				  MIPS32_SW(8, PRACC_OUT_OFFSET + (i + 32) * 4, 1));
	}
	pracc_add(&ctx, 0, MIPS32_MFC0(8, 31, 0));			/* move DeSave to $8, reg1 value */
	pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + 4,			/* store reg1 value from $8 to param out */
			  MIPS32_SW(8, PRACC_OUT_OFFSET + 4, 1));

	pracc_add(&ctx, 0, MIPS32_MFC0(1, 31, 0));					/* move COP0 DeSave to $1, restore reg1 */
	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));	/* jump to start */
	pracc_add(&ctx, 0, MIPS32_MTC0(15, 31, 0));					/* load $15 in DeSave */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, regs);

	ejtag_info->reg8 = regs[8];	/* reg8 is saved but not restored, next called function should restore it */
	ejtag_info->reg9 = regs[9];
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_pracc_write_fpu_regs(struct mips_ejtag *ejtag_info, uint32_t *regs)
{
	struct pracc_queue_info ctx;
	pracc_queue_init(&ctx);

	/* load f0..f31 */
	for (int i = MIPS32_F0; i < MIPS32_FCSR; i++) {
		pracc_add_li32(&ctx, 8, regs[i - MIPS32_F0], 1);
		pracc_add(&ctx, 0, MIPS32_MTC1(8, (i - MIPS32_F0)));
	}

	/* fcsr */
	pracc_add_li32(&ctx, 8, regs[MIPS32_FCSR - MIPS32_F0], 1);
	pracc_add(&ctx, 0, MIPS32_CTC1(8, 31));

	/* fir */
	pracc_add_li32(&ctx, 8, regs[MIPS32_FIR - MIPS32_F0], 1);
	pracc_add(&ctx, 0, MIPS32_CTC1(8, 0));

	pracc_add(&ctx, 0, MIPS32_MTC0(15, 31, 0));					/* load $15 in DeSave */
	pracc_add(&ctx, 0, MIPS32_LUI(1, UPPER16((regs[1]))));		/* load upper half word in $1 */
	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));	/* jump to start */
	pracc_add(&ctx, 0, MIPS32_ORI(1, 1, LOWER16((regs[1]))));	/* load lower half word in $1 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, regs);

	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_pracc_read_fpu_regs(struct mips_ejtag *ejtag_info, uint32_t *regs)
{
	struct pracc_queue_info ctx;
	pracc_queue_init(&ctx);

	pracc_add(&ctx, 0, MIPS32_MTC0(1, 31, 0));					/* move $1 to COP0 DeSave */
	pracc_add(&ctx, 0, MIPS32_LUI(1, PRACC_UPPER_BASE_ADDR));	/* $1 = MIP32_PRACC_BASE_ADDR */

	for (int i = MIPS32_F0; i != MIPS32_FCSR; i++) {
		pracc_add(&ctx, 0, MIPS32_MFC1(8, (i - MIPS32_F0)));	/* load FP registers to $8 */
		pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + ((i - MIPS32_F0) * 4),	/* store $8 at PARAM OUT */
				  MIPS32_SW(8, PRACC_OUT_OFFSET + ((i - MIPS32_F0) * 4), 1));
	}

	pracc_add(&ctx, 0, MIPS32_CFC1(8, 31));						/* load FCSR registers to $8 */
	pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + ((MIPS32_FCSR - MIPS32_F0) * 4),	/* store $8 at PARAM OUT */
			  MIPS32_SW(8, PRACC_OUT_OFFSET + ((MIPS32_FCSR - MIPS32_F0) * 4), 1));

	pracc_add(&ctx, 0, MIPS32_CFC1(8, 0));						/* load FIR registers to $8 */
	pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + ((MIPS32_FIR - MIPS32_F0) + 1) * 4,	/* store $8 at PARAM OUT */
			  MIPS32_SW(8, PRACC_OUT_OFFSET + ((MIPS32_FIR - MIPS32_F0) + 1) * 4, 1));

	pracc_add(&ctx, 0, MIPS32_MFC0(1, 31, 0));					/* move COP0 DeSave to $1, restore reg1 */
	pracc_add_li32(&ctx, 8, ejtag_info->reg8, 1);

	pracc_add(&ctx, 0, MIPS32_SYNC);
	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));	/* jump to start */

	pracc_add(&ctx, 0, MIPS32_MTC0(15, 31, 0));					/* load $15 in DeSave */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, regs);

	pracc_queue_free(&ctx);
	return ctx.retval;
}

/* fastdata upload/download requires an initialized working area
 * to load the download code; it should not be called otherwise
 * fetch order from the fastdata area
 * 1. start addr
 * 2. end addr
 * 3. data ...
 */
int mips32_pracc_fastdata_xfer(struct mips_ejtag *ejtag_info, struct working_area *source,
		int write_t, uint32_t addr, int count, uint32_t *buf)
{
	int retval;
	uint32_t val;
	uint32_t req_ctrl;
	uint32_t *ack_ctrl = NULL;
	uint32_t handler_code[] = {
		/* r15 points to the start of this code */
		MIPS32_SW(8, MIPS32_FASTDATA_HANDLER_SIZE - 4, 15),
		MIPS32_SW(9, MIPS32_FASTDATA_HANDLER_SIZE - 8, 15),
		MIPS32_SW(10, MIPS32_FASTDATA_HANDLER_SIZE - 12, 15),
		MIPS32_SW(11, MIPS32_FASTDATA_HANDLER_SIZE - 16, 15),
		/* start of fastdata area in t0 */
		MIPS32_LUI(8, UPPER16(MIPS32_PRACC_FASTDATA_AREA)),
		MIPS32_ORI(8, 8, LOWER16(MIPS32_PRACC_FASTDATA_AREA)),
		MIPS32_LW(9, 0, 8),				/* start addr in t1 */
		MIPS32_LW(10, 0, 8),			/* end addr to t2 */
		/* loop: */
		write_t ? MIPS32_LW(11, 0, 8) : MIPS32_LW(11, 0, 9),	/* from xfer area : from memory */
		write_t ? MIPS32_SW(11, 0, 9) : MIPS32_SW(11, 0, 8),	/* to memory      : to xfer area */

		MIPS32_BNE(10, 9, NEG16(3)),	/* bne $t2,t1,loop */
		MIPS32_ADDI(9, 9, 4),			/* addi t1,t1,4 */

		MIPS32_LW(8, MIPS32_FASTDATA_HANDLER_SIZE - 4, 15),
		MIPS32_LW(9, MIPS32_FASTDATA_HANDLER_SIZE - 8, 15),
		MIPS32_LW(10, MIPS32_FASTDATA_HANDLER_SIZE - 12, 15),
		MIPS32_LW(11, MIPS32_FASTDATA_HANDLER_SIZE - 16, 15),

		MIPS32_LUI(15, UPPER16(MIPS32_PRACC_TEXT)),
		MIPS32_ORI(15, 15, LOWER16(MIPS32_PRACC_TEXT)),
		MIPS32_JR(15),					/* jr start */
		MIPS32_MFC0(15, 31, 0),			/* move COP0 DeSave to $15 */
	};

	if (source->size < MIPS32_FASTDATA_HANDLER_SIZE)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	/* write program into RAM */
	if (write_t != ejtag_info->fast_access_save) {
		mips32_pracc_write_mem(ejtag_info, source->address, 4, ARRAY_SIZE(handler_code), handler_code);
		/* save previous operation to speed to any consecutive read/writes */
		ejtag_info->fast_access_save = write_t;
	}

	LOG_DEBUG("%s using 0x%.8" PRIx32 " for write handler", __func__, source->address);

	uint32_t jmp_code[] = {
		MIPS32_LUI(15, UPPER16(source->address)),		/* load addr of jump in $15 */
		MIPS32_ORI(15, 15, LOWER16(source->address)),
		MIPS32_JR(15),					/* jump to ram program */
		MIPS32_NOP,
	};

	/* execute jump code, with no address check */
	for (unsigned i = 0; i < ARRAY_SIZE(jmp_code); i++) {
		retval = wait_for_pracc_rw(ejtag_info);
		if (retval != ERROR_OK)
			return retval;

		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
		mips_ejtag_drscan_32_out(ejtag_info, jmp_code[i]);

		/* Clear the access pending bit (let the processor eat!) */
		uint32_t ejtag_ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_PRACC;
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
		mips_ejtag_drscan_32_out(ejtag_info, ejtag_ctrl);
	}

	while(1) {
		/* wait PrAcc pending bit for FASTDATA write, read address */
		retval = mips32_pracc_read_ctrl_addr(ejtag_info);
		if (retval != ERROR_OK)
			return retval;

		/* next fetch to dmseg should be in FASTDATA_AREA, check */
		if (ejtag_info->pa_addr == MIPS32_PRACC_FASTDATA_AREA)
			break;

		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
		mips_ejtag_drscan_32_out(ejtag_info, MIPS32_NOP);

		/* Clear the access pending bit (let the processor eat!) */
		uint32_t ejtag_ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_PRACC;
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
		mips_ejtag_drscan_32_out(ejtag_info, ejtag_ctrl);
	}

	if (ejtag_info->ejtag_version > EJTAG_VERSION_25) {
		/* Send the load start address */
		val = addr;
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_FASTDATA);
		mips_ejtag_fastdata_scan(ejtag_info, 1, &val);

		retval = wait_for_pracc_rw(ejtag_info);
		if (retval != ERROR_OK)
			return retval;

		/* Send the load end address */
		val = addr + (count - 1) * 4;
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_FASTDATA);
		mips_ejtag_fastdata_scan(ejtag_info, 1, &val);

		unsigned num_clocks = 0;	/* like in legacy code */
		if (ejtag_info->mode != 0)
			num_clocks = ((uint64_t)(ejtag_info->scan_delay) * jtag_get_speed_khz() + 500000) / 1000000;

		for (int i = 0; i < count; i++) {
			jtag_add_clocks(num_clocks);
			mips_ejtag_fastdata_scan(ejtag_info, write_t, buf++);
		}

		retval = jtag_execute_queue();
		if (retval != ERROR_OK) {
			LOG_ERROR("fastdata load failed");
			return retval;
		}
	} else {
		ack_ctrl = malloc((count + 2) * sizeof(uint32_t));
		if (ack_ctrl == NULL) {
			LOG_ERROR("Out of memory");
			return ERROR_FAIL;
		}
		req_ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_PRACC;

		val = addr; /* Send the load start address */
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
		mips_ejtag_add_drscan_32(ejtag_info, val, NULL);
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
		mips_ejtag_add_drscan_32(ejtag_info, req_ctrl, ack_ctrl);

		val = addr + (count - 1) * 4;	/* Send the load end address */
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
		mips_ejtag_add_drscan_32(ejtag_info, val, NULL);
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
		mips_ejtag_add_drscan_32(ejtag_info, req_ctrl, ack_ctrl + 1);

		/* from xfer area to memory */
		/* from memory to xfer area */
		for (int i = 0; i < count; i++) {
			mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
			mips_ejtag_add_drscan_32(ejtag_info, *buf, write_t ? NULL : buf);
			mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
			mips_ejtag_add_drscan_32(ejtag_info, req_ctrl, ack_ctrl + 2 + i);
			buf++;
		}

		retval = jtag_execute_queue();	/* execute queued scans */
		if (retval != ERROR_OK) {
			LOG_ERROR("fastdata load execute queue failed");
			return retval;
		}

		if (debug_level >= LOG_LVL_DEBUG) {	
			for (int i = 0; i < count + 2; i++) {
				if ((ack_ctrl[i] & EJTAG_CTRL_PRACC) == 0) {
					LOG_DEBUG("fastdata load verify failed");
					return ERROR_FAIL;
				}
			}
		}
		free(ack_ctrl);
	}

	retval = mips32_pracc_read_ctrl_addr(ejtag_info);
	if (retval != ERROR_OK)
		return retval;

	if (ejtag_info->pa_addr != MIPS32_PRACC_TEXT)
		LOG_ERROR("mini program did not return to start");

	return retval;
}

int mips32_pracc_fast_exec(struct mips_ejtag *ejtag_info, uint32_t start_addr, const char *exec_f, const char *data_f, const char *arg_l) {
	int retval = ERROR_OK;
	int ret;
	FILE *exec_p = fopen(exec_f, "r");
	FILE *data_p = fopen(data_f, "r");
	uint32_t exec_d, data_d;
	struct stat exec_stat, data_stat;
	uint32_t *buf = NULL;
	uint32_t req_ctrl;
	uint32_t block_size = 4 * 1024 * 1024, count = 0;
	bool data_finish = false, arg_finish = false;
	uint32_t arg_num = 2;

	stat(exec_f, &exec_stat);
	stat(data_f, &data_stat);

	buf = malloc(exec_stat.st_size);
	while(1) {
		ret = fread(&exec_d, 1, 4, exec_p);
		if (ret == EOF)
			break;
		if (ret != 4) {
			retval = ERROR_FAIL;
			free(buf);
			goto exit;
		} else {
			*buf = exec_d;
			buf++;
		}
	}

	/* write program into RAM */
	mips32_pracc_write_mem(ejtag_info, start_addr, 4, exec_stat.st_size/4, buf);
	free(buf);

	uint32_t jmp_code[] = {
		MIPS32_LUI(15, UPPER16(start_addr)),				/* load addr of jump in $15 */
		MIPS32_ORI(15, 15, LOWER16(start_addr)),
		MIPS32_JR(15),										/* jump to ram program */
		MIPS32_NOP,
	};
	/* execute jump code, with no address check */
	for (uint32_t i = 0; i < ARRAY_SIZE(jmp_code); i++) {
		retval = wait_for_pracc_rw(ejtag_info);
		if (retval != ERROR_OK)
			goto exit;

		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
		mips_ejtag_drscan_32_out(ejtag_info, jmp_code[i]);

		/* Clear the access pending bit (let the processor eat!) */
		mips32_pracc_finish(ejtag_info);
	}

	/* next fetch to dmseg should be in FASTDATA_AREA, check */
	while(1) {
		retval = mips32_pracc_read_ctrl_addr(ejtag_info);
		if (retval != ERROR_OK)
			return retval;
		if (ejtag_info->pa_addr == MIPS32_PRACC_FASTDATA_AREA)
			break;
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
		mips_ejtag_drscan_32_out(ejtag_info, MIPS32_NOP);

		/* Clear the access pending bit (let the processor eat!) */
		mips32_pracc_finish(ejtag_info);
	}

	/* send data*/
	buf = malloc(block_size);
	while(1) {
		if (arg_finish) {	
			ret = fread(&data_d, 1, 4, data_p);
			data_finish = ret == EOF;
		} else {
			buf[0] = 0;                  // flash type
			buf[1] = data_stat.st_size;  // data size(byte)
			count = arg_num;
		}
		if ((count == block_size) || (data_finish && (count > 0)) || !arg_finish) {
			req_ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_PRACC;
			for (uint32_t i = 0; i < count; i++) {
				mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
				mips_ejtag_add_drscan_32(ejtag_info, *buf, NULL);
				mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
				mips_ejtag_add_drscan_32(ejtag_info, req_ctrl, NULL);
				buf++;
			}
			count = 0;
			arg_finish = true;
			retval = jtag_execute_queue();/* execute queued scans */
			if (retval != ERROR_OK) {
				LOG_ERROR("fastdata load execute queue failed");
				break;
			}
		} else {
			*(buf + count) = data_d;
			count++;
		}
		if (data_finish)
			break;
	}
	free(buf);

	retval = mips32_pracc_read_ctrl_addr(ejtag_info);
	if (ejtag_info->pa_addr != MIPS32_PRACC_TEXT)
		LOG_ERROR("mini program did not return to start");

exit:
	fclose(data_p);
	fclose(exec_p);

	return retval;
}

