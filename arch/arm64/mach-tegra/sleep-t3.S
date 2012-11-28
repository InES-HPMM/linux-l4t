/* FIXME: Implementation TBD */
	.globl tegra3_iram_start
tegra3_iram_start:
	ret

	.globl tegra3_iram_end
tegra3_iram_end:
	b	.

/*
 * tegra3_sleep_core_finish(unsigned long int)
 *
 * enters suspend in LP0 or LP1 by turning off the mmu and jumping to
 * tegra3_tear_down_core in IRAM
 */
	.globl tegra3_sleep_core_finish
tegra3_sleep_core_finish:
	ret

	.globl tegra3_stop_mc_clk_finish
tegra3_stop_mc_clk_finish:
	ret
