#include "pico.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/structs/padsbank0.h"

#include "dvi.h"
#include "dvi_serialiser.h"
#include "dvi_serialiser.pio.h"
#include "dvi_clock.pio.h"

#if defined(USE_PIO_TMDS_ENCODE) || !defined (DVI_USE_PIO_CLOCK)
#define USE_PWM_CLOCK
#endif

#ifndef USE_PWM_CLOCK
static int clk_sm = 0;
#endif

static void dvi_configure_pad(uint gpio, bool invert) {
	// 2 mA drive, enable slew rate limiting (this seems fine even at 720p30, and
	// the 3V3 LDO doesn't get warm like when turning all the GPIOs up to 11).
	// Also disable digital receiver.
	hw_write_masked(
		&padsbank0_hw->io[gpio],
		(0 << PADS_BANK0_GPIO0_DRIVE_LSB),
		PADS_BANK0_GPIO0_DRIVE_BITS | PADS_BANK0_GPIO0_SLEWFAST_BITS | PADS_BANK0_GPIO0_IE_BITS
	);
	gpio_set_outover(gpio, invert ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
}

void dvi_serialiser_init(struct dvi_serialiser_cfg *cfg, const struct dvi_timing *t) {
#if DVI_SERIAL_DEBUG
	uint offset = pio_add_program(cfg->pio, &dvi_serialiser_debug_program);
#else
	uint offset = pio_add_program(cfg->pio, &dvi_serialiser_program);
#endif
	cfg->prog_offs = offset;

	for (int i = 0; i < N_TMDS_LANES; ++i) {
		pio_sm_claim(cfg->pio, cfg->sm_tmds[i]);
		dvi_serialiser_program_init(
			cfg->pio,
			cfg->sm_tmds[i],
			offset,
			cfg->pins_tmds[i],
			DVI_SERIAL_DEBUG,
			t->bit_clk_khz * 1000
		);
		dvi_configure_pad(cfg->pins_tmds[i], cfg->invert_diffpairs);
		dvi_configure_pad(cfg->pins_tmds[i] + 1, cfg->invert_diffpairs);
	}

#ifdef USE_PWM_CLOCK
	// Use a PWM slice to drive the pixel clock. Both GPIOs must be on the same
	// slice (lower-numbered GPIO must be even).
	assert(cfg->pins_clk % 2 == 0);
	uint slice = pwm_gpio_to_slice_num(cfg->pins_clk);
	// 5 cycles high, 5 low. Invert one channel so that we get complementary outputs.
	pwm_config pwm_cfg = pwm_get_default_config();
	pwm_config_set_output_polarity(&pwm_cfg, true, false);
	uint32_t wrap = clock_get_hz(clk_sys) / (t->bit_clk_khz * 100) - 1;
	pwm_config_set_wrap(&pwm_cfg, wrap);
	pwm_init(slice, &pwm_cfg, false);
	pwm_set_both_levels(slice, (wrap + 1) / 2, (wrap + 1) / 2);
#else
	// Use a state machine to generate the clock
	clk_sm = pio_claim_unused_sm(cfg->pio, true);
    offset = pio_add_program(cfg->pio, &dvi_clock_program);
	dvi_clock_program_init(cfg->pio, clk_sm, offset, cfg->pins_clk);
#endif

	for (uint i = cfg->pins_clk; i <= cfg->pins_clk + 1; ++i) {
#ifdef USE_PWM_CLOCK
		gpio_set_function(i, GPIO_FUNC_PWM);
#endif
		dvi_configure_pad(i, cfg->invert_diffpairs);
	}
}

void dvi_serialiser_enable(struct dvi_serialiser_cfg *cfg, bool enable) {
	uint mask = 0;
	for (int i = 0; i < N_TMDS_LANES; ++i)
		mask |= 1u << (cfg->sm_tmds[i] + PIO_CTRL_SM_ENABLE_LSB);
	if (enable) {
		// The DVI spec allows for phase offset between clock and data links.
		// So PWM and PIO do not need to be synchronised perfectly.
		hw_set_bits(&cfg->pio->ctrl, mask);
#ifdef USE_PWM_CLOCK
		pwm_set_enabled(pwm_gpio_to_slice_num(cfg->pins_clk), true);
#else
    	pio_sm_set_enabled(cfg->pio, clk_sm, true);
#endif
	}
	else {
		hw_clear_bits(&cfg->pio->ctrl, mask);
#ifdef USE_PWM_CLOCK
		pwm_set_enabled(pwm_gpio_to_slice_num(cfg->pins_clk), false);
#else
    	pio_sm_set_enabled(cfg->pio, clk_sm, false);
#endif
	}
}
