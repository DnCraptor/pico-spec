/*
	Author: bitluni 2019
	License: 
	Creative Commons Attribution ShareAlike 4.0
	https://creativecommons.org/licenses/by-sa/4.0/
	
	For further details check out: 
		https://youtube.com/bitlunislab
		https://github.com/bitluni
		http://bitluni.net
*/
#include "I2S.h"
///#include "../Tools/Log.h"
#include "../VGA/VGA.h"
///#include <soc/rtc.h>
///#include <driver/rtc_io.h>
#include "Config.h"
#define IRAM_ATTR 

///i2s_dev_t *i2sDevices[] = {&I2S0, &I2S1};

I2S::I2S(const int i2sIndex) : dmaBufferDescriptorCount(0), dmaBufferDescriptorActive(0), stopSignal(false)
{
	/**
	const periph_module_t deviceModule[] = {PERIPH_I2S0_MODULE, PERIPH_I2S1_MODULE};
	this->i2sIndex = i2sIndex;
	//enable I2S peripheral
	periph_module_enable(deviceModule[i2sIndex]);
	interruptHandle = 0;
	dmaBufferDescriptors = 0;
	*/
}

IRAM_ATTR void I2S::interruptStatic(void *arg)
{
	/**
	volatile i2s_dev_t &i2s = *i2sDevices[((I2S *)arg)->i2sIndex];
	//i2s object not safely accesed in DRAM or IRAM
	//i2s.int_clr.val = i2s.int_raw.val;
	//using REG_WRITE to clear the interrupt instead
	//note: there are still other alternatives, see i2s driver .c file
	//inside the i2s_intr_handler_default() function
	REG_WRITE(I2S_INT_CLR_REG(((I2S *)arg)->i2sIndex), (REG_READ(I2S_INT_RAW_REG(((I2S *)arg)->i2sIndex)) & 0xffffffc0) | 0x3f);
	//the call to the overloaded (or any) non-static member function definitely breaks the IRAM rule
	// causing an exception when concurrently accessing the flash (or flash-filesystem) or wifi
	//the reason is unknown but probably related with the compiler instantiation mechanism
	//(note: defining the code of the [member] interrupt function outside the class declaration,
	// and with IRAM flag does not avoid the crash)
	//((I2S *)arg)->interrupt();
	*/
	if(((I2S *)arg)->interruptStaticChild)
		((I2S *)arg)->interruptStaticChild(arg);
}

void I2S::reset()
{
	/**
	volatile i2s_dev_t &i2s = *i2sDevices[i2sIndex];
	const unsigned long lc_conf_reset_flags = I2S_IN_RST_M | I2S_OUT_RST_M | I2S_AHBM_RST_M | I2S_AHBM_FIFO_RST_M;
	i2s.lc_conf.val |= lc_conf_reset_flags;
	i2s.lc_conf.val &= ~lc_conf_reset_flags;

	const uint32_t conf_reset_flags = I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M | I2S_TX_RESET_M | I2S_TX_FIFO_RESET_M;
	i2s.conf.val |= conf_reset_flags;
	i2s.conf.val &= ~conf_reset_flags;
	while (i2s.state.rx_fifo_reset_back)
		;
		*/
}

void I2S::i2sStop()
{
	/**
	volatile i2s_dev_t &i2s = *i2sDevices[i2sIndex];
	esp_intr_disable(interruptHandle);
	reset();
	i2s.conf.rx_start = 0;
	i2s.conf.tx_start = 0;
	*/
}

void I2S::startTX()
{
	/**
	volatile i2s_dev_t &i2s = *i2sDevices[i2sIndex];
	// DEBUG_PRINTLN("I2S TX");
	esp_intr_disable(interruptHandle);
	reset();
    i2s.lc_conf.val    = I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN;
	dmaBufferDescriptorActive = 0;
	i2s.out_link.addr = (uint32_t)firstDescriptorAddress();
	i2s.out_link.start = 1;
	i2s.int_clr.val = i2s.int_raw.val;
	i2s.int_ena.val = 0;
	if(useInterrupt())
	{
		i2s.int_ena.out_eof = 1;
		//enable interrupt
		esp_intr_enable(interruptHandle);
	}
	//start transmission
	i2s.conf.tx_start = 1;
	*/
}

void I2S::resetDMA()
{
	/**
	volatile i2s_dev_t &i2s = *i2sDevices[i2sIndex];
	i2s.lc_conf.in_rst = 1;
	i2s.lc_conf.in_rst = 0;
	i2s.lc_conf.out_rst = 1;
	i2s.lc_conf.out_rst = 0;
	*/
}

void I2S::resetFIFO()
{
	/**
	volatile i2s_dev_t &i2s = *i2sDevices[i2sIndex];
	i2s.conf.rx_fifo_reset = 1;
	i2s.conf.rx_fifo_reset = 0;
	i2s.conf.tx_fifo_reset = 1;
	i2s.conf.tx_fifo_reset = 0;
	*/
}

/**
DMABufferDescriptor *I2S::firstDescriptorAddress() const
{
	return &dmaBufferDescriptors[0];
}
*/
bool I2S::useInterrupt()
{ 
	return false; 
};

bool I2S::initParallelOutputMode(const int *pinMap, int mode, const int bitCount, int wordSelect, int baseClock)
{
	/**
	volatile i2s_dev_t &i2s = *i2sDevices[i2sIndex];
	//route peripherals
	//in parallel mode only upper 16 bits are interesting in this case
	const int deviceBaseIndex[] = {I2S0O_DATA_OUT0_IDX, I2S1O_DATA_OUT0_IDX};
	const int deviceClockIndex[] = {I2S0O_BCK_OUT_IDX, I2S1O_BCK_OUT_IDX};
	const int deviceWordSelectIndex[] = {I2S0O_WS_OUT_IDX, I2S1O_WS_OUT_IDX};
	const periph_module_t deviceModule[] = {PERIPH_I2S0_MODULE, PERIPH_I2S1_MODULE};
	//works only since indices of the pads are sequential
	for (int i = 0; i < bitCount; i++)
		if (pinMap[i] > -1)
		{
			PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[pinMap[i]], PIN_FUNC_GPIO);
			gpio_set_direction((gpio_num_t)pinMap[i], (gpio_mode_t)GPIO_MODE_DEF_OUTPUT);
			//rtc_gpio_set_drive_capability((gpio_num_t)pinMap[i], (gpio_drive_cap_t)GPIO_DRIVE_CAP_3 );
			if(i2sIndex == 1)
			{
				if(bitCount == 16)
					gpio_matrix_out(pinMap[i], deviceBaseIndex[i2sIndex] + i + 8, false, false);
				else
					gpio_matrix_out(pinMap[i], deviceBaseIndex[i2sIndex] + i, false, false);
			}
			else
			{
				//there is something odd going on here in the two different I2S
				//the configuration seems to differ. Use i2s1 for high frequencies.
				gpio_matrix_out(pinMap[i], deviceBaseIndex[i2sIndex] + i + 24 - bitCount, false, false);
			}
		}
	if (baseClock > -1)
		gpio_matrix_out(baseClock, deviceClockIndex[i2sIndex], false, false);
	if (wordSelect > -1)
		gpio_matrix_out(wordSelect, deviceWordSelectIndex[i2sIndex], false, false);

	//enable I2S peripheral
	periph_module_enable(deviceModule[i2sIndex]);

	//reset i2s
	i2s.conf.tx_reset = 1;
	i2s.conf.tx_reset = 0;
	i2s.conf.rx_reset = 1;
	i2s.conf.rx_reset = 0;

	resetFIFO();
	resetDMA();

	//parallel mode
	i2s.conf2.val = 0;
	i2s.conf2.lcd_en = 1;
	//from technical datasheet figure 64
	i2s.conf2.lcd_tx_wrx2_en = 1;
	i2s.conf2.lcd_tx_sdx2_en = 0;

	i2s.sample_rate_conf.val = 0;
	i2s.sample_rate_conf.tx_bits_mod = bitCount;
	//clock setup
	int clockN = 2, clockA = 1, clockB = 0, clockDiv = 1;

	if (Config::esp32rev > 0)
	{
		// ESP32 chip revision > 0
		rtc_clk_apll_enable(true, vidmodes[mode][vmodeproperties::r1sdm0], vidmodes[mode][vmodeproperties::r1sdm1], vidmodes[mode][vmodeproperties::r1sdm2], vidmodes[mode][vmodeproperties::r1odiv]);
	}
	else
	{
		// ESP32 chip revision == 0
		rtc_clk_apll_enable(true, 0, 0, vidmodes[mode][vmodeproperties::r0sdm2], vidmodes[mode][vmodeproperties::r0odiv]);
	}

	i2s.clkm_conf.val = 0;
	i2s.clkm_conf.clka_en = 1;
	i2s.clkm_conf.clkm_div_num = clockN;
	i2s.clkm_conf.clkm_div_a = clockA;
	i2s.clkm_conf.clkm_div_b = clockB;
	i2s.sample_rate_conf.tx_bck_div_num = clockDiv;

	i2s.fifo_conf.val = 0;
	i2s.fifo_conf.tx_fifo_mod_force_en = 1;
	i2s.fifo_conf.tx_fifo_mod = 1;  //byte packing 0A0B_0B0C = 0, 0A0B_0C0D = 1, 0A00_0B00 = 3,
	i2s.fifo_conf.tx_data_num = 32; //fifo length
	i2s.fifo_conf.dscr_en = 1;		//fifo will use dma

	i2s.conf1.val = 0;
	i2s.conf1.tx_stop_en = 0;
	i2s.conf1.tx_pcm_bypass = 1;

	i2s.conf_chan.val = 0;
	i2s.conf_chan.tx_chan_mod = 1;

	//high or low (stereo word order)
	i2s.conf.tx_right_first = 1;

	i2s.timing.val = 0;

	//clear serial mode flags
	i2s.conf.tx_msb_right = 0;
	i2s.conf.tx_msb_shift = 0;
	i2s.conf.tx_mono = 0;
	i2s.conf.tx_short_sync = 0;

	//allocate disabled i2s interrupt
	const int interruptSource[] = {ETS_I2S0_INTR_SOURCE, ETS_I2S1_INTR_SOURCE};
	if(useInterrupt())
		esp_intr_alloc(interruptSource[i2sIndex], ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM, &interruptStatic, this, &interruptHandle);
	*/
	return true;
}

void I2S::stop()
{
	stopSignal = true;
	while (stopSignal)
		;
}
