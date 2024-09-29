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
#pragma once
///#include "../Tools/Log.h"
#define DEBUG_PRINTLN(x) { return 0; }
#define ERROR(x) { return 0; }
#define heap_caps_malloc(x, y) malloc(x)

class DMABufferDescriptor /// : protected lldesc_t
{
  public:
	static void *allocateBuffer(int bytes, bool clear = true, unsigned long clearValue = 0)
	{
		bytes = (bytes + 3) & 0xfffffffc;
		void *b = heap_caps_malloc(bytes, MALLOC_CAP_DMA);
		if (!b)
			DEBUG_PRINTLN("Failed to alloc dma buffer");
		if (clear)
			for (int i = 0; i < bytes / 4; i++)
				((unsigned long *)b)[i] = clearValue;
		return b;
	}

	static void **allocateDMABufferArray(int count, int bytes, bool clear = true, unsigned long clearValue = 0)
	{
		if (count == 480) {
			// 480, fake scanlines mode
			void **arr = (void **)malloc(241 * sizeof(void *));
			if(!arr)
				ERROR("Not enough DMA memory");
			for (int i = 0; i < 241; i++)
			{
				arr[i] = DMABufferDescriptor::allocateBuffer(bytes, true, clearValue);
				if(!arr[i])
					ERROR("Not enough DMA memory");
			}
			return arr;
		}
		if (count == 400) {
			// 400, fake scanlines mode
			void **arr = (void **)malloc(201 * sizeof(void *));
			if(!arr)
				ERROR("Not enough DMA memory");
			for (int i = 0; i < 201; i++)
			{
				arr[i] = DMABufferDescriptor::allocateBuffer(bytes, true, clearValue);
				if(!arr[i])
					ERROR("Not enough DMA memory");
			}
			return arr;
		}
		void **arr = (void **)malloc(count * sizeof(void *));
		if(!arr)
			ERROR("Not enough DMA memory");
		for (int i = 0; i < count; i++)
		{
			arr[i] = DMABufferDescriptor::allocateBuffer(bytes, true, clearValue);
			if(!arr[i])
				ERROR("Not enough DMA memory");
		}
		return arr;
	}
/**
	void setBuffer(void *buffer, int bytes)
	{
		length = bytes;
		size = length;
		buf = (uint8_t *)buffer;
	}

	void *buffer() const
	{
		return (void *)buf;
	}

	void init()
	{
		length = 0;
		size = 0;
		owner = 1;
		sosf = 0;
		buf = (uint8_t *)0;
		offset = 0;
		empty = 0;
		eof = 1;
		qe.stqe_next = 0;
	}

	static DMABufferDescriptor *allocateDescriptors(int count) {
		DMABufferDescriptor *b = (DMABufferDescriptor *)heap_caps_malloc(sizeof(DMABufferDescriptor) * count, MALLOC_CAP_DMA);
		if (!b)
			DEBUG_PRINTLN("Failed to alloc DMABufferDescriptors");
		for (int i = 0; i < count; i++)
			b[i].init();
		return b;
	}

	void next(DMABufferDescriptor &next)
	{
		qe.stqe_next = &next;
	}

	int sampleCount() const
	{
		return length / 4;
	}

	void destroy()
	{
		if (buf)
		{
			free((void *)buf);
			buf = 0;
		}
		free(this);
	}
*/
};