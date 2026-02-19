/*
	Author: bitluni 2019
	License:
	Creative Commons Attribution ShareAlike 4.0
	https://creativecommons.org/licenses/by-sa/4.0/

	For further details check out:
		https://youtube.com/bitlunislab
		https://github.com/bitluni
		http://bitluni.net

	Modified for VGA8: pixel byte is now a pure 8-bit palette index.
	Sync bits are handled at the output stage (VGA/HDMI DMA handlers).
*/
#pragma once
#include "Graphics.h"

class GraphicsR2G2B2S2Swapped: public Graphics<unsigned char>
{
	public:
	typedef unsigned char Color;
	static const Color RGBAXMask = 0xff; // full 8-bit palette index
	Color SBits; // kept for VGA sync generation, not embedded in pixels

	GraphicsR2G2B2S2Swapped()
	{
		SBits = 0xc0;
		frontColor = 0xff;
	}

	// Legacy accessors — no longer meaningful for palette indices
	// but kept for API compatibility (dead code, not called)
	virtual int R(Color c) const
	{
		return (((int)c & 3) * 255 + 1) / 3;
	}
	virtual int G(Color c) const
	{
		return (((int)(c >> 2) & 3) * 255 + 1) / 3;
	}
	virtual int B(Color c) const
	{
		return (((int)(c >> 4) & 3) * 255 + 1) / 3;
	}
	virtual int A(Color c) const
	{
		return (((int)(c >> 6) & 3) * 255 + 1) / 3;
	}

	virtual Color RGBA(int r, int g, int b, int a = 255) const
	{
		return ((r >> 6) & 0b11) | ((g >> 4) & 0b1100) | ((b >> 2) & 0b110000) | (a & 0b11000000);
	}

	virtual void dotFast(int x, int y, Color color)
	{
		frameBuffer[y][x^2] = color;
	}

	virtual void dot(int x, int y, Color color)
	{
		if ((unsigned int)x < xres && (unsigned int)y < yres)
			frameBuffer[y][x^2] = color;
	}

	virtual void dotAdd(int x, int y, Color color)
	{
		if ((unsigned int)x < xres && (unsigned int)y < yres)
			frameBuffer[y][x^2] = color; // simplified — was dead code
	}

	virtual void dotMix(int x, int y, Color color)
	{
		if ((unsigned int)x < xres && (unsigned int)y < yres)
			frameBuffer[y][x^2] = color; // simplified — was dead code
	}

	virtual Color get(int x, int y)
	{
		if ((unsigned int)x < xres && (unsigned int)y < yres)
			return frameBuffer[y][x^2];
		return 0;
	}

	virtual void clear(Color color = 0)
	{
		for (int y = 0; y < this->yres; y++)
			for (int x = 0; x < this->xres; x++)
				frameBuffer[y][x^2] = color;
	}
};
