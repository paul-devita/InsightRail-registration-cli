// Copyright (c) 2026 INSIGHT RAIL LLC. All rights reserved.

#include "../include/TiffOperations.h"

#include <string>



/**
 * loads a .tiff file of 8-bit data.
 * Updates a buffer of unsigned chars, typically luminance data.
 *
 * @param[in] tiff_path The path for where to read the .tiff file from.
 * @param[out] luminance_buffer The return buffer of unsigned shorts containing luminance data.
 * @return The error code. See IR and Keyence error codes.
 */
int IRTools::tiff::loadTiff8Bit(const std::string& tiff_path,
								std::unique_ptr<uint8_t[]>& height_buffer, uint32_t& width,
								uint32_t& height)
{
	// Open TIFF
	TIFF* tif = TIFFOpen(tiff_path.c_str(), "r");

	if (!tif)
	{
		return -1;
	}

	uint16_t bits_per_sample = 0;
	uint16_t samples_per_pixel = 0;

	if (!TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width) ||
		!TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &height) ||
		!TIFFGetField(tif, TIFFTAG_BITSPERSAMPLE, &bits_per_sample) ||
		!TIFFGetField(tif, TIFFTAG_SAMPLESPERPIXEL, &samples_per_pixel))
	{
		TIFFClose(tif);
		return -1;
	}

	// Validate TIFF format
	if (bits_per_sample != 8 || samples_per_pixel != 1) {
		TIFFClose(tif);
		return -1;
	}

	auto new_height_buffer = std::make_unique<uint8_t[]>(static_cast<size_t>(width) * height);

	// Read directly into flattened destination buffer
	// Layout:
	// [row0][row1][row2]...
	for (uint32_t row = 0; row < height; row++)
	{
		uint8_t* row_ptr = new_height_buffer.get() + static_cast<size_t>(row) * width;

		if (TIFFReadScanline(tif, row_ptr, row) < 0)
		{
			TIFFClose(tif);
			return -1;
		}
	}

	TIFFClose(tif);
	height_buffer = std::move(new_height_buffer);
	return 0;
}

/**
 * loads a .tiff file of 16 bit data.
 * Updates a buffer of unsigned shorts, typically height data.
 *
 * @param[in] tiff_path The path for where to read the .tiff file from.
 * @param[out] height_buffer The return buffer of unsigned shorts containing height data.
 * @param[out] width The width of the loaded tiff image
 * @param[out] height The height of the loaded tiff image
 * @return The error code. See IR and Keyence error codes.
 */
int IRTools::tiff::loadTiff16Bit(const std::string& tiff_path,
								 std::unique_ptr<uint16_t[]>& height_buffer, uint32_t& width,
								 uint32_t& height)
{

	// Open TIFF
	TIFF* tif = TIFFOpen(tiff_path.c_str(), "r");
	if (!tif)
	{
		return -1;
	}

	uint16_t bits_per_sample = 0;
	uint16_t samples_per_pixel = 0;

	if (!TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width) ||
		!TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &height) ||
		!TIFFGetField(tif, TIFFTAG_BITSPERSAMPLE, &bits_per_sample) ||
		!TIFFGetField(tif, TIFFTAG_SAMPLESPERPIXEL, &samples_per_pixel))
	{
		TIFFClose(tif);

		return -1;
	}

	if (bits_per_sample != 16 || samples_per_pixel != 1)
	{
		TIFFClose(tif);

		return -1;
	}

	auto new_height_buffer = std::make_unique<uint16_t[]>(static_cast<size_t>(width) * height);

	for (uint32_t row = 0; row < height; row++)
	{
		uint16_t* row_ptr = new_height_buffer.get() + static_cast<size_t>(row) * width;
		if (TIFFReadScanline(tif, row_ptr, row) < 0)
		{
			TIFFClose(tif);

			return -1;
		}
	}

	TIFFClose(tif);
	height_buffer = std::move(new_height_buffer);

	return 0;
}

/**
 * Saves a .tiff file of 16 bit data.
 * Operates on a buffer of unsigned shorts, typically height data. Utilizes memory mapping
 * to ultra-efficiently map the data to memory.
 *
 * @param[in] save_path The path for where to save the .tiff file.
 * @param[in] height_buffer The buffer of unsigned shorts containing height data.
 * @param[in] lines The number of lines, or rows (profiles in this case) in the data.
 * @param[in] width The number of columns (profile width or x size in this case) in the data.
 * @return The error code. See IR and Keyence error codes.
 */
int IRTools::tiff::saveTiff16Bit(std::string save_path, unsigned short* height_buffer, int lines,
								 int width)
{

	TIFF* tif = TIFFOpen(save_path.c_str(), "w");

	if (!tif)
	{
		return -1;
	}

	TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, width);
	TIFFSetField(tif, TIFFTAG_IMAGELENGTH, lines);
	TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
	TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 16);
	TIFFSetField(tif, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
	TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
	TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);

	for (int row = 0; row < lines; row++)
	{
		TIFFWriteScanline(tif, &height_buffer[row * width], row, 0);
	}

	TIFFClose(tif);

	return 0;
}

/**
 * Saves a .tiff file of 8 bit data.
 * Operates on a buffer of unsigned chars, typically luminance data. Utilizes memory mapping
 * to ultra-efficiently map the data to memory.
 *
 * @param[in] save_path The path for where to save the .tiff file.
 * @param[in] luminance_buffer The buffer of unsigned chars containing luminance data.
 * @param[in] lines The number of lines, or rows (profiles in this case) in the data.
 * @param[in] width The number of columns (profile width or x size in this case) in the data.
 * @return The error code. See IR and Keyence error codes.
 */
int IRTools::tiff::saveTiff8Bit(std::string save_path, unsigned char* luminance_buffer, int lines,
								int width)
{

	TIFF* tif = TIFFOpen(save_path.c_str(), "w");

	if (!tif)
	{
		return -1;
	}

	TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, width);
	TIFFSetField(tif, TIFFTAG_IMAGELENGTH, lines);
	TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
	TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 8);
	TIFFSetField(tif, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
	TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
	TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);

	for (int row = 0; row < lines; row++)
	{
		TIFFWriteScanline(tif, &luminance_buffer[row * width], row, 0);
	}

	TIFFClose(tif);

	return 0;
}