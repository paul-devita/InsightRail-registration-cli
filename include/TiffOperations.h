// Copyright (c) 2026 INSIGHT RAIL LLC. All rights reserved.
#pragma once

#include <limits>
#include <memory>
#include <string>
#include <eigen3/Eigen/Core>
#include <tiffio.h>



/// All utility tiff operations (saving, loading, opening)
namespace IRTools::tiff
{

int saveTiff16Bit(std::string save_path, unsigned short* height_buffer, int lines, int width);
int saveTiff8Bit(std::string save_path, unsigned char* luminance_buffer, int lines, int width);
int saveCsv(std::string savePath, unsigned short* pwImage, int lines, int width);
int loadTiff16Bit(const std::string& tiff_path, std::unique_ptr<uint16_t[]>& height_buffer,
				  uint32_t& width, Eigen::numext::uint32_t& height);
int loadTiff8Bit(const std::string& tiff_path, std::unique_ptr<uint8_t[]>& height_buffer,
				 uint32_t& width, uint32_t& height);
void openTiff(const std::string& filepath);
} // namespace IRTools::tiff

