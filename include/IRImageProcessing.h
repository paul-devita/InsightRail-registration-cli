// Copyright (c) 2026 INSIGHT RAIL LLC. All rights reserved.
#pragma once

#include <open3d/geometry/PointCloud.h>
#include <open3d/pipelines/registration/Registration.h>

#include <string>
#include <vector>
#include <variant>

namespace IRTools::ImageProcessing
{

namespace reg = open3d::pipelines::registration;
namespace geo = open3d::geometry;

enum RegistrationGlobalMode {
    RGM_OFF, // no global registration (RANSAC) is used
    RGM_ALWAYS, // RANSAC is used before ICP
    RGM_ON_FAILURE // RANSAC is only used when ICP fails
};

enum RegistrationInterpolationMode {
    RIM_LINEAR,
    RIM_NEAREST
};

enum DeviceType {
    LJS8_LASER
};

struct LJS8Settings {
    uint16_t x_raw_pixel_spacing; // size of non-skipped pixel spacing in um
    uint16_t y_raw_pixel_spacing; // in other words, the highest image resolution possible
    uint8_t lx_skip;
    uint8_t ly_skip;
    uint8_t ux_skip;
    uint8_t uy_skip;
    uint32_t height_zero;
    float height_scale;
    std::vector<uint16_t> invalid_values;
};

using DeviceSettings =  std::variant<LJS8Settings>;

struct RegistrationLevelSettings {
    uint16_t voxel_size; // voxel sizes in um
    uint16_t correspondence_distance; // correspondence distance in um
    uint16_t max_iterations;
};

struct RegistrationSettings { // todo: add mask?
    DeviceType type;
    DeviceSettings device_settings;
    uint16_t x_pixel_size; // overwritten by device settings
    uint16_t y_pixel_size; // overwritten by device settings
    std::vector<RegistrationLevelSettings> levels; // leave empty for no downsampling/levels
    float relative_fitness;
    float relative_rmse;
    float normal_radius_factor;
    float robust_scale; // loss
    bool loss_enabled;
    float global_voxel_size;
    uint32_t ransac_iterations;
    float ransac_confidence;
    uint32_t random_seed;
    float min_fitness;
    float max_plane_rmse;
    float max_center_shift;
    float max_rotation_deg;
    float min_constraint_ratio;
    float max_angle;
    RegistrationGlobalMode global_mode;
    RegistrationInterpolationMode interpolation_mode;
};

struct LJS8Input {
    std::string source;
    std::string reference;
    std::string luminance;
    bool luminance_valid;
};

using RegistrationInput = std::variant<LJS8Input>;

struct RegistrationResult {
    std::unique_ptr<double[]> heights;
    std::unique_ptr<uint8_t[]> luminance;
    std::unique_ptr<uint8_t[]> valid_bitmap;
    uint32_t width;
    uint32_t height;
    bool luminance_valid;
};

geo::PointCloud smoothSingleCloud(const geo::PointCloud& source, uint32_t size, uint32_t sigma);

RegistrationResult registerSingleCloud(const RegistrationInput& input, RegistrationSettings& settings);


}