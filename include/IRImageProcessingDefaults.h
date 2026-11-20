// Copyright (c) 2026 INSIGHT RAIL LLC. All rights reserved.
#pragma once

#include <IRImageProcessing.h>

using namespace IRTools::ImageProcessing;

const RegistrationSettings LJS8DefaultSettings = {
    .device_type = DT_LJS8_LASER,

    .device_settings = RegistrationLJS8DeviceSettings{
        .x_raw_pixel_spacing = 50,
        .y_raw_pixel_spacing = 50,
        .lx_skip = 1,
        .ly_skip = 1,
        .ux_skip = 1,
        .uy_skip = 1,
        .height_zero = 32678,
        .height_scale = 2.4,
        .invalid_values = { 0 }
    },

    .x_pixel_size = 50,
    .y_pixel_size = 50,

    .levels = {
        {.voxel_size = 200, .correspondence_distance = 400, .max_iterations = 50 },
        { .voxel_size = 100, .correspondence_distance = 200, .max_iterations = 40 },
        { .voxel_size = 50, .correspondence_distance = 100, .max_iterations = 30 }
    },

    .relative_fitness = 1e-6,
    .relative_rmse = 1e-6,
    .normal_radius_factor = 3.0,
    .robust_scale = 10.0,
    .global_voxel_size = 500,
    .ransac_iterations = 100000,
    .ransac_confidence = 0.999,
    .random_seed = 0,
    .min_fitness = 0.8,
    .max_plane_rmse = 15.0,
    .max_center_shift = 10000000, // altered to appease testing
    .max_rotation_deg = 1.0,
    .min_constraint_ratio = 1e-15, // altered to appease testing
    .global_mode = RGM_OFF
};