#include <IRImageProcessing.h>
#include <TiffOperations.h>

#include <eigen3/Eigen/Core>
#include <Eigen/Eigenvalues>

#include <open3d/Open3D.h>

#include <memory>
#include <numbers>
#include <vector>
#include <thread>


using namespace IRTools::ImageProcessing;

// types to improve readability and reduce verboseness
typedef geo::PointCloud PointCloud;
typedef std::shared_ptr<geo::PointCloud> PointCloudSP;
typedef Eigen::Matrix4d Mat4d;
typedef geo::KDTreeSearchParamHybrid KDSearchParam;

namespace {
    struct RegistrationLjs8PointCloud {
        PointCloud cloud;
        uint32_t width;
        uint32_t height;
    };

    struct RegistrationLuminanceData {
        const uint8_t* data;
        const bool* valid_map;
        uint32_t width;
        uint32_t height;
        bool valid;
    };
    struct RegistrationGridSpec {
        uint32_t height;
        uint32_t width;
        double x_pixel_spacing;
        double y_pixel_spacing;
    };
}

/**
 * @brief Checks every setting value
 *
 * @param[in] settings The settings to be checked and verified
 *
 * @exception RegistrationSettingsError Thrown when settings are invalid in any way,
 * which includes but is not limited to non-finite/NaN values, values outside a given range, missing values.
 * */
static void verifyRegistrationSettings(const RegistrationSettings& settings);

/**
 * @brief Converts the data received from an LJS8 Laser to an Open3D point cloud of Eigen::Vector3d
 *
 * @param[in] fp String filepath pointing to the input tiff file.
 * @param[in] settings Settings utilized to select device type. Device settings are used to verify that the correct device input is being accessed.
 *
 * @exception RegistrationLJS8DataError Thrown when the input tiff cannot be read, when tiff dimensions are invalid, or when
 * the function fails to construct/allocate the resulting point cloud.
 *
 * @return A RegistrationLjs8PointCloud structure containing:
 * - 'cloud': Open3D point cloud populated with 3D point vectors
 * - 'width': Width of the grid, in pixels. Guaranteed to be non-zero.
 * - 'height': Height of the grid, in pixels. Guaranteed to be non-zero.
 */
static RegistrationLjs8PointCloud ljs8DataToPointCloud(const std::string& fp, const LJS8Settings& settings);

/**
 * @brief Utilizes Iterative Closest Point (ICP) to align two point clouds. Iterates from smaller, more downsampled point
 * clouds to larger, higher-resolution point clouds, improving the transformation estimate at each level.
 *
 * @param t Starting 4x4 transformation matrix.
 * @param src_levels Vector of downsampled source point clouds.
 * @param ref_levels Vector of downsampled reference point clouds.
 * @param settings Settings utilized during the transformation.
 *
 * @exception RegistrationICPError Thrown when the ICP result fails validation.
 * @exception RegistrationCheckICPFailure Thrown when a failure occurs when checking the ICP transformation result.
 *
 * @return An Eigen::Matrix4d representing the new estimated transform as a result of ICP.
 */
static Mat4d refineTransformICP(const Mat4d& t, const std::vector<PointCloudSP>& src_levels, const std::vector<PointCloudSP>& ref_levels, const RegistrationSettings& settings);


static Mat4d globalTransformRANSAC(const PointCloudSP& source, const PointCloudSP& target, const RegistrationSettings& settings);

/**
 * @brief Verifies the transformation generated via ICP, throwing exceptions to indicate that the transformation does not meet the standards/thresholds specified in settings
 *
 * @param transform Transform generated during ICP that will be verified.
 * @param source Source point cloud used to generate the transform.
 * @param reference Reference point cloud used to generate the transform.
 * @param settings Settings that dictate the metrics/thresholds at which the transformation will be verified.
 *
 * @exception RegistrationICPError Thrown when the ICP result fails validation.
 * @exception RegistrationCheckICPFailure Thrown when a failure occurs when checking the ICP transformation result.
 */
static void checkIcpTransform(const Mat4d& transform, const PointCloudSP& source, const PointCloudSP& reference, const RegistrationSettings& settings);


static RegistrationResult regridHeight(PointCloud& source, const Mat4d& transform, const RegistrationGridSpec& grid_spec, const RegistrationLuminanceData& luminance_data);

PointCloud IRTools::ImageProcessing::smoothSingleCloud(const geo::PointCloud& source, const uint32_t size, const uint32_t sigma) {
    // set up temporary point structure and hash function

    // set up structure to store hashed data for
    //  1) index into point cloud for z value
    //  2) index into array of G vals

    // normalize the weights

    // go through each point, keeping track of
    //  1. max/min x and y values
    //  2. hashing each (x,y) pair to an index for fast access

    // do the fast Gaussian smoothing, single pass on one axis, store the intermediate result, then do another pass on the other axis using the intermediate values

    // return the new smoothed point cloud

    return PointCloud();
}

RegistrationResult IRTools::ImageProcessing::registerSingleCloud(const RegistrationInput& input, RegistrationSettings& settings) {
    // first verify settings
    verifyRegistrationSettings(settings);

    // get device type, designate the source and reference point clouds, and designate grid specifications
    const DeviceType dt = settings.device_type;
    PointCloud source_pc, reference_pc;
    RegistrationGridSpec grid_spec{};

    // luminance data if it is available
    RegistrationLuminanceData luminance_data{};

    // assemble point clouds based on the type of the data
    switch (dt) {
        case DT_LJS8_LASER: {
            // get the laser-specific settings from RegistrationSettings
            const LJS8Settings& ljs8_settings = std::get<LJS8Settings>(settings.device_settings);

            // verify and retrieve the ljs8 input
            if (!std::holds_alternative<LJS8Input>(input))
                throw RegistrationLJS8DataError("invalid input type for LJS8 data input");
            const auto& ljs8_input = std::get<LJS8Input>(input);

            // save each point cloud
            const auto [source_cloud, source_width, source_height] = ljs8DataToPointCloud(ljs8_input.source, ljs8_settings);
            const auto [reference_cloud, reference_width, reference_height] = ljs8DataToPointCloud(ljs8_input.reference, ljs8_settings);

            // ensure that they have the same dimensions
            if (source_width != reference_width || source_height != reference_height) {
                throw RegistrationInputError("source and reference grids have different dimensions");
            }

            // populate the grid specification, each is verified to be non-zero
            grid_spec.width = reference_width;
            grid_spec.height = reference_height;
            // conversion from uint8_t to double will never produce NaN, pixel spacings are verified before to not be NaN, so values only need to be checked for non-finite values
            grid_spec.x_pixel_spacing = static_cast<double>(ljs8_settings.lx_skip) * static_cast<double>(ljs8_settings.ux_skip) * ljs8_settings.x_raw_pixel_spacing;
            grid_spec.y_pixel_spacing = static_cast<double>(ljs8_settings.ly_skip) * static_cast<double>(ljs8_settings.uy_skip) * ljs8_settings.y_raw_pixel_spacing;

            // ensure that the pixel spacing is valid
            if (grid_spec.x_pixel_spacing < 0 || grid_spec.y_pixel_spacing < 0) {
                throw RegistrationInputError("invalid negative pixel spacing");
            }
            if (Eigen::internal::isApprox(grid_spec.x_pixel_spacing, 0.0) || Eigen::internal::isApprox(grid_spec.y_pixel_spacing, 0.0)) {
                throw RegistrationInputError("invalid zero pixel spacing");
            }

            // update the settings pixel spacing to match
            settings.x_pixel_size = grid_spec.x_pixel_spacing;
            settings.y_pixel_size = grid_spec.y_pixel_spacing;

            // ensure there is a minimum number of points in either cloud
            if (source_pc.points_.size() < settings.min_point_count || reference_pc.points_.size() < settings.min_point_count) {
                throw RegistrationInputError("point cloud size does not meet minimum count");
            }

            // set the source and reference point clouds
            source_pc = source_cloud;
            reference_pc = reference_cloud;

            break;
        }
        default:
            throw RegistrationInputError("invalid device type");
    }

    // must provide voxel levels
    if (settings.levels.empty()) {
        throw RegistrationInputError("invalid voxel levels (levels list is empty)");
    }

    // must be a vector of shared_ptr, prevents unnecessary copying of point cloud data
    std::vector<PointCloudSP> src_levels, ref_levels;

    // process voxel levels for registration
    const size_t voxel_count = settings.levels.size();

    // reserve space for the point clouds, use resize to ensure the vector size is actually changed
    src_levels.resize(voxel_count);
    ref_levels.resize(voxel_count);

    // function for a thread to execute, updates the ith element in either levels list
    const auto processLevel = [](std::vector<PointCloudSP>& dest, const size_t i, const float voxel, const float radius_factor, const PointCloud& original_cloud,
        const uint32_t neighbors, const bool is_ref) -> void {
        /* Assumptions about input, verified by previous checks:
         *  dest        - allocated with resize, allocation is always successful when program reaches this point
         *  i           - always ranged between 0 and voxel_count, dest is always resized using voxel_count
         *  voxel       - verified with settings verification function
         *  radius      - verified with settings verification function
         *  original    - verified with raw data to point cloud function
         *  iterations  - verified with settings verification function
         */

        DEBUG_LOG("REG: estimating normals for " << (is_ref ? "reference" : "source") << " level " << i << " at voxel scale " << voxel);

        // calculate the downsampled point cloud
        const PointCloudSP downsampled = original_cloud.VoxelDownSample(voxel);

        // estimate the normals for the downsampled point cloud
        downsampled->EstimateNormals(KDSearchParam(voxel * radius_factor, static_cast<int>(neighbors)), true);

        // set the designated index in the list to be the updated point cloud
        dest.at(i) = downsampled;
    };

    // for each voxel size, generate a point cloud and add it to the list, in order
    for (size_t i = 0; i < voxel_count; i++) {
        // maximum number of neighbors to sample for normal estimation
        constexpr uint32_t MAX_NN_NORMALS = 30;

        // the voxel size at this level
        const double voxel = settings.levels.at(i).voxel_size;

        // spawn thread for processing the source point cloud at the current voxel level
        DEBUG_LOG("REG: spawning thread to process source level " << i << " with voxel scale " << voxel);
        std::thread source_thread(
            processLevel,std::ref(src_levels), i, voxel, settings.normal_radius_factor, std::cref(source_pc), MAX_NN_NORMALS, false
        );
        // spawn thread for processing the reference point cloud at the current voxel level
        DEBUG_LOG("REG: spawning thread to process refernce level " << i << " with voxel scale " << voxel);
        std::thread reference_thread(
            processLevel,std::ref(ref_levels), i, voxel, settings.normal_radius_factor, std::cref(reference_pc), MAX_NN_NORMALS, true
        );

        // join the worker threads
        if (source_thread.joinable())
            source_thread.join();
        if (reference_thread.joinable())
            reference_thread.join();
        DEBUG_LOG("REG: joined threads for level " << i);
    }

    // generate transformation matrix
    Mat4d transform;

    switch (settings.global_mode) {
        case RGM_OFF: // no RANSAC, exceptions just ripple up to the caller
            DEBUG_LOG("REG: running ICP with global mode: OFF");
            try {
                transform = refineTransformICP(Eigen::Matrix4d::Identity(), src_levels, ref_levels, settings);
            }
            catch (std::exception& e) {
                // report the error
                std::cerr << "ICP ERROR: " << e.what() << std::endl;

                // let the error ripple up
                throw e;
            }
            break;
        case RGM_ALWAYS: // RANSAC before ICP, exceptions just ripple up to the caller
            // perform RANSAC registration
            DEBUG_LOG("REG: running RANSAC with global mode: ALWAYS");
            transform = globalTransformRANSAC(src_levels.at(0), ref_levels.at(0), settings);

            // use RANSAC registration as starting point for ICP
            DEBUG_LOG("REG: running ICP with global mode: ALWAYS");
            transform = refineTransformICP(transform, src_levels, ref_levels, settings);

            break;
        case RGM_ON_FAILURE: // RANSAC only when ICP fails
            // try to perform ICP registration
            try {
                DEBUG_LOG("REG: running ICP with global mode: ON FAILURE");
                transform = refineTransformICP(Eigen::Matrix4d::Identity(), src_levels, ref_levels, settings);
            }
            catch (std::exception& e) {
                // report the errors
                std::cerr << "ICP ERROR: " << e.what() << std::endl;
                std::cout << "ICP failed: retrying with initial RANSAC transformation . . ." << std::endl;

                // perform RANSAC registration
                transform = globalTransformRANSAC(src_levels.at(0), ref_levels.at(0), settings);

                // use RANSAC registration as starting point for ICP
                transform = refineTransformICP(transform, src_levels, ref_levels, settings);
            }

            break;
        default:
            throw std::runtime_error("invalid global mode");
    }

    // regrid the data and return the final structure
    DEBUG_LOG("REG: starting regrid");
    return regridHeight(source_pc, transform, grid_spec, luminance_data);
}

#define IRTOOLS_IP_VERIFY_FINITE(v, e) do { if (!std::isfinite(v)) throw e("invalid value: "#v" is non-finite"); } while(false)
#define IRTOOLS_IP_VERIFY_NAN(v, e) do { if (std::isnan(v)) throw e("invalid value: "#v" is nan"); } while(false)

static void verifyRegistrationSettings(const RegistrationSettings& settings) {
    const auto& s = settings;

    const auto verifyNegZeroFiniteNan = [](const double val, const std::string& name) -> void {
        if (val < 0.0)
            throw RegistrationSettingsError("invalid value: invalid negative " + name);
        if (Eigen::internal::isApprox(val, 0.0))
            throw RegistrationSettingsError("invalid value: invalid zero " + name);
        if (!std::isfinite(val))
            throw RegistrationSettingsError("invalid value: " + name + " is non-finite");
        if (std::isnan(val))
            throw RegistrationSettingsError("invalid value: " + name + " is nan");
    };

    // verify device settings
    switch (s.device_type) {
        case DT_LJS8_LASER: {
            // verify device_settings type
            if (!std::holds_alternative<LJS8Settings>(s.device_settings))
                throw RegistrationSettingsError("LJS8 SETTINGS ERROR: invalid settings type for DT_LJS8_LASER");

            // get reference to the device_settings
            const auto& ds = std::get<LJS8Settings>(s.device_settings);

            // verify pixel_spacing
            verifyNegZeroFiniteNan(ds.x_raw_pixel_spacing, "x_raw_pixel_spacing");
            verifyNegZeroFiniteNan(ds.y_raw_pixel_spacing, "y_raw_pixel_spacing");

            // verify skipping
            if (ds.lx_skip == 0 || ds.ly_skip == 0 || ds.ux_skip == 0 || ds.uy_skip == 0)
                throw RegistrationSettingsError("LJS8 SETTINGS ERROR: invalid skipping");

            // verify height_zero
            IRTOOLS_IP_VERIFY_FINITE(ds.height_zero, RegistrationSettingsError);
            IRTOOLS_IP_VERIFY_NAN(ds.height_zero, RegistrationSettingsError);

            // verify height_scale
            if (Eigen::internal::isApprox(ds.height_scale, 0.0))
                throw RegistrationSettingsError("LJS8 SETTINGS ERROR: invalid height scale");
            IRTOOLS_IP_VERIFY_FINITE(ds.height_scale, RegistrationSettingsError);
            IRTOOLS_IP_VERIFY_NAN(ds.height_scale, RegistrationSettingsError);

            break;
        }
        default:
            throw RegistrationSettingsError("invalid device type");
    }

    // verify levels
    if (s.levels.empty())
        throw RegistrationSettingsError("no levels provided");
    for (size_t i = 0; i < s.levels.size(); i++) {
        const auto&[voxel_size, correspondence_distance, max_iterations] = s.levels.at(i);

        // verify voxel_size
        verifyNegZeroFiniteNan(voxel_size, "voxel_size for level " + std::to_string(i));

        // verify correspondence_distance
        verifyNegZeroFiniteNan(correspondence_distance, "correspondence_distance for level " + std::to_string(i));

        // verify max_iterations
        if (max_iterations == 0)
            throw RegistrationSettingsError("invalid zero max iterations for level " + std::to_string(i));
    }

    // verify relative_fitness
    if (s.relative_fitness < 0.0)
        throw RegistrationSettingsError("invalid relative_fitness value (must be non-negative)");
    IRTOOLS_IP_VERIFY_FINITE(s.relative_fitness, RegistrationSettingsError);
    IRTOOLS_IP_VERIFY_NAN(s.relative_fitness, RegistrationSettingsError);

    // verify relative_rmse
    if (s.relative_rmse < 0.0)
        throw RegistrationSettingsError("invalid relative_fitness value (must be non-negative)");
    IRTOOLS_IP_VERIFY_FINITE(s.relative_rmse, RegistrationSettingsError);
    IRTOOLS_IP_VERIFY_NAN(s.relative_rmse, RegistrationSettingsError);

    // verify normal_radius_factor
    verifyNegZeroFiniteNan(s.normal_radius_factor, "normal_radius_factor");

    // verify robust_scale if it exists
    if (s.robust_scale != std::nullopt)
        verifyNegZeroFiniteNan(s.robust_scale.value(), "robust_scale");

    // verify global_voxel_size
    verifyNegZeroFiniteNan(s.global_voxel_size, "global_voxel_size");

    // verify ransac_confidence
    if (s.ransac_confidence <= 0.0 || s.ransac_confidence > 1.0)
        throw RegistrationSettingsError("ransac confidence must be greater than zero and at most 1");
    IRTOOLS_IP_VERIFY_FINITE(s.ransac_confidence, RegistrationSettingsError);
    IRTOOLS_IP_VERIFY_NAN(s.ransac_confidence, RegistrationSettingsError);

    // verify min_fitness
    if (s.min_fitness <= 0.0 || s.min_fitness >= 1.0)
        throw RegistrationSettingsError("invalid min_fitness value (must be strictly between 0 and 1)");
    IRTOOLS_IP_VERIFY_FINITE(s.min_fitness, RegistrationSettingsError);
    IRTOOLS_IP_VERIFY_NAN(s.min_fitness, RegistrationSettingsError);

    // verify max_plane_rmse
    verifyNegZeroFiniteNan(s.max_plane_rmse, "max_plane_rmse");

    // verify max_center_shift
    verifyNegZeroFiniteNan(s.max_center_shift, "max_center_shift");

    // verify max_rotation_deg
    verifyNegZeroFiniteNan(s.max_rotation_deg, "max_rotation_deg");

    // verify min_constraint_ratio
    if (s.min_constraint_ratio <= 0.0 || s.min_constraint_ratio >= 1.0)
        throw RegistrationSettingsError("invalid min_constraint_ratio value (must be strictly between 0 and 1)");
    IRTOOLS_IP_VERIFY_FINITE(s.min_constraint_ratio, RegistrationSettingsError);
    IRTOOLS_IP_VERIFY_NAN(s.min_constraint_ratio, RegistrationSettingsError);

    // verify global mode
    if (s.global_mode < 0 || s.global_mode >= RGM_COUNT)
        throw RegistrationSettingsError("invalid global mode");

    // verify interpolation mode
    if (s.interpolation_mode < 0 || s.interpolation_mode >= RIM_COUNT)
        throw RegistrationSettingsError("invalid interpolation mode");
}

static RegistrationLjs8PointCloud ljs8DataToPointCloud(const std::string& fp, const LJS8Settings& settings)
{
    RegistrationLjs8PointCloud result{};

    // get the raw tiff data
    std::unique_ptr<uint16_t[]> data;
    uint32_t width, height;
    try {
        IRTools::tiff::loadTiff16Bit(fp, data, width, height);
    }
    catch (const std::exception& e) {
        throw RegistrationLJS8DataError("error reading tiff data: " + std::string(e.what()));
    }

    // populate the height and width fields
    result.width = width;
    result.height = height;
    if (width == 0 || height == 0)
        throw RegistrationLJS8DataError("invalid tiff image dimensions");

    // allocate space for each point in the cloud
    PointCloud& pc = result.cloud;
    try {
        pc.points_.resize(width * height);
    }
    catch (const std::exception& e) {
        throw RegistrationLJS8DataError("error allocating point cloud data: " + std::string(e.what()));
    }

    // for each datapoint, convert to a mm height value and add it to the point cloud
    for (uint32_t c = 0; c < height; c++) {
        for (uint32_t r = 0; r < width; r++) {
            // get index
            const size_t i =  c * width + r;

            // get the floating-point value of the height in mm
            const uint16_t raw_data = data[i];

            if (raw_data != 0) {
                const double X_PIXEL_SPACING_MM = settings.x_raw_pixel_spacing; // pixel spacing in um
                const double Y_PIXEL_SPACING_MM = settings.y_raw_pixel_spacing; // pixel spacing in um

                const double x_spacing = X_PIXEL_SPACING_MM * settings.lx_skip * settings.ux_skip; // unit length of image pixels along x-axis
                const double y_spacing = Y_PIXEL_SPACING_MM * settings.ly_skip * settings.uy_skip; // unit length of image pixels along y-axis

                const double x = r * x_spacing; // get um value for x coordinate, accounting for laser and user skipping
                const double y = c * y_spacing; // get um value for y coordinate, accounting for laser and user skipping
                const double z = (static_cast<double>(raw_data) - settings.height_zero) * settings.height_scale; // get um height from raw laser values

                // assemble then into a Vector3d
                const Eigen::Vector3d p = { x, y, z };

                // insert it into the list of points
                pc.points_.at(i) = p;
            }
        }
    }

    result.cloud = pc;

    // return the point cloud
    return result;
}

static Mat4d refineTransformICP(const Mat4d& t, const std::vector<PointCloudSP>& src_levels, const std::vector<PointCloudSP>& ref_levels, const RegistrationSettings& settings) {
    // ensure that the two vectors are of equal length
    if (src_levels.size() != ref_levels.size()) {
        throw RegistrationICPError("invalid levels vectors");
    }

    // estimator to be utilized
    const auto estimator = settings.robust_scale != std::nullopt ?
        reg::TransformationEstimationPointToPlane(std::make_shared<reg::HuberLoss>(reg::HuberLoss(settings.robust_scale.value())))
        : reg::TransformationEstimationPointToPlane();
    DEBUG_LOG("ICP: estimator instantiated");

    // transform to be modified and refined
    Mat4d transform = t;

    // iterate over each level, refining the transformation with each iteration
    for (size_t i = 0; i < src_levels.size(); i++) {
        // correspondence distance
        const auto cd = settings.levels.at(i).correspondence_distance;

        // convergence criteria to be utilized
        const auto convergence_criteria = reg::ICPConvergenceCriteria(
            settings.relative_fitness,
            settings.relative_rmse,
            settings.levels.at(i).max_iterations
        );

        // source and target point cloud at this level
        const auto& source = *src_levels.at(i);
        const auto& reference = *ref_levels.at(i);

        // perform ICP to get transform
        DEBUG_LOG("ICP: running RegistrationICP for level " << i);
        const reg::RegistrationResult res = reg::RegistrationICP(
            source,
            reference,
            cd,
            transform,
            estimator,
            convergence_criteria
        );

        // update with the more refined transform
        transform = res.transformation_;
    }

    // check the ICP transform, will throw if the transform is ill-formed
    DEBUG_LOG("ICP: checking transform");
    checkIcpTransform(transform, src_levels.back(), ref_levels.back(), settings);

    // return the refined transform
    return transform;
}

static Mat4d globalTransformRANSAC(const PointCloudSP& source, const PointCloudSP& target, const RegistrationSettings& settings) {
    // type aliases to reduce verboseness
    typedef std::shared_ptr<reg::Feature> FeatureSP;

    // global voxel size for downsampling
    const float global_voxel_size = settings.global_voxel_size;

    // generate downsampled point clouds according to the global voxel size
    const PointCloudSP src_cloud = source->VoxelDownSample(global_voxel_size);
    const PointCloudSP tgt_cloud = target->VoxelDownSample(global_voxel_size);

    // estimate the normals for each point cloud
    constexpr uint32_t MAX_NN_NORMALS = 30;
    constexpr double VOXEL_SCALE_NORMALS = 2;

    src_cloud->EstimateNormals(KDSearchParam(VOXEL_SCALE_NORMALS * global_voxel_size, MAX_NN_NORMALS));
    tgt_cloud->EstimateNormals(KDSearchParam(VOXEL_SCALE_NORMALS * global_voxel_size, MAX_NN_NORMALS));

    // orient the normals to point at the positive z-axis
    src_cloud->OrientNormalsToAlignWithDirection(Eigen::Vector3d::UnitZ());
    tgt_cloud->OrientNormalsToAlignWithDirection(Eigen::Vector3d::UnitZ());

    // compute features according to each cloud
    constexpr uint32_t MAX_NN_FEATURES = 100;
    constexpr double VOXEL_SCALE_FEATURES = 5;

    const FeatureSP src_feature = reg::ComputeFPFHFeature(*src_cloud, KDSearchParam(VOXEL_SCALE_FEATURES * global_voxel_size, MAX_NN_FEATURES));
    const FeatureSP tgt_feature = reg::ComputeFPFHFeature(*tgt_cloud, KDSearchParam(VOXEL_SCALE_FEATURES * global_voxel_size, MAX_NN_FEATURES));

    // seed open3d random
    open3d::utility::random::Seed(settings.random_seed);

    // perform RANSAC registration
    typedef reg::TransformationEstimationPointToPoint EstimationPTP;
    typedef std::vector<std::reference_wrapper<const reg::CorrespondenceChecker>> Checkers;
    typedef reg::CorrespondenceCheckerBasedOnEdgeLength CCEdgeLength;
    typedef reg::CorrespondenceCheckerBasedOnDistance CCDistance;
    typedef reg::RANSACConvergenceCriteria ConvergenceCriteria;

    constexpr double CORRESPONDENCE_SCALE_RANSAC = 1.5;
    constexpr bool MUTUAL_FILTER_RANSAC = true;
    constexpr int CORRESPONDENCE_SAMPLES_NUM = 3;
    constexpr double CC_EDGE_LENGTH_SIMILARITY_THRESHOLD = 0.9;
    constexpr double CC_DISTANCE_VOXEL_SCALE = 1.5;

    const CCEdgeLength edge_checker(CC_EDGE_LENGTH_SIMILARITY_THRESHOLD);
    const CCDistance distance_checker(CC_DISTANCE_VOXEL_SCALE * global_voxel_size);

    const reg::RegistrationResult res = reg::RegistrationRANSACBasedOnFeatureMatching(
        *src_cloud,
        *tgt_cloud,
        *src_feature,
        *tgt_feature,
        MUTUAL_FILTER_RANSAC,
        CORRESPONDENCE_SCALE_RANSAC * global_voxel_size,
        EstimationPTP(false),
        CORRESPONDENCE_SAMPLES_NUM,
        Checkers({
            std::cref(edge_checker),
            std::cref(distance_checker)
        }),
        ConvergenceCriteria(static_cast<int>(settings.ransac_iterations), settings.ransac_confidence)
    );

    // todo: add throw for res.correspondence_set < 3

    // return the resulting transformation matrix
    return res.transformation_;
}

static void checkIcpTransform(const Mat4d& transform, const PointCloudSP& source, const PointCloudSP& reference, const RegistrationSettings& settings) {
    const auto& points = source->points_;

    // todo: add checks for the transformation matrix itself

    // separate the transformation matrix into its rotation and translation components
    const Eigen::Matrix3d rotation = transform.topLeftCorner<3, 3>();
    const Eigen::Vector3d translation = transform.topRightCorner<3, 1>();

    // initialize the transformed coordinates vector
    std::vector<Eigen::Vector3d> moved;
    try {
        moved.reserve(points.size());
    }
    catch (const std::exception& e) {
        throw RegistrationCheckICPFailure("failed to construct transformed points list: " + std::string(e.what()));
    }

    // process at most 100,000 points from the point cloud
    constexpr size_t MAX_SAMPLE_COUNT = 100000;
    // amount of samples to be processed
    const size_t sample_count = (std::min)(points.size(), MAX_SAMPLE_COUNT);
    // constant step to get index within the points vector
    const double step = sample_count > 1
        ? (static_cast<double>(points.size()) - 1) / static_cast<double>(sample_count - 1)
        : 0;

    // translate and add each point to moved
    for (size_t i = 0; i < sample_count; i++) {
        // get points vector index
        const auto index = static_cast<size_t>(static_cast<double>(i) * step);

        // get reference to the corresponding point
        const Eigen::Vector3d& point = points.at(index);

        // perform the rotation and translation and add the resulting vector to the moved list/vector
        moved.emplace_back((rotation * point) + translation);
    }
    DEBUG_LOG("CHECK ICP: added each point to transformed coordinates vector");

    // get the minimum correspondence distance
    double min_correspondence_distance = settings.levels.at(0).correspondence_distance;
    for (size_t i = 1; i < settings.levels.size(); i++)
        if (const double cd = settings.levels.at(i).correspondence_distance; cd < min_correspondence_distance)
            min_correspondence_distance = cd;

    // initialize the indices and distances vectors
    struct ValidTransformedPoint {
        Eigen::Vector3d point;
        size_t tgt_index; // index wrt target point cloud of the closest point by Euclidean distance
        double distance; // actual Euclidean distance between the closest point in target and this point
    };

    // valid points within the provided correspondence distance
    std::vector<ValidTransformedPoint> valid_points;
    try {
        valid_points.reserve(moved.size());
    }
    catch (const std::runtime_error& e) {
        throw RegistrationCheckICPFailure("failed to construct valid points list: " + std::string(e.what()));
    }

    // squared sum of how far a transformed point lies along the reference surface's normal direction
    double residual2_sum = 0.0;

    // values used for metric calculations

    // center of the point cloud
    const Eigen::Vector3d center = (reference->GetMinBound() + reference->GetMaxBound()) / 2.0;
    // how much the transformation moves that center point
    const Eigen::Vector3d shift = rotation * center + translation - center;

    const Eigen::AngleAxisd angle_axis(rotation);
    // angle of the rotation, in degrees
    const double angle_degrees = angle_axis.angle() * 180.0 / std::numbers::pi;

    // the bounding box, representing the minimum and maximum xyz coordinates
    const Eigen::Vector3d dimensions = (reference->GetMaxBound() - reference->GetMinBound());
    // the diagonal length of the bounding box
    const double size = (std::max)(dimensions.norm(), std::numeric_limits<double>::epsilon());
    // represents how well the transformation constrains the six possible motions of a rigid transformation (i.e. translation on x, y, and z and rotation on x, y, and z)
    Eigen::Matrix<double, 6, 6> constraint_matrix = Eigen::Matrix<double, 6, 6>::Zero();


    // construct the KD-tree from the target point cloud
    geo::KDTreeFlann tree(*reference);

    // for each point in moved, find the single nearest neighbor, and add it to each list only if the distance is less than the min correspondence distance
    DEBUG_LOG("CHECK ICP: calculating nearest neighbors for each valid transformed coordinate");
    for (const Eigen::Vector3d& p : moved) {
        std::vector<int> nn_indices;
        std::vector<double> nn_distances_squared;

        const int found = tree.SearchKNN(
            p,
            1,
            nn_indices,
            nn_distances_squared
        );

        // if we find a point
        if (found > 0) {
            // the non-squared distance of the closest point
            const double nn_distance = std::sqrt(nn_distances_squared.at(0));

            // ensure that the point found is within our correspondence distance threshold
            if (nn_distance <= min_correspondence_distance) {
                // index of the closest reference point
                const size_t ref_index = nn_indices.at(0);
                // the normal of the reference point
                const Eigen::Vector3d& ref_normal = reference->normals_.at(ref_index);
                // the reference point
                const Eigen::Vector3d& ref_point = reference->points_.at(ref_index);

                ValidTransformedPoint vtp{};

                // populate the structure
                vtp.point = p;
                vtp.tgt_index = ref_index;
                vtp.distance = nn_distance; // add the Euclidean distance to the vector

                // calculate residual
                const double residual = (p - ref_point).dot(ref_normal);
                // add the square the of the residual to the sum
                residual2_sum += residual * residual;

                // accumulate constraint matrix
                const Eigen::Vector3d centered_normalized = (p - center) / size;
                const Eigen::Vector3d rotational = centered_normalized.cross(ref_normal);
                Eigen::Matrix<double, 6, 1> jacobian;
                jacobian.head<3>() = rotational;
                jacobian.tail<3>() = ref_normal;
                constraint_matrix.noalias() += jacobian * jacobian.transpose(); // accumulate J^T * J for each valid point

                // add it to the transformed points list
                valid_points.push_back(vtp);
            }
        }
    }

    // amount of points with a valid correspondence
    const size_t valid_point_count = valid_points.size();

    // flag to indicate that an error was generated
    bool should_throw = false;

    // string to indicate the errors generated
    std::string errors;

    // metrics to determine the success of ICP
    DEBUG_LOG("CHECK ICP: calcualting ICP checking metrics");
    constexpr uint32_t MIN_POINT_COUNT = 30;
    // proportion of samples that have a valid correspondence
    const double fitness = sample_count > 0
        ? static_cast<double>(valid_point_count) / static_cast<double>(sample_count)
        : 0;
    // normalized length of the shift
    const double center_shift_length = shift.norm();
    // point-to-plane root mean squared error
    double plane_rmse = sample_count > 0
        ? std::sqrt(residual2_sum / static_cast<double>(valid_point_count))
        : 0.0;
    // how well correspondences constrain the six degrees of freedom of the estimated rigid transformation
    double constrain_ratio;
    {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(
            constraint_matrix,
            Eigen::EigenvaluesOnly
        );

        // report failure if decomposition fails
        if (solver.info() != Eigen::Success) {
            throw RegistrationCheckICPFailure("eigenvalue decomposition failed");
        }

        const auto eigen_values = solver.eigenvalues();
        const double smallest = eigen_values[0];
        const double largest = eigen_values[5];
        constrain_ratio = (std::max)(0.0, smallest) / (std::max)(largest, 1e-30);
    }

    // check each metrix to ensure ICP effectiveness
    if (valid_point_count < MIN_POINT_COUNT) {
        const std::string MSG = "too few correspondences";
        should_throw = true;
        errors += errors.empty()
            ? MSG
            : ", " + MSG;
    }
    if (plane_rmse > settings.max_plane_rmse) {
        const std::string MSG = "residual exceeds limit";
        should_throw = true;
        errors += errors.empty()
            ? MSG
            : ", " + MSG;
    }
    if (constrain_ratio < settings.min_constraint_ratio) {
        const std::string MSG = "motion is weakly constrained";
        should_throw = true;
        errors += errors.empty()
            ? MSG
            : ", " + MSG;
    }
    if (fitness < settings.min_fitness) {
        const std::string MSG = "correspondence coverage below limit";
        should_throw = true;
        errors += errors.empty()
            ? MSG
            : ", " + MSG;
    }
    if (center_shift_length > settings.max_center_shift) {
        const std::string MSG = "translation exceeds limit";
        should_throw = true;
        errors += errors.empty()
            ? MSG
            : ", " + MSG;
    }
    if (angle_degrees > settings.max_rotation_deg) {
        const std::string MSG = "rotation exceeds limit";
        should_throw = true;
        errors += errors.empty()
            ? MSG
            : ", " + MSG;
    }

    // if we encounter any ICP errors, throw with the appropriate message
    if (should_throw)
        throw RegistrationICPError(errors);
}

// todo: make a copy of the data to find median, moves normals
static double pointCloudMedian(PointCloud& pc) {
    auto& points = pc.points_;

    // ensure the point cloud is non-empty
    if (points.empty()) {
        throw std::runtime_error("cannot find median of empty point cloud");
    }

    const size_t n = points.size();
    const size_t mid = n / 2;

    // comparison operator
    const auto& compare = [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) { return a.z() < b.z(); };

    std::ranges::nth_element(
        points.begin(),
        points.begin() + mid,
        points.end(),
        compare
    );

    const double upper = points[mid].z();

    // if n is odd, just return upper
    if (n % 2 != 0) {
        return upper;
    }

    // for even amounts, get the second middle number
    const double lower = std::ranges::max_element(
        points.begin(),
        points.begin() + mid,
        compare
    )->z();

    // calculate the average between the two values
    return (lower + upper) / 2.0;
}

template <typename T>
static T bilinearInterpolation(const T* const source, const bool* const valid_map, const Eigen::Vector2d& uv, const RegistrationGridSpec& grid_spec, const T fill) {
    const double& x = uv.x();
    const double& y = uv.y();

    // reject outside of image boundary
    if (x < 0 || x >= grid_spec.width || y < 0 || y >= grid_spec.height) {
        return fill;
    }

    const double alpha = x - std::floor(x);
    const double beta = y - std::floor(y);

    // calculate the bilinear weights
    const double w00 = (1.0 - alpha) * (1.0 - beta);
    const double w01 = alpha * (1.0 - beta);
    const double w10 = (1.0 - alpha) * beta;
    const double w11 = alpha * beta;

    const auto xlo = static_cast<size_t>(std::floor(x));
    const size_t xhi = (std::min)(xlo + 1, static_cast<size_t>(grid_spec.width - 1));
    const auto ylo = static_cast<size_t>(std::floor(y));
    const size_t yhi = (std::min)(ylo + 1, static_cast<size_t>(grid_spec.height - 1));

    // get indices
    const size_t i00 = ylo * grid_spec.width + xlo;
    const size_t i01 = ylo * grid_spec.width + xhi;
    const size_t i10 = yhi * grid_spec.width + xlo;
    const size_t i11 = yhi * grid_spec.width + xhi;

    // get valid values
    const bool v00 = valid_map[i00];
    const bool v01 = valid_map[i01];
    const bool v10 = valid_map[i10];
    const bool v11 = valid_map[i11];

    // get z values
    const T z00 = v00 ? source[i00] : fill;
    const T z01 = v01 ? source[i01] : fill;
    const T z10 = v10 ? source[i10] : fill;
    const T z11 = v11 ? source[i11] : fill;

    // calculate interpolated z value
    const T z_interpolated = static_cast<T>(w00 * z00 + w01 * z01 + w10 * z10 + w11 * z11);

    // return interpolated z value
    return z_interpolated;
}

static RegistrationResult regridHeight(PointCloud& source, const Mat4d& transform, const RegistrationGridSpec& grid_spec, const RegistrationLuminanceData& luminance_data) {
    // result structure packaging dimensions, buffers, and flags
    RegistrationResult result;

    // image dimensions and overall buffer size
    const uint32_t width = grid_spec.width;
    const uint32_t height = grid_spec.height;
    const uint32_t size = width * height;

    // populate result structure
    result.height = height;
    result.width = width;
    result.heights = std::make_unique<double[]>(size);
    result.valid_map = std::make_unique<bool[]>(size);
    result.luminance = luminance_data.valid
        ? std::make_unique<uint8_t[]>(size)
        : std::unique_ptr<uint8_t[]>();
    result.luminance_valid = luminance_data.valid;

    // ensure that if the luminance exists, it has the same dimensions as the grid specification
    if (luminance_data.valid && (luminance_data.width != grid_spec.width || luminance_data.height != grid_spec.height)) {
        throw std::runtime_error("luminance grid and height grid have different dimensions");
    }

    // create source grid and bitmap
    const auto source_grid = std::make_unique<double[]>(size);
    const auto source_bitmap = std::make_unique<bool[]>(size);

    // reconstruct original grid and bitmap
    for (const Eigen::Vector3d& p : source.points_) {
        // get the x, y, z, and index into the grid
        const size_t x = std::llround(p.x() / grid_spec.x_pixel_spacing);
        const size_t y = std::llround(p.y() / grid_spec.y_pixel_spacing);
        const double& z = p.z();
        const size_t i = y * width + x;

        // ensure that the values are within the grid
        if (x < 0 || y < 0 ||
            x >= static_cast<size_t>(width) ||
            y >= static_cast<size_t>(height))
        {
            const std::string msg = x >= width
                ? (y >= height
                    ? "x and y coordinates out of grid"
                    : "y coordinate out of grid")
                : "x coordinate out of grid";
            throw std::runtime_error(msg.c_str()); // todo: create custom exception
        }

        // ensure that the point is unique
        if (source_bitmap[i] == true) {
            throw std::runtime_error("duplicate coordinate in point cloud"); // todo: create custom exception
        }

        // save the z value into that position in the grid
        source_grid[i] = z;
        // set the bitmap value
        source_bitmap[i] = true;
    }
    DEBUG_LOG("REGRID: reconstructed source grid and bitmap from original height data");

    // find the median in linear time
    const double median_z = pointCloudMedian(source);
    DEBUG_LOG("REGRID: calculated source point cloud median");

    // extract rotation and translation from transformation matrix
    const Eigen::Matrix3d rotation = transform.topLeftCorner<3, 3>();
    const Eigen::Vector2d translation_xy = transform.topRightCorner<2, 1>(); // keep only x and y portions of translation

    // compute the matrix A, b, and sensitivity
    const Eigen::Matrix2d A = rotation.topLeftCorner<2, 2>();
    // ensure that the matrix is invertible
    if (std::abs(A.determinant()) < 1e-8) {
        throw std::runtime_error("regrid failure: rotation matrix is singular or nearly singular"); // todo: create custom exception
    }
    const Eigen::Vector2d b = rotation.topRightCorner<2, 1>();
    const Eigen::Matrix2d A_inv = A.inverse();
    const Eigen::Vector2d sensitivity = A_inv * b;

    // for each destination row and column, calculate the physical XY coordinates
    DEBUG_LOG("REGRID: starting regridding of grid size r: " << height << " x c: " << width);
    for (size_t r = 0; r < height; r++) {
        for (size_t c = 0; c < width; c++) {
            constexpr size_t MAX_ESTIMATION_ITERATIONS = 30;

            const double dx = grid_spec.x_pixel_spacing;
            const double dy = grid_spec.y_pixel_spacing;

            // calculate the physical x and y coordinates from grid indices
            const Eigen::Vector2d xy_physical = {
                static_cast<double>(c) * dx,
                static_cast<double>(r) * dy
            };

            // iteratively estimate the corresponding z value
            double z_estimate = median_z;
            bool converged = false;
            for (size_t i = 0; i < MAX_ESTIMATION_ITERATIONS; i++) {
                constexpr double CONVERGENCE_EPSILON = 1e-7;

                const Eigen::Vector2d uv_physical = A_inv * (xy_physical - translation_xy - b * z_estimate);
                const Eigen::Vector2d uv = {
                    uv_physical.x() / dx,
                    uv_physical.y() / dy
                };

                const auto z_next = bilinearInterpolation<double>(source_grid.get(), source_bitmap.get(), uv, grid_spec, median_z);

                const double z_delta = z_next - z_estimate;
                const double movement = (std::max)(
                    std::abs(sensitivity.x() * z_delta) / dx,
                    std::abs(sensitivity.y() * z_delta) / dy
                );

                converged = movement < CONVERGENCE_EPSILON;

                // update the current z estimation
                z_estimate = z_next;

                // break the loop if we converged
                if (converged) {
                    break;
                }
            }

            const Eigen::Vector2d uv_physical = A_inv * (xy_physical - translation_xy - b * z_estimate);
            const Eigen::Vector2d uv = {
                uv_physical.x() / dx,
                uv_physical.y() / dy
            };

            // check to ensure that the coordinates are inside the grid
            const double& x = uv.x();
            const double& y = uv.y();

            const bool inside =
                grid_spec.width > 0 &&
                grid_spec.height > 0 &&
                std::isfinite(x) &&
                std::isfinite(y) &&
                x >= 0.0 &&
                x <= static_cast<double>(grid_spec.width - 1) &&
                y >= 0.0 &&
                y <= static_cast<double>(grid_spec.height - 1);

            // proceed to checking only if the point is inside the grid
            if (inside) {
                // check to ensure that all four surrounding points are valid
                const auto xlo = static_cast<size_t>(std::floor(x));
                const size_t xhi = (std::min)(xlo + 1, static_cast<size_t>(grid_spec.width - 1));
                const auto ylo = static_cast<size_t>(std::floor(y));
                const size_t yhi = (std::min)(ylo + 1, static_cast<size_t>(grid_spec.height - 1));

                // indices into the source grid
                const size_t i00 = ylo * grid_spec.width + xlo;
                const size_t i01 = ylo * grid_spec.width + xhi;
                const size_t i10 = yhi * grid_spec.width + xlo;
                const size_t i11 = yhi * grid_spec.width + xhi;

                // ensure that every source pixel value is actually valid in the original grid
                const bool v00 = source_bitmap[i00];
                const bool v01 = source_bitmap[i01];
                const bool v10 = source_bitmap[i10];
                const bool v11 = source_bitmap[i11];

                const double alpha = x - static_cast<double>(xlo);
                const double beta = y - static_cast<double>(ylo);

                const double w00 = (1.0 - alpha) * (1.0 - beta);
                const double w01 = alpha * (1.0 - beta);
                const double w10 = (1.0 - alpha) * beta;
                const double w11 = alpha * beta;

                const double support =
                    w00 * static_cast<double>(v00) +
                    w01 * static_cast<double>(v01) +
                    w10 * static_cast<double>(v10) +
                    w11 * static_cast<double>(v11);

                const bool supported = support >= 1.0 - 1e-12;

                // final point is valid only if it is inside, supported, and has converged
                if (converged && supported) {
                    const auto z_sampled = bilinearInterpolation<double>(source_grid.get(), source_bitmap.get(), uv, grid_spec, median_z);

                    const double z_out =
                        rotation(2, 0) * uv_physical.x() +
                        rotation(2, 1) * uv_physical.y() +
                        rotation(2, 2) * z_sampled +
                        transform(2, 3);

                    const size_t index = r * grid_spec.width + c;

                    // save the z value to the grid
                    result.heights[index] = z_out;

                    // set the bit in the bitmap
                    result.valid_map[index] = true;
                }
            }

            // populate luminance buffer if it is valid
            if (result.luminance_valid) {
                const auto intensity = bilinearInterpolation<uint8_t>(luminance_data.data, luminance_data.valid_map, uv, grid_spec, 0);
                result.luminance[r * width + c] = intensity;
            }
        }
    }
    DEBUG_LOG("REGRID: regridding finished");

    return result;
}