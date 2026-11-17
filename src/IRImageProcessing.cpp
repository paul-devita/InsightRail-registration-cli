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
        const uint8_t* valid_map;
        uint32_t width;
        uint32_t height;
        bool valid;
    };
    struct RegistrationGridSpec {
        uint32_t height;
        uint32_t width;
        uint32_t x_pixel_spacing;
        uint32_t y_pixel_spacing;
    };
}

static RegistrationLjs8PointCloud ljs8DataToPointCloud(const std::string& fp, const LJS8Settings& settings);
static Mat4d refineTransformICP(const Mat4d& t, const std::vector<PointCloudSP>& src_levels, const std::vector<PointCloudSP>& ref_levels, const RegistrationSettings& settings);
static Mat4d globalTransformRANSAC(const PointCloudSP& source, const PointCloudSP& target, const RegistrationSettings& settings);
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
    const DeviceType dt = settings.type;
    PointCloud source_pc, reference_pc;
    RegistrationGridSpec grid_spec{};

    // luminance data if it is available
    RegistrationLuminanceData luminance_data{};

    // assemble point clouds based on the type of the data
    switch (dt) {
        case LJS8_LASER: {
            // get the laser-specific settings from RegistrationSettings
            const LJS8Settings& ljs8_settings = std::get<LJS8Settings>(settings.device_settings);
            const auto& ljs8_input = std::get<LJS8Input>(input);

            // save each point cloud
            const RegistrationLjs8PointCloud src_ljs8_cloud = ljs8DataToPointCloud(ljs8_input.source, ljs8_settings);
            const RegistrationLjs8PointCloud ref_ljs8_cloud = ljs8DataToPointCloud(ljs8_input.reference, ljs8_settings);

            // ensure that they have the same dimensions
            if (src_ljs8_cloud.width != ref_ljs8_cloud.width || src_ljs8_cloud.height != ref_ljs8_cloud.height) {
                throw std::runtime_error("source and reference grids have different dimensions");
            }

            // populate the grid specification
            grid_spec.width = ref_ljs8_cloud.width;
            grid_spec.height = ref_ljs8_cloud.height;
            grid_spec.x_pixel_spacing = ljs8_settings.lx_skip * ljs8_settings.ux_skip * ljs8_settings.x_raw_pixel_spacing;
            grid_spec.y_pixel_spacing = ljs8_settings.ly_skip * ljs8_settings.uy_skip * ljs8_settings.y_raw_pixel_spacing;

            // update the settings pixel spacing to match
            settings.x_pixel_size = grid_spec.x_pixel_spacing;
            settings.y_pixel_size = grid_spec.y_pixel_spacing;

            source_pc = src_ljs8_cloud.cloud;
            reference_pc = ref_ljs8_cloud.cloud;

            break;
        }
        default:
            throw std::runtime_error("invalid device type");
    }

    // todo: ask brennan about the scale for these
    constexpr uint32_t MAX_NN_NORMALS = 30;
    constexpr double VOXEL_SCALE_NORMALS = 2;

    // must be vector of shared_ptr, prevents unnecessary copying of point cloud data
    std::vector<PointCloudSP> src_levels, ref_levels;

    // if there are no voxel sizes provided, use full point clouds for registration
    if (settings.levels.empty()) {
        DEBUG_LOG("warning: no levels provided, performing normal calculation on entire dataset");

        // estimate normals for each point cloud, VERY EXPENSIVE
        source_pc.EstimateNormals(KDSearchParam(VOXEL_SCALE_NORMALS * settings.global_voxel_size, MAX_NN_NORMALS), true);
        reference_pc.EstimateNormals(KDSearchParam(VOXEL_SCALE_NORMALS * settings.global_voxel_size, MAX_NN_NORMALS), true);

        // orient the normals to point at the positive z-axis
        source_pc.OrientNormalsToAlignWithDirection(Eigen::Vector3d::UnitZ());
        reference_pc.OrientNormalsToAlignWithDirection(Eigen::Vector3d::UnitZ());

        // must provide a noop destructor so shared_ptr does not deallocate stack-allocated memory
        constexpr auto noop_destructor = [](geo::PointCloud*){};

        // create shared pointers for each full point cloud
        src_levels.emplace_back(&source_pc, noop_destructor);
        ref_levels.emplace_back(&reference_pc, noop_destructor);
    }
    // if there are voxel sizes provided, use them for registration
    else {
        const size_t voxel_count = settings.levels.size();

        // reserve space for the point clouds
        src_levels.resize(voxel_count);
        ref_levels.resize(voxel_count);

        const auto processLevel = [](std::vector<PointCloudSP>& dest, const size_t i, const float voxel, const float radius_factor, const PointCloud& original_cloud,
            const uint32_t iterations, bool is_ref) -> void {

            DEBUG_LOG("REG: estimating normals for " << (is_ref ? "reference" : "source") << " level " << i << " at voxel scale " << voxel);

            // calculate the downsampled point cloud
            const PointCloudSP downsampled = original_cloud.VoxelDownSample(voxel);

            // estimate the normals for the downsampled point cloud
            downsampled->EstimateNormals(KDSearchParam(voxel * radius_factor, static_cast<int>(iterations)), true);

            // set the designated index in the list to be the updated point cloud
            dest.at(i) = downsampled;
        };

        // for each voxel size, generate a point cloud and add it to the list, in order
        for (size_t i = 0; i < voxel_count; i++) {
            const float voxel = settings.levels.at(i).voxel_size;

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
            transform = refineTransformICP(Eigen::Matrix4d::Identity(), src_levels, ref_levels, settings);

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

static RegistrationLjs8PointCloud ljs8DataToPointCloud(const std::string& fp, const LJS8Settings& settings)
{
    RegistrationLjs8PointCloud result;

    // get the raw tiff data
    std::unique_ptr<uint16_t[]> data;
    uint32_t width, height;
    IRTools::tiff::loadTiff16Bit(fp, data, width, height);

    // populate the height and width fields
    result.width = width;
    result.height = height;

    // allocate space for each point in the cloud
    PointCloud& pc = result.cloud;
    pc.points_.reserve(width * height);

    // for each datapoint, convert to a mm height value and add it to the point cloud
    for (uint32_t c = 0; c < height; c++) {
        for (uint32_t r = 0; r < width; r++) {
            // get the floating-point value of the height in mm
            const uint16_t raw_data = data[c * width + r];

            if (raw_data != 0) {
                const double X_PIXEL_SPACING_MM = settings.x_raw_pixel_spacing; // pixel spacing in um
                const double Y_PIXEL_SPACING_MM = settings.y_raw_pixel_spacing; // pixel spacing in um

                const double x_spacing = X_PIXEL_SPACING_MM * settings.lx_skip * settings.ux_skip; // unit length of image pixels along x-axis
                const double y_spacing = Y_PIXEL_SPACING_MM * settings.ly_skip * settings.uy_skip; // unit length of image pixels along y-axis

                const double x = r * x_spacing; // get um value for x coordinate, accounting for laser and user skipping
                const double y = c * y_spacing; // get um value for y coordinate, accounting for laser and user skipping
                const double z = static_cast<double>(raw_data - settings.height_zero) * settings.height_scale; // get um height from raw laser values

                // assemble then into a Vector3d
                const Eigen::Vector3d p = { x, y, z };

                // insert it into the list of points
                pc.points_.push_back(p);
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
        throw std::runtime_error("invalid levels vectors");
    }

    // amount of levels to be processed
    const size_t len = src_levels.size();

    // estimator to be utilized
    const auto estimator = settings.loss_enabled ?
        reg::TransformationEstimationPointToPlane(std::make_shared<reg::HuberLoss>(reg::HuberLoss(settings.robust_scale)))
        : reg::TransformationEstimationPointToPlane();
    DEBUG_LOG("ICP: estimator instantiated");

    // transform to be modified and refined
    Mat4d transform = t;

    // iterate over each level, refining the transformation with each iteration
    for (size_t i = 0; i < len; i++) {
        // correspondence distance
        const auto cd = settings.levels.at(i).correspondence_distance;

        // convergence criteria to be utilized
        const auto convergence_criteria = reg::ICPConvergenceCriteria(
            settings.relative_fitness,
            settings.relative_rmse,
            settings.levels.at(i).max_iterations
        );

        // source and target point cloud at this level
        const auto& src = *src_levels.at(i);
        const auto& ref = *ref_levels.at(i);

        // perform ICP to get transform
        DEBUG_LOG("ICP: running RegistrationICP for level " << i);
        const reg::RegistrationResult res = reg::RegistrationICP(
            src,
            ref,
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
    //checkIcpTransform(transform, src_levels.front(), ref_levels.front(), settings);

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
        ConvergenceCriteria(settings.ransac_iterations, settings.ransac_confidence)
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
    moved.reserve(points.size());

    // process at most 100,000 points from the point cloud
    constexpr size_t MAX_SAMPLE_COUNT = 100000;
    const size_t sample_count = (std::min)(points.size(), MAX_SAMPLE_COUNT); // amount of samples to be processed
    const double step = sample_count > 1 // constant step to get index within the points vector
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
    uint16_t min_correspondence_distance = settings.levels.at(0).correspondence_distance;
    for (size_t i = 1; i < settings.levels.size(); i++)
        if (const uint16_t cd = settings.levels.at(i).correspondence_distance; cd < min_correspondence_distance)
            min_correspondence_distance = cd;

    // initialize the indices and distances vectors
    struct ValidTransformedPoint {
        Eigen::Vector3d point;
        size_t tgt_index; // index wrt target point cloud of the closest point by euclidean distance
        double distance; // actual euclidean distance between the closest point in target and this point
    };

    // valid points within the provided correspondence distance
    std::vector<ValidTransformedPoint> valid_points;
    valid_points.reserve(moved.size());

    // squared sum of how far a transformed point lies along the reference surface's normal direction
    double residual2_sum = 0.0;

    // values used for metric calculations
    const Eigen::Vector3d center = (reference->GetMinBound() + reference->GetMaxBound()) / 2.0;
    const Eigen::Vector3d shift = rotation * center + translation - center;
    const Eigen::AngleAxisd angle_axis(rotation);
    const double angle_degrees = angle_axis.angle() * 180.0 / std::numbers::pi;
    const Eigen::Vector3d dimensions = (reference->GetMaxBound() - reference->GetMinBound());
    const double size = (std::max)(dimensions.norm(), std::numeric_limits<double>::epsilon());
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

        if (found > 0) {
            const double nn_distance = std::sqrt(nn_distances_squared.at(0));

            if (nn_distance <= min_correspondence_distance) {
                const size_t ref_index = nn_indices.at(0); // index of the closest reference point
                const Eigen::Vector3d& ref_normal = reference->normals_.at(ref_index);
                const Eigen::Vector3d& ref_point = reference->points_.at(ref_index);

                ValidTransformedPoint vtp{};

                // populate the structure
                vtp.point = p;
                vtp.tgt_index = ref_index;
                vtp.distance = nn_distance; // add the Euclidean distance to the vector

                // calculate residual
                const double residual = (p - ref_point).dot(ref_normal);
                residual2_sum += residual * residual; // add the square the of the residual to the sum

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
    const size_t count = valid_points.size();

    // flag to indicate that an error was generated
    bool should_throw = false;

    // string to indicate the errors generated
    std::string errors;

    // metrics to determine the success of ICP
    DEBUG_LOG("CHECK ICP: calcualting ICP checking metrics");
    constexpr uint32_t MIN_POINT_COUNT = 30;
    const double fitness = sample_count > 0 // proportion of samples that have a valid correspondence
        ? static_cast<double>(count) / static_cast<double>(sample_count)
        : 0;
    const double center_shift_length = shift.norm(); // normalized length of the shift
    double plane_rmse = sample_count > 0 // point-to-plane root mean squared error
        ? std::sqrt(residual2_sum / static_cast<double>(count))
        : 0.0;
    double constrain_ratio; // how well correspondences constrain the six degrees of freedom of the estimated rigid transformation
    {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(
            constraint_matrix,
            Eigen::EigenvaluesOnly
        );

        if (solver.info() != Eigen::Success) {
            throw std::runtime_error("eigenvalue decomposition failed");
        }

        const auto eigen_values = solver.eigenvalues();
        const double smallest = eigen_values[0];
        const double largest = eigen_values[5];
        constrain_ratio = (std::max)(0.0, smallest) / (std::max)(largest, 1e-30);
    }

    // check each metrix to ensure ICP effectiveness
    if (count < MIN_POINT_COUNT) {
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
        throw std::runtime_error(errors.c_str());
}

// returns true if the ith bit is set, false otherwise
static bool checkBitmapIndex(const size_t i, const uint8_t* const bitmap) {
    const size_t byte = i / 8;
    const size_t bit = i % 8;

    const uint8_t val = bitmap[byte] & (1u << bit);

    return val > 0;
}

static void setBitmapIndex(const size_t i, const bool val, uint8_t* const bitmap) {
    const size_t byte = i / 8;
    const size_t bit = i % 8;

    bitmap[byte] = val
        ? bitmap[byte] | (1u << bit)
        : bitmap[byte] & ~(1u << bit);
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
    )._Unwrapped()->z();

    // calculate the average between the two values
    return (lower + upper) / 2.0;
}

template <typename T>
static T bilinearInterpolation(const T* const source, const uint8_t* const bitmap, const Eigen::Vector2d& uv, const RegistrationGridSpec& grid_spec, const T fill) {
    const double& x = uv.x();
    const double& y = uv.y();

    // reject outside of image boundary
    if (x < 0 || x >= grid_spec.width || y < 0 || y >= grid_spec.height) {
        return fill;
    }

    const double alpha = x - std::floor(x);
    const double beta = y - std::floor(y);

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
    const bool v00 = checkBitmapIndex(i00, bitmap);
    const bool v01 = checkBitmapIndex(i01, bitmap);
    const bool v10 = checkBitmapIndex(i10, bitmap);
    const bool v11 = checkBitmapIndex(i11, bitmap);

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
    const uint32_t xp_size = grid_spec.x_pixel_spacing;
    const uint32_t yp_size = grid_spec.y_pixel_spacing;

    // populate result structure
    result.height = height;
    result.width = width;
    result.heights = std::make_unique<double[]>(size);
    result.valid_bitmap = std::make_unique<uint8_t[]>((size - 1) / 8 + 1);
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
    const auto source_bitmap = std::make_unique<uint8_t[]>((size - 1) / 8 + 1);

    // reconstruct original grid and bitmap
    for (const Eigen::Vector3d& p : source.points_) {
        // get the x, y, z, and index into the grid
        const size_t x = static_cast<size_t>(p.x() / xp_size);
        const size_t y = static_cast<size_t>(p.y() / yp_size);
        const double& z = p.z();
        const size_t i = y * width + x;

        // ensure that the values are within the grid
        if (i >= size) {
            const std::string msg = x >= width
                ? (y >= height
                    ? "x and y coordinates out of grid"
                    : "y coordinate out of grid")
                : "x coordinate out of grid";
            throw std::runtime_error(msg.c_str()); // todo: create custom exception
        }

        // ensure that the point is unique
        if (checkBitmapIndex(i, source_bitmap.get())) {
            throw std::runtime_error("duplicate coordinate in point cloud"); // todo: create custom exception
        }

        // save the z value into that position in the grid
        source_grid[i] = z;
        // set the bitmap value
        setBitmapIndex(i, true, source_bitmap.get());
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

            const uint32_t dx = grid_spec.x_pixel_spacing;
            const uint32_t dy = grid_spec.y_pixel_spacing;

            // calculate the physical x and y coordinates from grid indices
            const Eigen::Vector2d xy_physical = {
                c * dx,
                r * dy
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

            // proceed to checking only if the point is inside of the grid
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
                const bool v00 = checkBitmapIndex(i00, source_bitmap.get());
                const bool v01 = checkBitmapIndex(i01, source_bitmap.get());
                const bool v10 = checkBitmapIndex(i10, source_bitmap.get());
                const bool v11 = checkBitmapIndex(i11, source_bitmap.get());

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
                    setBitmapIndex(index, true, result.valid_bitmap.get());
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