
#include <IRImageProcessing.h>
#include <IRImageProcessingDefaults.h>

#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <charconv>
#include <cctype>
#include <iomanip>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <variant>

#include "TiffOperations.h"

#define VERSION "v1.0"

/*
COMMANDS:
help - lists command
settings - settings related commands
    settings view -> prints current registration settings
    settings set <id> <val> -> sets the designated setting
    settings default -> sets settings to default vals
    settings import <json> -> sets the settings to the json
    settings export -> exports current settings to json
register - settings related to registration
    register folder <height_path> <optional: lum_path> -> registers all images in the designated folder, saves to "<path>_REGISTERED"
    register single <height_path> <optional: lum_path> -> registers a single image
*/
namespace {
    struct Token {
        std::string text;
        bool isList = false;
    };

    enum class CommandType {
        Help,
        Exit,
        Test,
        SettingsView,
        SettingsSet,
        SettingsDefault,
        SettingsImport,
        SettingsExport,
        RegisterFolder,
        RegisterSingle
    };

    struct SettingAssignment {
        std::string id;
        Token value;  // Preserves whether the value was a number list.
    };

    struct ImportArguments {
        std::string filePath;
    };

    struct RegistrationArguments {
        std::string heightPath;
        std::string referenceHeightPath;
        std::optional<std::string> lumPath;
    };

    struct TestArguments {
        std::string heightPath;
    };

    struct Command {
        CommandType type;
        std::variant<std::monostate, SettingAssignment,
            ImportArguments, RegistrationArguments, TestArguments> arguments;
    };

}
static std::vector<Token> parseTokens(std::string_view input);
static Command processCommand(std::string_view input);
static void runCommand(const Command& command);

static void printHelp(std::ostream& out = std::cout);
static void printSettings(const RegistrationSettings& settings_, std::ostream& out = std::cout);
static RegistrationResult registerSingle(std::string source, std::string reference, const std::optional<std::string>& luminance);
static void testSingle(const std::string& reference);

static IRTools::ImageProcessing::RegistrationSettings settings = LJS8DefaultSettings;

int main(void) {
    std::cout << "InsightRail Image Registration CLI " << VERSION << std::endl;
    std::cout << "Copyright (c) 2026 INSIGHT RAIL LLC. All rights reserved." << std::endl;

    bool running = true;

    while (running) {
        std::cout << "IRImageProcessing>>";
        std::string input;
        std::getline(std::cin, input);

        try {
            // parse command from tokens
            const Command c = processCommand(input);
            runCommand(c);
        }
        catch (const std::invalid_argument& e) {
            std::cout << "ERROR: " << e.what() << std::endl;
        }
    }
}

static std::vector<Token> parseTokens(std::string_view input) {
    auto isSpace = [](char ch) {
        return std::isspace(static_cast<unsigned char>(ch)) != 0;
    };

    auto trim = [&](std::string_view value) {
        while (!value.empty() && isSpace(value.front())) {
            value.remove_prefix(1);
        }
        while (!value.empty() && isSpace(value.back())) {
            value.remove_suffix(1);
        }
        return value;
    };

    std::vector<Token> tokens;
    std::size_t pos = 0;

    while (pos < input.size()) {
        while (pos < input.size() && isSpace(input[pos])) {
            ++pos;
        }
        if (pos == input.size()) {
            break;
        }

        const std::size_t start = pos;

        if (input[pos] != '[') {
            while (pos < input.size() && !isSpace(input[pos])) {
                ++pos;
            }
            tokens.push_back({std::string(input.substr(start, pos - start)), false});
            continue;
        }

        const std::size_t close = input.find(']', pos + 1);
        if (close == std::string_view::npos ||
            (close + 1 < input.size() && !isSpace(input[close + 1]))) {
            throw std::invalid_argument("Malformed number list");
        }

        const auto contents = input.substr(pos + 1, close - pos - 1);
        if (contents.empty() || contents.find('[') != std::string_view::npos) {
            throw std::invalid_argument("Malformed number list");
        }

        // Check each comma-separated entry. The actual numbers remain in
        // Token::text for the command handler to convert when needed.
        std::size_t entryStart = 0;
        while (true) {
            const std::size_t comma = contents.find(',', entryStart);
            auto entry = trim(contents.substr(
                entryStart,
                comma == std::string_view::npos
                    ? std::string_view::npos
                    : comma - entryStart));

            if (!entry.empty() && entry.front() == '+') {
                entry.remove_prefix(1);
            }
            if (entry.empty()) {
                throw std::invalid_argument("Empty number in list");
            }

            double number;
            const auto [end, error] = std::from_chars(
                entry.data(), entry.data() + entry.size(),
                number, std::chars_format::general);

            if (error != std::errc{} ||
                end != entry.data() + entry.size() ||
                !std::isfinite(number)) {
                throw std::invalid_argument("Invalid number in list");
            }

            if (comma == std::string_view::npos) {
                break;
            }
            entryStart = comma + 1;
        }

        pos = close + 1;
        tokens.push_back({std::string(input.substr(start, pos - start)), true});
    }

    return tokens;
}

static Command processCommand(std::string_view input) {
    const auto tokens = parseTokens(input);

    auto require = [](bool condition, const char* usage) {
        if (!condition) {
            throw std::invalid_argument(usage);
        }
    };

    if (tokens.empty()) {
        throw std::invalid_argument("Enter a command");
    }

    if (tokens[0].text == "help") {
        require(tokens.size() == 1, "Usage: help");
        return {CommandType::Help, std::monostate{}};
    }

    if (tokens[0].text == "exit") {
        require(tokens.size() == 1, "Usage: exit");
        return {CommandType::Exit, std::monostate{}};
    }

    if (tokens[0].text == "test") {
        require(tokens.size() == 2, "Usage: test <height_path>");
        return {CommandType::Test, TestArguments{tokens[1].text}};
    }

    if (tokens[0].text == "settings") {
        require(tokens.size() >= 2, "Usage: settings <subcommand>");

        const auto& action = tokens[1].text;

        if (action == "view") {
            require(tokens.size() == 2, "Usage: settings view");
            return {CommandType::SettingsView, std::monostate{}};
        }
        if (action == "default") {
            require(tokens.size() == 2, "Usage: settings default");
            return {CommandType::SettingsDefault, std::monostate{}};
        }
        if (action == "export") {
            require(tokens.size() == 2, "Usage: settings export");
            return {CommandType::SettingsExport, std::monostate{}};
        }
        if (action == "import") {
            require(tokens.size() == 3 && !tokens[2].isList,
                    "Usage: settings import <json_file_path>");
            return {CommandType::SettingsImport,
                    ImportArguments{tokens[2].text}};
        }
        if (action == "set") {
            require(tokens.size() == 4 && !tokens[2].isList,
                    "Usage: settings set <id> <val>");
            return {CommandType::SettingsSet,
                    SettingAssignment{tokens[2].text, tokens[3]}};
        }

        throw std::invalid_argument("Unknown settings subcommand");
    }

    if (tokens[0].text == "register") {
        constexpr auto usage =
            "Usage: register <folder|single> "
            "<height_path> <reference_height_path> [lum_path]";

        require(tokens.size() >= 2, usage);

        const auto& action = tokens[1].text;
        const bool folder = action == "folder";
        require(folder || action == "single", usage);

        require((tokens.size() == 4 || tokens.size() == 5) &&
                    !tokens[2].isList &&
                    !tokens[3].isList &&
                    (tokens.size() == 4 || !tokens[4].isList),
                usage);

        RegistrationArguments args{
            .heightPath = tokens[2].text,
            .referenceHeightPath = tokens[3].text,
            .lumPath = tokens.size() == 5
                ? std::optional<std::string>{tokens[4].text}
                : std::nullopt
    };

        return {
            .type = folder ? CommandType::RegisterFolder
                           : CommandType::RegisterSingle,
            .arguments = args
        };
    }

    throw std::invalid_argument("Unknown command");
}

static void runCommand(const Command& command) {
    switch (command.type) {
        case CommandType::Help:
            printHelp();
            break;
        case CommandType::Exit:
            std::exit(0);
            break;
        case CommandType::Test: {
            const auto& args = std::get<TestArguments>(command.arguments);
            DEBUG_LOG("CLI: running test on " << args.heightPath);
            testSingle(args.heightPath);
            break;
        }
        case CommandType::SettingsView:
            printSettings(settings);
            break;
        case CommandType::SettingsSet:
        case CommandType::SettingsDefault:
            settings = LJS8DefaultSettings;
            break;
        case CommandType::SettingsImport:
        case CommandType::SettingsExport:
        case CommandType::RegisterFolder:
        case CommandType::RegisterSingle: {
            const auto& args = std::get<RegistrationArguments>(command.arguments);
            DEBUG_LOG("CLI: calling register single");
            registerSingle(
                args.heightPath,
                args.referenceHeightPath,
                args.lumPath
            );
            DEBUG_LOG("CLI: register single returned");
            break;
        }
    }
}

void printHelp(std::ostream& out) {
    out << R"(Registration CLI

GENERAL
  help
      Show this command list.
  exit
      Quit the program.

SETTINGS
  settings view
      Display the current registration settings.

  settings set <id> <value>
      Set one setting. A value may be a single token or a number list
      such as [1, 2, 3].

  settings default
      Restore the default registration settings.

  settings import <json_file_path>
      Load registration settings from a JSON file.

  settings export
      Export the current registration settings to JSON.

REGISTRATION
  register folder <source> <reference> [lum_path]
      Register all images in the designated folder. Results are saved
      to "<source>_REGISTERED".

  register single <source> <reference> [lum_path]
      Register one image.

EXAMPLES
  settings set voxel_sizes [4, 2, 1]
  settings import config.json
  register folder ./images
  register single ./images/height.tif ./images/luminance.tif

Arguments in [square brackets] are optional. Number lists must use
square brackets and comma-separated numbers. Paths cannot contain spaces
with the current command parser.
)";
}

void printSettings(const RegistrationSettings& settings_, std::ostream& out) {
    std::ostringstream text;
    text << std::boolalpha << std::setprecision(6);

    auto row = [&](std::string_view label, const auto& value) {
        text << "  " << std::left << std::setw(32) << label
             << value << '\n';
    };

    text << "REGISTRATION SETTINGS\n\n";

    text << "Device\n";
    if (settings_.device_type == DeviceType::DT_LJS8_LASER) {
        row("Type", "LASER");

        if (const auto* laser = std::get_if<LJS8Settings>(&settings_.device_settings)) {
            row("Raw X pixel spacing (um)", laser->x_raw_pixel_spacing);
            row("Raw Y pixel spacing (um)", laser->y_raw_pixel_spacing);
            row("Laser X skip", static_cast<unsigned>(laser->lx_skip));
            row("Laser Y skip", static_cast<unsigned>(laser->ly_skip));
            row("User X skip", static_cast<unsigned>(laser->ux_skip));
            row("User Y skip", static_cast<unsigned>(laser->uy_skip));
            row("Height zero", laser->height_zero);
            row("Height scale", laser->height_scale);

            text << "  " << std::left << std::setw(32)
                 << "Invalid values" << '[';
            for (std::size_t i = 0; i < laser->invalid_values.size(); ++i) {
                if (i != 0) text << ", ";
                text << laser->invalid_values[i];
            }
            text << "]\n";
        } else {
            row("Device settings", "<type mismatch>");
        }
    }

    row("Effective X pixel size", settings_.x_pixel_size);
    row("Effective Y pixel size", settings_.y_pixel_size);

    text << "\nRegistration levels\n";
    if (settings_.levels.empty()) {
        text << "  None\n";
    } else {
        for (std::size_t i = 0; i < settings_.levels.size(); ++i) {
            const auto& level = settings_.levels[i];
            text << "  Level " << i
                 << ": voxel=" << level.voxel_size << " um"
                 << ", correspondence=" << level.correspondence_distance << " um"
                 << ", iterations=" << level.max_iterations << '\n';
        }
    }

    text << "\nRefinement\n";
    row("Relative fitness", settings_.relative_fitness);
    row("Relative RMSE", settings_.relative_rmse);
    row("Normal radius factor", settings_.normal_radius_factor);
    row("Robust scale", (settings_.robust_scale == std::nullopt ? 0.0 : settings_.robust_scale.value()));

    text << "\nGlobal registration\n";
    row("Global voxel size", settings_.global_voxel_size);
    row("RANSAC iterations", settings_.ransac_iterations);
    row("RANSAC confidence", settings_.ransac_confidence);
    row("Random seed", settings_.random_seed);
    row("Global mode (enum value)", static_cast<int>(settings_.global_mode));

    text << "\nAcceptance limits\n";
    row("Minimum fitness", settings_.min_fitness);
    row("Maximum plane RMSE", settings_.max_plane_rmse);
    row("Maximum center shift", settings_.max_center_shift);
    row("Maximum rotation (deg)", settings_.max_rotation_deg);
    row("Minimum constraint ratio", settings_.min_constraint_ratio);

    text << "\nOutput\n";
    row("Interpolation mode (enum value)",
        static_cast<int>(settings_.interpolation_mode));

    out << text.str();
}

static RegistrationResult registerSingle(std::string source, std::string reference, const std::optional<std::string>& luminance) {
    LJS8Input input = {
        .source = std::move(source),
        .reference = std::move(reference),
        .luminance = luminance == std::nullopt
            ? ""
            : luminance.value(),
        .luminance_valid = luminance != std::nullopt
    };

    return registerSingleCloud(input, settings);

    // todo: ask brennan how he wants the heights represented since at this point they are decimal values
}

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

void testSingle(const std::string &reference) {
    struct HeightData {
        uint32_t width;
        uint32_t height;
        std::unique_ptr<uint16_t[]> data;
        std::unique_ptr<double[]> scaled_data;
        std::vector<bool> valid_bitmap;
    } reference_data{};

    // open the source file
    IRTools::tiff::loadTiff16Bit(reference, reference_data.data, reference_data.width, reference_data.height);

    // allocate required bitmap elements
    reference_data.valid_bitmap.resize(reference_data.width * reference_data.height);

    // get height scale and height zero
    const double height_scale = std::get<IRTools::ImageProcessing::LJS8Settings>(settings.device_settings).height_scale;
    const double height_zero = std::get<IRTools::ImageProcessing::LJS8Settings>(settings.device_settings).height_zero;

    // populate source bitmap and scaled data
    for (size_t i = 0; i < reference_data.width * reference_data.height; i++)
        // if the data value is non-zero, set it as valid within the bitmap
        if (reference_data.data[i] != 0)
            reference_data.valid_bitmap.at(i) = true;

    struct ShiftAmount {
        size_t sx;
        size_t sy;
    };

    // test params
    constexpr size_t SHIFTS_NUM = 4;
    constexpr std::array<ShiftAmount, SHIFTS_NUM> shifts = {
        ShiftAmount{.sx = 0, .sy = 0},
        ShiftAmount{.sx = 0, .sy = 10},
        ShiftAmount{.sx = 10, .sy = 0},
        ShiftAmount{.sx = 10, .sy = 10}
    };

    // print test params
    std::cout << "TEST: testing input \'" << reference << "\' on shift amounts: [";
    for (size_t i = 0; i < SHIFTS_NUM; i++) {
        if (i > 0) std::cout << " ";
        std::cout << "(" << shifts[i].sx << "," << shifts[i].sy << ")";
        if (i < SHIFTS_NUM - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // metrics for test
    double sum_rmse_scaled = 0.0;
    double min_rmse_scaled = std::numeric_limits<double>::infinity();
    double max_rmse_scaled = -std::numeric_limits<double>::infinity();
    double sum_rmse_unscaled = 0.0;
    double min_rmse_unscaled = std::numeric_limits<double>::infinity();
    double max_rmse_unscaled = -std::numeric_limits<double>::infinity();
    uint32_t mismatched_points = 0;

    // buffer for shifted data
    bool is_clean = true;
    const size_t size = reference_data.width * reference_data.height;
    std::unique_ptr<uint16_t[]> shifted = std::make_unique<uint16_t[]>(size);

    // run each test
    for (const auto& shift : shifts) {
        // clear the shifted buffer if it is dirty
        if (is_clean)
            is_clean = false;
        else
            memset(shifted.get(), 0, size * sizeof(uint16_t));

        const size_t dx = shift.sx;
        const size_t dy = shift.sy;

        // populate the shifted buffer according to the shift
        for (size_t y = 0; y < reference_data.height; y++) {
            for (size_t x = 0; x < reference_data.width; x++) {
                const size_t xdest = x + dx;
                const size_t ydest = y + dy;

                // if the shifted data is in bounds, save it to the shifted buffer
                if (xdest < reference_data.width && ydest < reference_data.height) {
                    const size_t dest_index = ydest * reference_data.width + xdest;
                    const size_t src_index = y * reference_data.width + x;

                    shifted[dest_index] = reference_data.data[src_index];
                }
            }
        }

        // save the shifted heightmap to disk
        const std::string shifted_path = "shifted_temp.tiff";
        IRTools::tiff::saveTiff16Bit(shifted_path, shifted.get(), reference_data.height, reference_data.width);

        // run registration using the shift as the source
        const auto result = registerSingle(shifted_path, reference, std::nullopt);

        // ensure that the image dimensions match up
        if (bool width_mismatch = result.width != reference_data.width, height_mismatch = result.height != reference_data.height;
            width_mismatch || height_mismatch) {
            std::cout << "TEST ERROR: " <<
                (width_mismatch
                ? (height_mismatch
                    ? "result width and height do not match with source image width and height"
                    : "result height does not match with source image height")
                : "result width does not match with source image width")
            << std::endl;
            return;
        }

        // sums of rmse for scaled and unscaled values
        double se_scaled = 0.0;
        double se_unscaled = 0.0;
        uint32_t n = 0;
        uint32_t matched = 0;

        // buffer to save for testing
        std::unique_ptr<uint16_t[]> buff = std::make_unique<uint16_t[]>(size);

        // check each valid data point, taking note of any data points whose validness doesn't match
        for (size_t i = 0; i < size; i++) {
            const bool source_valid = reference_data.valid_bitmap.at(i);
            const bool shifted_valid = checkBitmapIndex(i, result.valid_bitmap.get());

            if (shifted_valid) {
                const double shifted_scaled = result.heights[i];
                const auto shifted_unscaled = static_cast<uint16_t>(std::round(shifted_scaled / height_scale + height_zero));

                buff[i] = shifted_unscaled;
            }

            if (source_valid == shifted_valid)
                matched++;
            else
                mismatched_points++;

            if (source_valid && shifted_valid) {
                n++;

                const uint16_t source_unscaled = reference_data.data[i];
                const double source_scaled = (static_cast<double>(source_unscaled) - static_cast<double>(height_zero)) * height_scale;

                const double shifted_scaled = result.heights[i];
                const auto shifted_unscaled = static_cast<uint16_t>(std::round(shifted_scaled / height_scale + height_zero));

                const double diff_unscaled = shifted_unscaled - source_unscaled;
                const double diff_scaled = shifted_scaled - source_scaled;

                se_unscaled += diff_unscaled * diff_unscaled;
                se_scaled += diff_scaled * diff_scaled;
            }
        }

        double rmse_scaled = std::sqrt(se_scaled / static_cast<double>(n));
        double rmse_unscaled = std::sqrt(se_unscaled / static_cast<double>(n));

        // update min rmse
        if (rmse_scaled < min_rmse_scaled) min_rmse_scaled = rmse_scaled;
        if (rmse_unscaled < min_rmse_unscaled) min_rmse_unscaled = rmse_unscaled;

        // update max rmse
        if (rmse_scaled > max_rmse_scaled) max_rmse_scaled = rmse_scaled;
        if (rmse_unscaled > max_rmse_unscaled) max_rmse_unscaled = rmse_unscaled;

        // update rmse sum
        sum_rmse_scaled += rmse_scaled;
        sum_rmse_unscaled += rmse_unscaled;

        IRTools::tiff::saveTiff16Bit("height_out_temp.tiff", buff.get(), reference_data.height, reference_data.width);

        DEBUG_LOG("TEST: done with test for shift (" << shift.sx << "," << shift.sy << "), got " << n << " matched points and scaled RMSE " << rmse_scaled << " and unscaled RMSE " << rmse_unscaled);
    }

    // print results
    std::cout << std::fixed << std::setprecision(6);

    std::cout << "\n========== REGISTRATION RMSE STATISTICS ==========\n";

    std::cout << std::left << std::setw(15) << "Metric"
              << std::right << std::setw(18) << "Scaled"
              << std::setw(18) << "Unscaled" << '\n';

    std::cout << std::string(51, '-') << '\n';

        std::cout << std::left << std::setw(15) << "Average RMSE"
                  << std::right << std::setw(18) << sum_rmse_scaled / SHIFTS_NUM
                  << std::setw(18) << sum_rmse_unscaled / SHIFTS_NUM << '\n';

        std::cout << std::left << std::setw(15) << "Minimum RMSE"
                  << std::right << std::setw(18) << min_rmse_scaled
                  << std::setw(18) << min_rmse_unscaled << '\n';

        std::cout << std::left << std::setw(15) << "Maximum RMSE"
                  << std::right << std::setw(18) << max_rmse_scaled
                  << std::setw(18) << max_rmse_unscaled << '\n';

    std::cout << "==================================================\n";
}
