//
// RealSense Camera Device Handler
//

#include "realsense_camera.hpp"


realsense_camera::realsense_camera()
{
    active_dev_idx_ = 0;
    is_color_streaming_ = false;
    is_depth_streaming_ = false;
    is_color_enabled_ = false;
    is_depth_enabled_ = false;
    is_init_ = false;
    reconfigure_ = false;
    change_props_ = false;
    stream_alignment_ = STREAM_ALIGN_OFF;

    rs_depth_to_disparity_ = std::make_unique<rs2::disparity_transform>(true);
    rs_disparity_to_depth_ = std::make_unique<rs2::disparity_transform>(false);

    filters_.emplace_back("Decimate", rs_dec_filter_);
    filters_.emplace_back("Threshold", rs_thr_filter_);
    filters_.emplace_back("Disparity", *rs_depth_to_disparity_);
    filters_.emplace_back("Spatial", rs_spat_filter_);
    filters_.emplace_back("Temporal", rs_temp_filter_);
    filters_.emplace_back("Hole Fill", rs_hole_fill_filter_);

    align_to_depth = std::make_unique<rs2::align>(RS2_STREAM_DEPTH);
    align_to_color = std::make_unique<rs2::align>(RS2_STREAM_COLOR);

    RefreshDeviceList();
}

void realsense_camera::RefreshDeviceList()
{
    camera_name_list_.clear();
    camera_dev_list_.clear();

    camera_name_list_.emplace_back("None");

    rs2::context ctx;
    auto devices_list = ctx.query_devices();
    size_t device_count = devices_list.size();
    if (!device_count) {
        return;
    }

    for ( auto i = 0u; i < device_count; i++) {
        try {
            auto dev = devices_list[i];
            std::string cam_name;
            cam_name = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            cam_name += " - ";
            cam_name += dev.get_info(RS2_CAMERA_INFO_NAME);
            camera_dev_list_.emplace_back(dev);
            camera_name_list_.emplace_back(cam_name);
        }
        catch (const std::exception & e) {
            std::cout << "Could not create device - " << e.what() <<" . Check SDK logs for details" << std::endl;
        }
        catch(...) {
            std::cout << "Failed to created device. Check SDK logs for details" << std::endl;
        }
    }
}

int realsense_camera::GetDeviceCount()
{
    return (int)camera_dev_list_.size();
}

const std::vector<std::string> &realsense_camera::GetDeviceList()
{
    return camera_name_list_;
}

void realsense_camera::InitCamera_()
{
    std::lock_guard<std::mutex> lck(io_mutex_);
    if (is_init_) {
        if (is_color_streaming_ || is_depth_streaming_) {
            pipe_.stop();
            cfg_.disable_all_streams();
        }
        is_color_streaming_ = false;
        is_depth_streaming_ = false;
    }

    is_init_ = false;
    if (init_idx_ > 0 && init_idx_ < camera_dev_list_.size() + 1) {
        active_dev_idx_ = init_idx_ - 1;
        color_configs_.clear();
        color_props_.clear();
        depth_props_.clear();
        rs_device_ = camera_dev_list_.at(active_dev_idx_);
        rs_dev_serial_ = rs_device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        cfg_.enable_device(rs_dev_serial_);
        pipe_profile_ = cfg_.resolve(pipe_);
        for (auto&& sensor : rs_device_.query_sensors())
        {
            for (auto&& profile : sensor.get_stream_profiles())
            {
                // Set Sensor Handles
                if (profile.stream_type() == RS2_STREAM_DEPTH)
                    rs_depth_sensor_ = sensor;
                else if (profile.stream_type() == RS2_STREAM_COLOR)
                    rs_color_sensor_ = sensor;

                // Get Supported Resolutions and Frame Rates
                if (auto video = profile.as<rs2::video_stream_profile>())
                {
                    if (profile.stream_name() == "Depth") {
                        int cfg_idx = 0;
                        bool exists = false;
                        std::string profile_res;
                        profile_res += std::to_string(video.width());
                        profile_res += " x ";
                        profile_res += std::to_string(video.height());
                        for (int i = 0; i < depth_configs_.size(); i++) {
                            if (depth_configs_.at(i).str_resolution == profile_res) {
                                cfg_idx = i;
                                exists = true;
                                break;
                            }
                        }
                        if (exists) {
                            depth_configs_.at(cfg_idx).fps_list.emplace_back(profile.fps());
                        }
                        else {
                            StreamConfig str_cfg;
                            str_cfg.str_stream_name = profile.stream_name();
                            str_cfg.str_resolution = profile_res;
                            str_cfg.width = video.width();
                            str_cfg.height = video.height();
                            str_cfg.fps_idx = 0;
                            str_cfg.stream_type = rs2_stream::RS2_STREAM_DEPTH;
                            str_cfg.fps_list.emplace_back(profile.fps());
                            depth_configs_.emplace_back(str_cfg);
                        }
                    }
                    else if (profile.stream_name() == "Color") {
                        if (profile.format() == rs2_format::RS2_FORMAT_BGR8) {
                            int cfg_idx = 0;
                            bool exists = false;
                            std::string profile_res;
                            profile_res += std::to_string(video.width());
                            profile_res += " x ";
                            profile_res += std::to_string(video.height());
                            for (int i = 0; i < color_configs_.size(); i++) {
                                if (color_configs_.at(i).str_resolution == profile_res) {
                                    cfg_idx = i;
                                    exists = true;
                                    break;
                                }
                            }
                            if (exists) {
                                color_configs_.at(cfg_idx).fps_list.emplace_back(profile.fps());
                            }
                            else {
                                StreamConfig str_cfg;
                                str_cfg.str_stream_name = profile.stream_name();
                                str_cfg.str_resolution = profile_res;
                                str_cfg.width = video.width();
                                str_cfg.height = video.height();
                                str_cfg.fps_idx = 0;
                                str_cfg.stream_type = rs2_stream::RS2_STREAM_COLOR;
                                str_cfg.fps_list.emplace_back(profile.fps());
                                color_configs_.emplace_back(str_cfg);
                            }
                        }
                    }
                }
            }

            // Get Supported Sensor Properties/Options
            std::string sensor_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
            for (auto j = 0; j < RS2_OPTION_COUNT; ++j) {
                auto opt = static_cast<rs2_option>(j);
                if (sensor.supports(opt)) {
                    try {
                        auto range = sensor.get_option_range(opt);
                        Property pr;
                        pr.str_prop = rs2_option_to_string(opt);
                        pr.is_list = false;
                        if (opt == RS2_OPTION_DEPTH_UNITS)
                            continue;
                        else if (opt == RS2_OPTION_STEREO_BASELINE)
                            continue;
                        else if (opt == RS2_OPTION_INTER_CAM_SYNC_MODE)
                            continue;
                        else if (pr.str_prop.find("Exposure Limit") != std::string::npos)
                            continue;
                        else if (pr.str_prop.find("Gain Limit") != std::string::npos)
                            continue;
                        else if (pr.str_prop.find("Sequence") != std::string::npos)
                            continue;
                        else if (opt == RS2_OPTION_VISUAL_PRESET) {
                            pr.is_list = true;
                            for (int i = 0; i < (int)range.max + 1; i++) {
                                pr.opt_list.emplace_back(sensor.get_option_value_description(RS2_OPTION_VISUAL_PRESET, (float) i));
                            }
                        }
                        else if (opt == RS2_OPTION_EMITTER_ENABLED) {
                            pr.is_list = true;
                            for (int i = 0; i < (int)range.max + 1; i++) {
                                pr.opt_list.emplace_back(sensor.get_option_value_description(RS2_OPTION_EMITTER_ENABLED, (float) i));
                            }
                        }
                        else if (opt == RS2_OPTION_POWER_LINE_FREQUENCY) {
                            pr.is_list = true;
                            for (int i = 0; i < (int)range.max + 1; i++) {
                                std::string opt_str = sensor.get_option_value_description(RS2_OPTION_POWER_LINE_FREQUENCY, (float)i);
                                pr.opt_list.emplace_back(opt_str);
                            }
                        }
                        pr.prop = opt;
                        pr.range = range;
                        pr.value = (int)range.def;
                        pr.changed = false;
                        if (sensor_name.find("RGB") != std::string::npos) {
                            if (pr.str_prop.find("Enable") != std::string::npos)
                                color_props_.insert(color_props_.begin(), std::move(pr));
                            else
                                color_props_.emplace_back(pr);
                        }
                        else if (sensor_name.find("Stereo") != std::string::npos) {
                            if (pr.str_prop.find("Enable") != std::string::npos)
                                depth_props_.insert(depth_props_.begin(), pr);
                            else
                                depth_props_.emplace_back(pr);

                        }
                    }
                    catch (const rs2::error & e) {
                        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
                    }
                }
            }
        }
        is_init_ = true;
    }
    init_ = false;
}

void realsense_camera::InitCamera(int index)
{
    is_color_enabled_ = false;
    is_depth_enabled_ = false;
    init_idx_ = index;
    init_ = true;
}

void realsense_camera::ReconfigureDevice_()
{
    std::lock_guard<std::mutex> lck(io_mutex_);
    if (is_init_) {
        if (is_color_streaming_ || is_depth_streaming_) {
            pipe_.stop();
            cfg_.disable_all_streams();
        }
        is_color_streaming_ = false;
        is_depth_streaming_ = false;
        if (is_color_enabled_ || is_depth_enabled_) {
            cfg_.enable_device(rs_dev_serial_);
            pipe_profile_ = cfg_.resolve(pipe_);
            if (is_color_enabled_) {
                cfg_.enable_stream(RS2_STREAM_COLOR,
                                   active_color_cfg_.width,
                                   active_color_cfg_.height,
                                   RS2_FORMAT_BGR8,
                                   active_color_cfg_.fps_list.at(active_color_cfg_.fps_idx));
                is_color_streaming_ = true;
            }
            else
                color_frame_ = cv::Mat();

            if (is_depth_enabled_) {
                cfg_.enable_stream(RS2_STREAM_DEPTH,
                                   active_depth_cfg_.width,
                                   active_depth_cfg_.height,
                                   RS2_FORMAT_Z16,
                                   active_depth_cfg_.fps_list.at(active_depth_cfg_.fps_idx));
                is_depth_streaming_ = true;
            }
            else
                depth_frame_ = cv::Mat();

            pipe_profile_ = pipe_.start(cfg_);

            // Get Current Properties from Device
            for (auto &pr: color_props_) {
                pr.value = GetProperty(RS2_STREAM_COLOR, pr.prop);
            }
            for (auto &pr: depth_props_) {
                pr.value = GetProperty(RS2_STREAM_DEPTH, pr.prop);
            }

            rs2_error *e = nullptr;
            if (is_color_streaming_)
                rs2_get_video_stream_intrinsics(pipe_profile_.get_stream(RS2_STREAM_COLOR), &rsRGBIntrinsics_, &e);
            if (is_depth_streaming_)
                rs2_get_video_stream_intrinsics(pipe_profile_.get_stream(RS2_STREAM_DEPTH), &rsDepthIntrinsics_, &e);

            reconfigure_ = false;
        }
    }
}

bool realsense_camera::EnableStream(StreamConfig& config)
{
    if (config.stream_type == rs2_stream::RS2_STREAM_COLOR) {
        std::lock_guard<std::mutex> lck(io_mutex_);
        active_color_cfg_ = config;
        is_color_enabled_ = true;
        reconfigure_ = true;
        return true;
    }
    else if (config.stream_type == rs2_stream::RS2_STREAM_DEPTH) {
        std::lock_guard<std::mutex> lck(io_mutex_);
        active_depth_cfg_ = config;
        is_depth_enabled_ = true;
        reconfigure_ = true;
        return true;
    }

    return false;
}

void realsense_camera::DisableStream(rs2_stream stream)
{
    if (stream == rs2_stream::RS2_STREAM_COLOR) {
        is_color_enabled_ = false;
        reconfigure_ = true;
    }
    else if (stream == rs2_stream::RS2_STREAM_DEPTH) {
        is_depth_enabled_ = false;
        reconfigure_ = true;
    }
}

void realsense_camera::ProcessStreams()
{
    if (init_)
        InitCamera_();

    if (reconfigure_)
        ReconfigureDevice_();

    if (change_props_)
        ChangeProperties_();

    if (is_init_) {
        if (is_color_streaming_ || is_depth_streaming_) {
            meta_data_.clear();
            nlohmann::json jMeta;
            nlohmann::json intrinsic;
            meta_data_["data_type"] = "metadata";

            auto frames = pipe_.wait_for_frames();
            if (is_color_streaming_ && is_depth_streaming_) {
                if (frames.get_color_frame().get_data_size() > 0 && frames.get_depth_frame().get_data_size() > 0) {
                    if (stream_alignment_ == STREAM_ALIGN_COLOR_TO_DEPTH) {
                        frames = align_to_depth->process(frames);
                    } else if (stream_alignment_ == STREAM_ALIGN_DEPTH_TO_COLOR) {
                        frames = align_to_color->process(frames);
                    }
                }
            }
            if (is_color_streaming_) {
                auto color = frames.get_color_frame();
                if (color) {
                    if (color.get_data_size() > 0) {
                        color_frame_ = cv::Mat(cv::Size(color.get_width(), color.get_height()), CV_8UC3, (void *)color.get_data(), cv::Mat::AUTO_STEP);
                    }
                }
                nlohmann::json ref;
                ref["w"] = color.get_width();
                ref["h"] = color.get_height();
                meta_data_["ref_frame"] = ref;
                nlohmann::json color_frame;
                if (color.supports_frame_metadata(RS2_FRAME_METADATA_ACTUAL_FPS))
                    color_frame["fps"] = color.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_FPS);
                if (color.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_COUNTER))
                    color_frame["frame_num"] = color.get_frame_metadata(RS2_FRAME_METADATA_FRAME_COUNTER);
                if (color.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP))
                    color_frame["timestamp"] = color.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP);
                jMeta["color_frame"] = color_frame;
                nlohmann::json color_int;
                color_int["width"] = rsRGBIntrinsics_.width;
                color_int["height"] = rsRGBIntrinsics_.height;
                color_int["fx"] = rsRGBIntrinsics_.fx;
                color_int["fy"] = rsRGBIntrinsics_.fy;
                color_int["ppx"] = rsRGBIntrinsics_.ppx;
                color_int["ppy"] = rsRGBIntrinsics_.ppy;
                color_int["coeffs"] = rsRGBIntrinsics_.coeffs;
                intrinsic["color"] = color_int;
            }
            if (is_depth_streaming_) {
                bool is_decimate = false;
                float decimate_val = 1.0;
                auto depth = frames.get_depth_frame();
                if (depth) {
                    if (depth.get_data_size() > 0) {
                        rs2::frame filtered = depth;
                        bool revert_disparity = false;
                        for (auto&& filter : filters_)
                        {
                            if (filter.is_enabled)
                            {
                                filtered = filter.filter.process(filtered);
                                if (filter.filter_name == "Disparity") {
                                    revert_disparity = true;
                                }
                                else if (filter.filter_name == "Decimate") {
                                    is_decimate = true;
                                    decimate_val = rs_dec_filter_.get_option(RS2_OPTION_FILTER_MAGNITUDE);
                                }
                            }
                        }
                        if (revert_disparity)
                        {
                            filtered = rs_disparity_to_depth_->process(filtered);
                        }
                        filtered_data_.enqueue(filtered);
                        filtered_data_.poll_for_frame(&filtered);
                        depth = filtered.as<rs2::depth_frame>();
                        if (depth) {
                            if (depth.get_data_size() > 0) {
                                depth_frame_ = cv::Mat(cv::Size(depth.get_width(), depth.get_height()), CV_16U, (void *)depth.get_data(), cv::Mat::AUTO_STEP);
                            }
                        }
                        nlohmann::json refDepth;
                        refDepth["w"] = depth.get_width();
                        refDepth["h"] = depth.get_height();
                        meta_data_["depth_frame"] = refDepth;
                        nlohmann::json depth_frame;
                        if (depth.supports_frame_metadata(RS2_FRAME_METADATA_ACTUAL_FPS))
                            depth_frame["fps"] = depth.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_FPS);
                        if (depth.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_COUNTER))
                            depth_frame["frame_num"] = depth.get_frame_metadata(RS2_FRAME_METADATA_FRAME_COUNTER);
                        if (depth.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP))
                            depth_frame["timestamp"] = depth.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP);
                        jMeta["depth_frame"] = depth_frame;
                        nlohmann::json depth_int;
                        if (is_decimate) {
                            depth_int["width"] = rsDepthIntrinsics_.width / (int)decimate_val;
                            depth_int["height"] = rsDepthIntrinsics_.height / (int)decimate_val;
                            depth_int["fx"] = rsDepthIntrinsics_.fx / decimate_val;
                            depth_int["fy"] = rsDepthIntrinsics_.fy / decimate_val;
                            depth_int["ppx"] = rsDepthIntrinsics_.ppx / decimate_val;
                            depth_int["ppy"] = rsDepthIntrinsics_.ppy / decimate_val;
                        } else {
                            depth_int["width"] = rsDepthIntrinsics_.width;
                            depth_int["height"] = rsDepthIntrinsics_.height;
                            depth_int["fx"] = rsDepthIntrinsics_.fx;
                            depth_int["fy"] = rsDepthIntrinsics_.fy;
                            depth_int["ppx"] = rsDepthIntrinsics_.ppx;
                            depth_int["ppy"] = rsDepthIntrinsics_.ppy;
                        }
                        depth_int["coeffs"] = rsDepthIntrinsics_.coeffs;
                        intrinsic["depth"] = depth_int;
                    }
                }
            }
            if (!intrinsic.empty())
                jMeta["intrinsics"] = intrinsic;
            if (!jMeta.empty())
                meta_data_["data"].emplace_back(jMeta);
        }
    }
}

cv::Mat &realsense_camera::GetFrame(rs2_stream stream)
{
    if (stream == rs2_stream::RS2_STREAM_DEPTH)
        return depth_frame_;

    return color_frame_;
}

std::vector<StreamConfig> *realsense_camera::GetStreamConfigList(rs2_stream stream_type)
{
    if (stream_type == rs2_stream::RS2_STREAM_DEPTH)
        return &depth_configs_;

    return &color_configs_;
}

bool realsense_camera::IsInit()
{
    return is_init_;
}

std::vector<Property> *realsense_camera::GetPropertyList(rs2_stream stream_type)
{
    if (stream_type == RS2_STREAM_DEPTH)
        return &depth_props_;

    return &color_props_;
}

void realsense_camera::SetProperty(rs2_stream stream_type, rs2_option prop, bool immediate_mode)
{
    if (stream_type == RS2_STREAM_COLOR) {
        for (auto &pr : color_props_) {
            if (prop == RS2_OPTION_WHITE_BALANCE && pr.prop == RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE)
                pr.value = (int)false;
            else if (prop == RS2_OPTION_EXPOSURE && pr.prop == RS2_OPTION_ENABLE_AUTO_EXPOSURE)
                pr.value = (int)false;
            if (pr.prop == prop) {
                std::lock_guard<std::mutex> lck(io_mutex_);
                if (immediate_mode) {
                    rs_color_sensor_.set_option(pr.prop, (float)pr.value);
                }
                else {
                    pr.changed = true;
                    change_props_ = true;
                }
            }
        }
        return;
    }
    else if (stream_type == RS2_STREAM_DEPTH) {
        for (auto &pr: depth_props_) {
            if (prop == RS2_OPTION_WHITE_BALANCE && pr.prop == RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE)
                pr.value = (int)false;
            else if (prop == RS2_OPTION_EXPOSURE && pr.prop == RS2_OPTION_ENABLE_AUTO_EXPOSURE)
                pr.value = (int)false;
            if (pr.prop == prop) {
                std::lock_guard<std::mutex> lck(io_mutex_);
                if (immediate_mode) {
                    rs_depth_sensor_.set_option(pr.prop, (float)pr.value);
                }
                else {
                    pr.changed = true;
                    change_props_ = true;
                }
            }
        }
        return;
    }
}

int realsense_camera::GetProperty(rs2_stream stream_type, rs2_option prop)
{
    if (is_init_) {
        if (is_color_streaming_ || is_depth_streaming_) {
            if (stream_type == RS2_STREAM_COLOR) {
                return (int)rs_color_sensor_.get_option(prop);
            } else if (stream_type == RS2_STREAM_DEPTH) {
                return (int)rs_depth_sensor_.get_option(prop);
            }
        }
    }
}

void realsense_camera::ChangeProperties_()
{
    std::lock_guard<std::mutex> lck(io_mutex_);
    for (auto &pr : color_props_) {
        if (pr.changed) {
            rs_color_sensor_.set_option(pr.prop, (float)pr.value);
            pr.changed = false;
        }
    }
    for (auto &pr : depth_props_) {
        if (pr.changed) {
            rs_depth_sensor_.set_option(pr.prop, (float)pr.value);
            pr.changed = false;
        }
    }
    change_props_ = false;
}

void realsense_camera::ResetProperties(rs2_stream stream_type)
{
    if (stream_type == RS2_STREAM_COLOR) {
        for (auto &pr: color_props_) {
            pr.value = (int)pr.range.def;
            pr.changed = true;
        }
        change_props_ = true;
    }
    else if (stream_type == RS2_STREAM_DEPTH) {
        for (auto &pr: depth_props_) {
            pr.value = (int)pr.range.def;
            pr.changed = true;
        }
        change_props_ = true;
    }
}

std::string realsense_camera::GetDeviceSerial(int index)
{
    std::string cam_serial;
    if (index > 0 && index < camera_dev_list_.size() + 1) {
        cam_serial = camera_dev_list_.at(index - 1).get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    }
    return cam_serial;
}

std::vector<filter_options> *realsense_camera::GetFilterList(rs2_stream stream_type)
{
    if (stream_type == RS2_STREAM_DEPTH)
        return &filters_;

    return nullptr;
}

nlohmann::json &realsense_camera::GetMetaData()
{
    return meta_data_;
}

void realsense_camera::SetStreamAlignment(StreamAlignment align)
{
    stream_alignment_ = align;
    rs2_error *e = nullptr;
    if (stream_alignment_ == STREAM_ALIGN_DEPTH_TO_COLOR && is_color_streaming_) {
        rs2_get_video_stream_intrinsics(pipe_profile_.get_stream(RS2_STREAM_COLOR), &rsDepthIntrinsics_, &e);
    } else {
        rs2_get_video_stream_intrinsics(pipe_profile_.get_stream(RS2_STREAM_DEPTH), &rsDepthIntrinsics_, &e);
    }
}

bool realsense_camera::IsReconfiguring() const
{
    return reconfigure_;
}

bool FilterOptions::is_all_integers(const rs2::option_range& range)
{
    const auto is_integer = [](float f)
    {
        return (fabs(fmod(f, 1)) < std::numeric_limits<float>::min());
    };

    return is_integer(range.min) && is_integer(range.max) &&
        is_integer(range.def) && is_integer(range.step);
}

filter_options::filter_options(const std::string name, rs2::filter& flt) :
    filter_name(name),
    filter(flt),
    is_enabled(false)
{
    const std::array<rs2_option, 6> possible_filter_options = {
        RS2_OPTION_FILTER_MAGNITUDE,
        RS2_OPTION_FILTER_SMOOTH_ALPHA,
        RS2_OPTION_MIN_DISTANCE,
        RS2_OPTION_MAX_DISTANCE,
        RS2_OPTION_FILTER_SMOOTH_DELTA,
        RS2_OPTION_HOLES_FILL
    };

    for (rs2_option opt : possible_filter_options)
    {
        if (flt.supports(opt))
        {
            rs2::option_range range = flt.get_option_range(opt);
            supported_options[opt].is_list = false;
            if (opt == RS2_OPTION_HOLES_FILL) {
                supported_options[opt].is_list = true;
                for (int i = 0; i < (int)range.max + 1; i++) {
                    supported_options[opt].opt_list.emplace_back(flt.get_option_value_description(RS2_OPTION_HOLES_FILL, (float)i));
                }
            }
            supported_options[opt].range = range;
            supported_options[opt].value = range.def;
            supported_options[opt].is_int = FilterOptions::is_all_integers(range);
            supported_options[opt].description = flt.get_option_description(opt);
            std::string opt_name = flt.get_option_name(opt);
            supported_options[opt].name = name;
            supported_options[opt].name += "_";
            supported_options[opt].name += opt_name;
            std::string prefix = "Filter ";
            supported_options[opt].label = opt_name;
        }
    }
}

filter_options::filter_options(filter_options&& other) noexcept :
    filter_name(std::move(other.filter_name)),
    filter(other.filter),
    supported_options(std::move(other.supported_options)),
    is_enabled(other.is_enabled)
{
}


