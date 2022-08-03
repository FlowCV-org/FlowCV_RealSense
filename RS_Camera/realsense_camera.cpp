//
// FlowCV Plugin - Realsense Camera
// Written By Richard Wardlow
//

#include "realsense_camera.hpp"
#include "imgui.h"
#include "imgui_instance_helper.hpp"

using namespace DSPatch;
using namespace DSPatchables;

int32_t global_inst_counter = 0;

namespace DSPatch::DSPatchables::internal
{
class RealsenseCamera
{
};
}  // namespace DSPatch

RealsenseCamera::RealsenseCamera()
    : Component( ProcessOrder::OutOfOrder )
    , p( new internal::RealsenseCamera() )
{
    // Name and Category
    SetComponentName_("Realsense_Camera");
    SetComponentCategory_(Category::Category_Source);
    SetComponentAuthor_("Richard");
    SetComponentVersion_("0.1.0");
    SetInstanceCount(global_inst_counter);
    global_inst_counter++;

    // 1 inputs
    SetInputCount_( 0 );

    // 1 outputs
    SetOutputCount_( 3, {"color", "depth", "metadata"}, {IoType::Io_Type_CvMat, IoType::Io_Type_CvMat, IoType::Io_Type_JSON} );

    rs_cam_idx_ = 0;
    cam_list_.emplace_back("None");
    init_once_ = false;
    cam_is_init_ = false;
    should_init_ = false;
    first_list_refresh_ = true;
    enable_color_ = false;
    enable_depth_ = false;
    is_valid_color_mode_ = true;
    is_valid_depth_mode_ = true;
    depth_controls_.auto_exposure = true;
    depth_controls_.exposure = 20000;
    depth_controls_.gain = 60;
    laser_power_ = 150;
    preset_idx_ = 0;
    should_set_params_ = false;
    enable_decimation_filter_ = false;
    enable_threshold_filter_ = false;
    enable_spatial_filter_ = false;
    enable_temporal_filter_ = false;
    enable_hole_fill_filter_ = false;
    align_color_to_depth_ = true;
    align_depth_to_color_ = false;
    decimation_amount_ = 2;
    thresh_min_ = 0;
    thresh_max_ = 8;
    spatial_mag_ = 2;
    spatial_alpha_ = 0.54f;
    spatial_delta_ = 33;
    spatial_hole_fill_ = 2;
    temporal_alpha_ = 0.5f;
    temporal_delta_ = 32;
    temporal_persistency_ = 4;
    hole_fill_option_ = 1;
    rs_depth_to_disparity_ = std::make_unique<rs2::disparity_transform>(true);
    rs_disparity_to_depth_ = std::make_unique<rs2::disparity_transform>(false);

    color_modes_ = {VideoMode{320, 180},
                    VideoMode{320, 240},
                    VideoMode{424, 240},
                    VideoMode{640, 360},
                    VideoMode{640, 480},
                    VideoMode{848, 480},
                    VideoMode{960, 540},
                    VideoMode{1280, 720}};
    color_mode_idx_ = (int)color_modes_.size() - 1;

    depth_modes_ = {VideoMode{256, 144},
                    VideoMode{424, 240},
                    VideoMode{480, 270},
                    VideoMode{640, 360},
                    VideoMode{640, 480},
                    VideoMode{848, 480},
                    VideoMode{960, 540},
                    VideoMode{1280, 720}};
    depth_mode_idx_ = (int)depth_modes_.size() - 1;

    color_fps_ = {60, 30, 15, 6};
    color_fps_idx_ = 1;

    depth_fps_ = {90, 60, 30, 25, 15, 6};
    depth_fps_idx_ = 2;

    SetFilterOptions();

    SetEnabled(true);

}

void RealsenseCamera::RefreshDeviceList_()
{
    devices_.clear();
    cam_list_.clear();
    cam_list_.emplace_back("None");

    rs2::context ctx;
    auto devices_list = ctx.query_devices();
    size_t device_count = devices_list.size();
    if (!device_count)
    {
        std::cout << "No device detected. Is it plugged in?\n";
        return;
    }

    for ( auto i = 0u; i < device_count; i++)
    {
        try
        {
            auto dev = devices_list[i];
            std::string cam_name;
            cam_name = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            cam_name += " - ";
            cam_name += dev.get_info(RS2_CAMERA_INFO_NAME);
            devices_.emplace_back(dev);
            cam_list_.emplace_back(cam_name);
        }
        catch (const std::exception & e)
        {
            std::cout << "Could not create device - " << e.what() <<" . Check SDK logs for details" << std::endl;
        }
        catch(...)
        {
            std::cout << "Failed to created device. Check SDK logs for details" << std::endl;
        }
    }
}

bool RealsenseCamera::IsValidProfile(rs2_stream stream_type, int width, int height, int fps)
{
    for (auto&& sensor : rsDevice_.query_sensors()) {
        for (auto &&sp: sensor.get_stream_profiles()) {
            if (stream_type == sp.stream_type()) {
                if (auto vp = sp.as<rs2::video_stream_profile>()) {
                    if (vp.width() == width && vp.height() == height && sp.fps() == fps) {
                        if (stream_type == RS2_STREAM_COLOR)
                            rsRGBSensor_ = sensor;
                        else if (stream_type == RS2_STREAM_DEPTH)
                            rsStereoSensor_ = sensor;
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool RealsenseCamera::InitCamera_()
{
    if (cam_is_init_) {
        preset_list_.clear();
        pipe_.stop();
        cfg_.disable_all_streams();
    }

    cam_is_init_ = false;
    if (rs_cam_idx_ > 0) {
        if (rs_cam_idx_ < devices_.size() + 1) {
            if (enable_color_ || enable_depth_) {
                rsDevice_ = devices_.at(rs_cam_idx_ - 1);
                rs_cam_serial_ = rsDevice_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                cfg_.enable_device(rs_cam_serial_);
                pipeProfile_ = cfg_.resolve(pipe_);
                bool color_is_init = false;
                bool depth_is_init = false;
                if (enable_color_) {
                    // Verify Current Settings first
                    is_valid_color_mode_ = IsValidProfile(RS2_STREAM_COLOR,
                                                          color_modes_.at(color_mode_idx_).width,
                                                          color_modes_.at(color_mode_idx_).height,
                                                          color_fps_.at(color_fps_idx_));
                    if (is_valid_color_mode_) {
                        cfg_.enable_stream(RS2_STREAM_COLOR,
                                          color_modes_.at(color_mode_idx_).width,
                                          color_modes_.at(color_mode_idx_).height,
                                          RS2_FORMAT_BGR8,
                                          color_fps_.at(color_fps_idx_));

                        if (should_set_params_) {
                            rsRGBSensor_.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, (float)color_controls_.auto_exposure);
                            if (!color_controls_.auto_exposure) {
                                rsRGBSensor_.set_option(RS2_OPTION_EXPOSURE, (float)color_controls_.exposure);
                                rsRGBSensor_.set_option(RS2_OPTION_GAIN, (float)color_controls_.gain);
                            }
                        }
                        else {
                            color_controls_.auto_exposure = (int) rsRGBSensor_.get_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE);
                            color_controls_.exposure = (int) rsRGBSensor_.get_option(RS2_OPTION_EXPOSURE);
                            color_controls_.gain = (int) rsRGBSensor_.get_option(RS2_OPTION_GAIN);
                        }
                        color_is_init = true;
                    }
                }
                if (enable_depth_) {
                    // Verify Current Settings first
                    is_valid_depth_mode_ = IsValidProfile(RS2_STREAM_DEPTH,
                                                          depth_modes_.at(depth_mode_idx_).width,
                                                          depth_modes_.at(depth_mode_idx_).height,
                                                          depth_fps_.at(depth_fps_idx_));
                    if (is_valid_depth_mode_) {
                        cfg_.enable_stream(RS2_STREAM_DEPTH,
                                          depth_modes_.at(depth_mode_idx_).width,
                                          depth_modes_.at(depth_mode_idx_).height,
                                          RS2_FORMAT_Z16,
                                          depth_fps_.at(depth_fps_idx_));
                        auto optRange = rsStereoSensor_.get_option_range(RS2_OPTION_VISUAL_PRESET);
                        for (int i = (int)optRange.min; i < (int)optRange.max; i++) {
                            auto optDesc = rsStereoSensor_.get_option_value_description(RS2_OPTION_VISUAL_PRESET, (float)i);
                            preset_list_.emplace_back(optDesc);
                        }
                        if (should_set_params_) {
                            rsStereoSensor_.set_option(RS2_OPTION_VISUAL_PRESET, (float)preset_idx_);
                            rsStereoSensor_.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, (float)depth_controls_.auto_exposure);
                            rsStereoSensor_.set_option(RS2_OPTION_LASER_POWER, (float)laser_power_);
                            if (!depth_controls_.auto_exposure) {
                                rsStereoSensor_.set_option(RS2_OPTION_EXPOSURE, (float)depth_controls_.exposure);
                                rsStereoSensor_.set_option(RS2_OPTION_GAIN, (float)depth_controls_.gain);
                            }
                        }
                        else {
                            preset_idx_ = (int) rsStereoSensor_.get_option(RS2_OPTION_VISUAL_PRESET);
                            depth_controls_.auto_exposure = (int) rsStereoSensor_.get_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE);
                            depth_controls_.exposure = (int) rsStereoSensor_.get_option(RS2_OPTION_EXPOSURE);
                            depth_controls_.gain = (int) rsStereoSensor_.get_option(RS2_OPTION_GAIN);
                            laser_power_ = (int) rsStereoSensor_.get_option(RS2_OPTION_LASER_POWER);
                        }
                        depth_is_init = true;
                    }
                }
                if (is_valid_color_mode_ || is_valid_depth_mode_) {
                    should_set_params_ = false;
                    pipeProfile_ = pipe_.start(cfg_);
                }

                rs2_error *e = nullptr;
                if (enable_color_ && is_valid_color_mode_ && color_is_init)
                    rs2_get_video_stream_intrinsics(pipeProfile_.get_stream(RS2_STREAM_COLOR), &rsRGBIntrinsics_, &e);
                if (enable_depth_ && is_valid_depth_mode_ && depth_is_init)
                    rs2_get_video_stream_intrinsics(pipeProfile_.get_stream(RS2_STREAM_DEPTH), &rsDepthIntrinsics_, &e);

                if (is_valid_color_mode_ || is_valid_depth_mode_)
                    return true;
            }
        }
    }

    return false;
}

void RealsenseCamera::SetFilterOptions()
{
    rs_dec_filter_.set_option(RS2_OPTION_FILTER_MAGNITUDE, (float)decimation_amount_);
    rs_thr_filter_.set_option(RS2_OPTION_MIN_DISTANCE, (float)thresh_min_);
    rs_thr_filter_.set_option(RS2_OPTION_MAX_DISTANCE, (float)thresh_max_);
    rs_spat_filter_.set_option(RS2_OPTION_FILTER_MAGNITUDE, (float)spatial_mag_);
    rs_spat_filter_.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, spatial_alpha_);
    rs_spat_filter_.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, (float)spatial_delta_);
    rs_spat_filter_.set_option(RS2_OPTION_HOLES_FILL, (float)spatial_hole_fill_);
    rs_temp_filter_.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, temporal_alpha_);
    rs_temp_filter_.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, (float)temporal_delta_);
    rs_temp_filter_.set_option(RS2_OPTION_HOLES_FILL, (float)temporal_persistency_);
    rs_hole_fill_filter_.set_option(RS2_OPTION_HOLES_FILL, (float)hole_fill_option_);
}

void RealsenseCamera::Process_( SignalBus const& inputs, SignalBus& outputs )
{
    if (io_mutex_.try_lock()) { // Try lock so other threads will skip if locked instead of waiting
        if (should_init_) {
            bool isInit = InitCamera_();
            if (isInit) {
                should_init_ = false;
                init_once_ = false;
                cam_is_init_ = true;
            }
        }

        if (cam_is_init_ && !init_once_) {
            nlohmann::json json_out;
            nlohmann::json jMeta;
            nlohmann::json intrinsic;
            json_out["data_type"] = "metadata";

            auto frames = pipe_.wait_for_frames();
            if (enable_color_ && is_valid_color_mode_) {
                auto color = frames.get_color_frame();
                if (enable_depth_ && is_valid_depth_mode_) {
                    auto depth = frames.get_depth_frame();
                    if (color && depth) {
                        if (align_color_to_depth_) {
                            rs2::align align_to_depth(RS2_STREAM_DEPTH);
                            frames = align_to_depth.process(frames);
                        } else if (align_depth_to_color_) {
                            rs2::align align_to_color(RS2_STREAM_COLOR);
                            frames = align_to_color.process(frames);
                        }
                    }
                }
                color = frames.get_color_frame();
                if (color) {
                    if (color.get_data_size() > 0) {
                        auto frame = cv::Mat(cv::Size(color.get_width(), color.get_height()), CV_8UC3, (void *) color.get_data(), cv::Mat::AUTO_STEP);
                        nlohmann::json ref;
                        ref["w"] = color.get_width();
                        ref["h"] = color.get_height();
                        json_out["ref_frame"] = ref;
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
                        if (!frame.empty()) {
                            frame.copyTo(frame_);
                            outputs.SetValue(0, frame_);
                        }
                    }
                }
            }
            if (enable_depth_ && is_valid_depth_mode_) {
                auto depth = frames.get_depth_frame();
                if (depth) {
                    if (depth.get_data_size() > 0) {
                        rs2::frame filtered = depth;
                        if (enable_decimation_filter_) {
                            filtered = rs_dec_filter_.process(filtered);
                        }
                        if (enable_threshold_filter_) {
                            filtered = rs_thr_filter_.process(filtered);
                        }
                        if (enable_spatial_filter_ || enable_temporal_filter_) {
                            filtered = rs_depth_to_disparity_->process(filtered);
                            if (enable_spatial_filter_) {
                                filtered = rs_spat_filter_.process(filtered);
                            }
                            if (enable_temporal_filter_) {
                                filtered = rs_temp_filter_.process(filtered);
                            }
                            filtered = rs_disparity_to_depth_->process(filtered);
                        }

                        if (enable_hole_fill_filter_) {
                            filtered = rs_hole_fill_filter_.process(filtered);
                        }

                        // Enqueue Filtering
                        filtered_data_.enqueue(filtered);

                        // Get Filtered Frame From Queue
                        filtered_data_.poll_for_frame(&filtered);
                        depth = filtered.as<rs2::depth_frame>();
                        if (depth) {
                            if (depth.get_data_size() > 0) {
                                auto dFrame = cv::Mat(cv::Size(depth.get_width(), depth.get_height()), CV_16U, (void *) depth.get_data(), cv::Mat::AUTO_STEP);
                                nlohmann::json refDepth;
                                refDepth["w"] = depth.get_width();
                                refDepth["h"] = depth.get_height();
                                json_out["depth_frame"] = refDepth;
                                nlohmann::json depth_frame;
                                if (depth.supports_frame_metadata(RS2_FRAME_METADATA_ACTUAL_FPS))
                                    depth_frame["fps"] = depth.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_FPS);
                                if (depth.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_COUNTER))
                                    depth_frame["frame_num"] = depth.get_frame_metadata(RS2_FRAME_METADATA_FRAME_COUNTER);
                                if (depth.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP))
                                    depth_frame["timestamp"] = depth.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP);
                                jMeta["depth_frame"] = depth_frame;
                                nlohmann::json depth_int;
                                rs2_error *e = nullptr;
                                if (align_depth_to_color_ && enable_color_ && is_valid_color_mode_) {
                                    rs2_get_video_stream_intrinsics(pipeProfile_.get_stream(RS2_STREAM_COLOR), &rsDepthIntrinsics_, &e);
                                } else {
                                    rs2_get_video_stream_intrinsics(pipeProfile_.get_stream(RS2_STREAM_DEPTH), &rsDepthIntrinsics_, &e);
                                }
                                if (enable_decimation_filter_) {
                                    depth_int["width"] = rsDepthIntrinsics_.width / decimation_amount_;
                                    depth_int["height"] = rsDepthIntrinsics_.height / decimation_amount_;
                                    depth_int["fx"] = rsDepthIntrinsics_.fx / (float) decimation_amount_;
                                    depth_int["fy"] = rsDepthIntrinsics_.fy / (float) decimation_amount_;
                                    depth_int["ppx"] = rsDepthIntrinsics_.ppx / (float) decimation_amount_;
                                    depth_int["ppy"] = rsDepthIntrinsics_.ppy / (float) decimation_amount_;
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
                                if (!dFrame.empty()) {
                                    dFrame.copyTo(dFrame_);
                                    outputs.SetValue(1, dFrame_);
                                }
                            }
                        }
                    }
                }
            }

            if (!intrinsic.empty())
                jMeta["intrinsics"] = intrinsic;
            if (!jMeta.empty())
                json_out["data"].emplace_back(jMeta);

            if (!json_out.empty())
                outputs.SetValue(2, json_out);
        }
        io_mutex_.unlock();
    }
}

bool RealsenseCamera::HasGui(int interface)
{
    if (interface == (int)FlowCV::GuiInterfaceType_Controls) {
        return true;
    }

    return false;
}

void RealsenseCamera::UpdateGui(void *context, int interface)
{
    auto *imCurContext = (ImGuiContext *)context;
    ImGui::SetCurrentContext(imCurContext);

    if (first_list_refresh_) {
        RefreshDeviceList_();
        first_list_refresh_ = false;
    }

    if (interface == (int)FlowCV::GuiInterfaceType_Controls) {
        std::lock_guard<std::mutex> lck(io_mutex_);
        if (ImGui::Button(CreateControlString("Refresh RS Cam List", GetInstanceName()).c_str())) {
            RefreshDeviceList_();
        }
        ImGui::Separator();
        if (ImGui::Combo(CreateControlString("RS Cameras", GetInstanceName()).c_str(), &rs_cam_idx_, [](void* data, int idx, const char** out_text) {
            *out_text = ((const std::vector<std::string>*)data)->at(idx).c_str();
            return true;
        }, (void*)&cam_list_, (int)cam_list_.size())) {
            enable_color_ = false;
            enable_depth_ = false;
            init_once_ = true;
        }
        if (rs_cam_idx_ > 0) {
            ImGui::Separator();
            if (ImGui::TreeNode("Color Sensor")) {
                if (ImGui::Checkbox(CreateControlString("Enable Color Sensor", GetInstanceName()).c_str(), &enable_color_)) {
                    init_once_ = true;
                    should_init_ = true;
                }
                if (enable_color_) {
                    if (!is_valid_color_mode_) {
                        ImGui::TextColored(ImVec4(1.0, 0.0, 0.0, 1.0), "Invalid Mode Selected");
                    }
                    std::vector<std::string> cmodes;
                    for (auto m : color_modes_) {
                        std::string res;
                        res += std::to_string(m.width);
                        res += " x ";
                        res += std::to_string(m.height);
                        cmodes.emplace_back(res);
                    }
                    ImGui::SetNextItemWidth(100);
                    if (ImGui::Combo(CreateControlString("Color Resolution", GetInstanceName()).c_str(), &color_mode_idx_, [](void* data, int idx, const char** out_text) {
                        *out_text = ((const std::vector<std::string>*)data)->at(idx).c_str();
                        return true;
                    }, (void*)&cmodes, (int)cmodes.size())) {
                        init_once_ = true;
                        should_init_ = true;
                    }
                    std::vector<std::string> fpsList;
                    for (auto m : color_fps_) {
                        std::string res = std::to_string(m);
                        fpsList.emplace_back(res);
                    }
                    ImGui::SetNextItemWidth(100);
                    if (ImGui::Combo(CreateControlString("Color FPS", GetInstanceName()).c_str(), &color_fps_idx_, [](void* data, int idx, const char** out_text) {
                        *out_text = ((const std::vector<std::string>*)data)->at(idx).c_str();
                        return true;
                    }, (void*)&fpsList, (int)fpsList.size())) {
                        init_once_ = true;
                        should_init_ = true;
                    }
                    if (enable_depth_) {
                        if (ImGui::Checkbox(CreateControlString("Align To Depth", GetInstanceName()).c_str(), &align_color_to_depth_)){
                            if (align_color_to_depth_)
                                align_depth_to_color_ = false;
                        }
                    }
                    if (ImGui::TreeNode("Color Controls")) {
                        if (ImGui::Checkbox(CreateControlString("Auto Exposure", GetInstanceName()).c_str(), &color_controls_.auto_exposure)) {
                            if (enable_color_ && is_valid_color_mode_) {
                                rsRGBSensor_.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, (float)color_controls_.auto_exposure);
                            }
                        }
                        if (!color_controls_.auto_exposure) {
                            ImGui::SetNextItemWidth(120);
                            if (ImGui::DragInt(CreateControlString("Exposure", GetInstanceName()).c_str(), &color_controls_.exposure, 1.0f, 1, 200000)) {
                                if (enable_color_ && is_valid_color_mode_) {
                                    auto optRange = rsRGBSensor_.get_option_range(RS2_OPTION_EXPOSURE);
                                    if ((float) color_controls_.exposure < optRange.min)
                                        color_controls_.exposure = (int) optRange.min;
                                    else if ((float) color_controls_.exposure > optRange.max)
                                        color_controls_.exposure = (int) optRange.max;
                                    rsRGBSensor_.set_option(RS2_OPTION_EXPOSURE, (float) color_controls_.exposure);
                                }
                            }
                            ImGui::SetNextItemWidth(120);
                            if (ImGui::DragInt(CreateControlString("Gain", GetInstanceName()).c_str(), &color_controls_.gain, 1.0f, 16, 248)) {
                                if (enable_color_ && is_valid_color_mode_) {
                                    auto optRange = rsRGBSensor_.get_option_range(RS2_OPTION_GAIN);
                                    if ((float)color_controls_.gain < optRange.min)
                                        color_controls_.gain = (int)optRange.min;
                                    else if ((float)color_controls_.gain > optRange.max)
                                        color_controls_.gain = (int)optRange.max;
                                    rsRGBSensor_.set_option(RS2_OPTION_GAIN, (float)color_controls_.gain);
                                }
                            }
                        }
                        ImGui::TreePop();
                    }
                }
                ImGui::TreePop();
            }
            ImGui::Separator();
            if (ImGui::TreeNode("Depth Sensor")) {
                if (ImGui::Checkbox(CreateControlString("Enable Depth Sensor", GetInstanceName()).c_str(), &enable_depth_)) {
                    init_once_ = true;
                    should_init_ = true;
                }
                if (enable_depth_) {
                    if (!is_valid_depth_mode_) {
                        ImGui::TextColored(ImVec4(1.0, 0.0, 0.0, 1.0), "Invalid Mode Selected");
                    }
                    std::vector<std::string> dmodes;
                    for (auto m : depth_modes_) {
                        std::string res;
                        res += std::to_string(m.width);
                        res += " x ";
                        res += std::to_string(m.height);
                        dmodes.emplace_back(res);
                    }
                    ImGui::SetNextItemWidth(100);
                    if (ImGui::Combo(CreateControlString("Depth Resolution", GetInstanceName()).c_str(), &depth_mode_idx_, [](void* data, int idx, const char** out_text) {
                        *out_text = ((const std::vector<std::string>*)data)->at(idx).c_str();
                        return true;
                    }, (void*)&dmodes, (int)dmodes.size())) {
                        init_once_ = true;
                        should_init_ = true;
                    }
                    std::vector<std::string> fpsList;
                    for (auto m : depth_fps_) {
                        std::string res = std::to_string(m);
                        fpsList.emplace_back(res);
                    }
                    ImGui::SetNextItemWidth(100);
                    if (ImGui::Combo(CreateControlString("Depth FPS", GetInstanceName()).c_str(), &depth_fps_idx_, [](void* data, int idx, const char** out_text) {
                        *out_text = ((const std::vector<std::string>*)data)->at(idx).c_str();
                        return true;
                    }, (void*)&fpsList, (int)fpsList.size())) {
                        init_once_ = true;
                        should_init_ = true;
                    }
                    if (enable_color_) {
                        if (ImGui::Checkbox(CreateControlString("Align To Color", GetInstanceName()).c_str(), &align_depth_to_color_)){
                            if (align_depth_to_color_)
                                align_color_to_depth_ = false;
                        }
                    }
                    if (ImGui::TreeNode("Depth Controls")) {
                        ImGui::SetNextItemWidth(120);
                        if (ImGui::Combo(CreateControlString("Preset", GetInstanceName()).c_str(), &preset_idx_, [](void* data, int idx, const char** out_text) {
                            *out_text = ((const std::vector<std::string>*)data)->at(idx).c_str();
                            return true;
                        }, (void*)&preset_list_, (int)preset_list_.size())) {
                            if (enable_depth_ && is_valid_depth_mode_) {
                                rsStereoSensor_.set_option(RS2_OPTION_VISUAL_PRESET, (float)preset_idx_);
                            }
                        }
                        ImGui::Separator();
                        if (ImGui::Checkbox(CreateControlString("Auto Exposure", GetInstanceName()).c_str(), &depth_controls_.auto_exposure)) {
                            if (enable_depth_ && is_valid_depth_mode_) {
                                rsStereoSensor_.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, (float)depth_controls_.auto_exposure);
                            }
                        }
                        if (!depth_controls_.auto_exposure) {
                            ImGui::SetNextItemWidth(120);
                            if (ImGui::DragInt(CreateControlString("Exposure", GetInstanceName()).c_str(), &depth_controls_.exposure, 1.0f, 1, 200000)) {
                                if (enable_depth_ && is_valid_depth_mode_) {
                                    auto optRange = rsStereoSensor_.get_option_range(RS2_OPTION_EXPOSURE);
                                    if ((float)depth_controls_.exposure < optRange.min)
                                        depth_controls_.exposure = (int)optRange.min;
                                    else if ((float)depth_controls_.exposure > optRange.max)
                                        depth_controls_.exposure = (int)optRange.max;
                                    rsStereoSensor_.set_option(RS2_OPTION_EXPOSURE, (float)depth_controls_.exposure);
                                }
                            }
                            ImGui::SetNextItemWidth(120);
                            if (ImGui::DragInt(CreateControlString("Gain", GetInstanceName()).c_str(), &depth_controls_.gain, 1.0f, 16, 248)) {
                                if (enable_depth_ && is_valid_depth_mode_) {
                                    auto optRange = rsStereoSensor_.get_option_range(RS2_OPTION_GAIN);
                                    if ((float)depth_controls_.gain < optRange.min)
                                        depth_controls_.gain = (int)optRange.min;
                                    else if ((float)depth_controls_.gain > optRange.max)
                                        depth_controls_.gain = (int)optRange.max;
                                    rsStereoSensor_.set_option(RS2_OPTION_GAIN, (float)depth_controls_.gain);
                                }
                            }
                        }
                        ImGui::SetNextItemWidth(120);
                        if (ImGui::DragInt(CreateControlString("Laser Power", GetInstanceName()).c_str(), &laser_power_, 1.0f, 0, 360)) {
                            if (enable_depth_ && is_valid_depth_mode_) {
                                auto optRange = rsStereoSensor_.get_option_range(RS2_OPTION_LASER_POWER);
                                if ((float)laser_power_ < optRange.min)
                                    laser_power_ = (int)optRange.min;
                                else if ((float)laser_power_ > optRange.max)
                                    laser_power_ = (int)optRange.max;
                                rsStereoSensor_.set_option(RS2_OPTION_LASER_POWER, (float)laser_power_);
                            }
                        }
                        ImGui::TreePop();
                    }
                }
                ImGui::TreePop();
            }
            if (enable_depth_ && is_valid_depth_mode_) {
                ImGui::Separator();
                if (ImGui::TreeNode("Post-Processing")) {
                    if (ImGui::TreeNode("Decimation Filter")) {
                        ImGui::Checkbox(CreateControlString("Enable Decimation", GetInstanceName()).c_str(), &enable_decimation_filter_);
                        if (enable_decimation_filter_) {
                            ImGui::SetNextItemWidth(100);
                            if (ImGui::DragInt(CreateControlString("Amount", GetInstanceName()).c_str(), &decimation_amount_, 0.1f, 1, 8)) {
                                rs_dec_filter_.set_option(RS2_OPTION_FILTER_MAGNITUDE, (float)decimation_amount_);
                            }
                        }
                        ImGui::TreePop();
                    }
                    ImGui::Separator();
                    if (ImGui::TreeNode("Threshold Filter")) {
                        ImGui::Checkbox(CreateControlString("Enable Threshold", GetInstanceName()).c_str(), &enable_threshold_filter_);
                        if (enable_threshold_filter_) {
                            ImGui::SetNextItemWidth(100);
                            if(ImGui::DragInt(CreateControlString("Thresh Min", GetInstanceName()).c_str(), &thresh_min_, 0.1f, 0, 16)) {
                                rs_thr_filter_.set_option(RS2_OPTION_MIN_DISTANCE, (float)thresh_min_);
                            }
                            ImGui::SetNextItemWidth(100);
                            if(ImGui::DragInt(CreateControlString("Thresh Max", GetInstanceName()).c_str(), &thresh_max_, 0.1f, 0, 16)) {
                                rs_thr_filter_.set_option(RS2_OPTION_MAX_DISTANCE, (float)thresh_max_);
                            }
                        }
                        ImGui::TreePop();
                    }
                    ImGui::Separator();
                    if (ImGui::TreeNode("Spatial Filter")) {
                        ImGui::Checkbox(CreateControlString("Enable Spatial", GetInstanceName()).c_str(), &enable_spatial_filter_);
                        if (enable_spatial_filter_) {
                            ImGui::SetNextItemWidth(100);
                            if(ImGui::DragInt(CreateControlString("Spatial Amount", GetInstanceName()).c_str(), &spatial_mag_, 0.1f, 1, 5)) {
                                rs_spat_filter_.set_option(RS2_OPTION_FILTER_MAGNITUDE, (float)spatial_mag_);
                            }
                            ImGui::SetNextItemWidth(100);
                            if(ImGui::DragFloat(CreateControlString("Spatial Alpha", GetInstanceName()).c_str(), &spatial_alpha_, 0.01f, 0.25f, 1.0f)) {
                                rs_spat_filter_.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, spatial_alpha_);
                            }
                            ImGui::SetNextItemWidth(100);
                            if(ImGui::DragInt(CreateControlString("Spatial Delta", GetInstanceName()).c_str(), &spatial_delta_, 0.1f, 1, 50)) {
                                rs_spat_filter_.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, (float)spatial_delta_);
                            }
                            ImGui::SetNextItemWidth(200);
                            if (ImGui::Combo(CreateControlString("Hole Filling", GetInstanceName()).c_str(), &spatial_hole_fill_,
                                "Disabled\0Radius 2\0Radius 4\0Radius 8\0Radius 16\0Unlimited\0\0")) {
                                rs_spat_filter_.set_option(RS2_OPTION_HOLES_FILL, (float)spatial_hole_fill_);
                            }
                        }
                        ImGui::TreePop();
                    }
                    ImGui::Separator();
                    if (ImGui::TreeNode("Temporal Filter")) {
                        ImGui::Checkbox(CreateControlString("Enable Temporal", GetInstanceName()).c_str(), &enable_temporal_filter_);
                        if (enable_temporal_filter_) {
                            ImGui::SetNextItemWidth(100);
                            if(ImGui::DragFloat(CreateControlString("Temp Alpha", GetInstanceName()).c_str(), &temporal_alpha_, 0.01f, 0.0f, 1.0f)) {
                                rs_temp_filter_.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, temporal_alpha_);
                            }
                            ImGui::SetNextItemWidth(100);
                            if(ImGui::DragInt(CreateControlString("Temp Delta", GetInstanceName()).c_str(), &temporal_delta_, 0.1f, 1, 100)) {
                                rs_temp_filter_.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, (float)temporal_delta_);
                            }
                            ImGui::SetNextItemWidth(200);
                            if (ImGui::Combo(CreateControlString("Persistence", GetInstanceName()).c_str(), &temporal_persistency_,
                            "Disabled\0Valid in 8/8\0Valid in 2/last 3\0Valid in 2/last 4\0Valid in 2/8\0Valid in 1/last 2\0Valid in 1/last 5\0Valid in 1/8\0Always On\0\0")) {
                                rs_temp_filter_.set_option(RS2_OPTION_HOLES_FILL, (float)temporal_persistency_);
                            }
                        }
                        ImGui::TreePop();
                    }
                    ImGui::Separator();
                    if (ImGui::TreeNode("Hole Filling Filter")) {
                        ImGui::Checkbox(CreateControlString("Enable Hole Fill", GetInstanceName()).c_str(), &enable_hole_fill_filter_);
                        if (enable_hole_fill_filter_) {
                            ImGui::SetNextItemWidth(200);
                            if (ImGui::Combo(CreateControlString("Fill Mode", GetInstanceName()).c_str(), &hole_fill_option_,
                                "Fill From Left\0Farest From Around\0Nearest From Around\0\0")) {
                                rs_hole_fill_filter_.set_option(RS2_OPTION_HOLES_FILL, (float)hole_fill_option_);
                            }
                        }
                        ImGui::TreePop();
                    }
                    ImGui::TreePop();
                }
            }
        }
    }

}

std::string RealsenseCamera::GetState()
{
    using namespace nlohmann;

    json state;

    state["rs_serial"] = rs_cam_serial_;
    state["color_enabled"] = enable_color_;
    state["color_res_idx"] = color_mode_idx_;
    state["color_fps_idx"] = color_fps_idx_;
    state["color_auto_exposure"] = color_controls_.auto_exposure;
    state["color_exposure"] = color_controls_.exposure;
    state["color_gain"] = color_controls_.gain;
    state["depth_enabled"] = enable_depth_;
    state["depth_res_idx"] = depth_mode_idx_;
    state["depth_fps_idx"] = depth_fps_idx_;
    state["depth_auto_exposure"] = depth_controls_.auto_exposure;
    state["depth_exposure"] = depth_controls_.exposure;
    state["depth_gain"] = depth_controls_.gain;
    state["laser_power"] = laser_power_;
    state["align_color_to_depth"] = align_color_to_depth_;
    state["align_depth_to_color"] = align_depth_to_color_;
    state["depth_preset"] = preset_idx_;
    state["enable_dec_filter"] = enable_decimation_filter_;
    if (enable_decimation_filter_) {
        state["dec_magnitude"] = decimation_amount_;
    }
    state["enable_threshold_filter"] = enable_threshold_filter_;
    if (enable_threshold_filter_) {
        state["thresh_min"] = thresh_min_;
        state["thresh_max"] = thresh_max_;
    }
    state["enable_spatial_filter"] = enable_spatial_filter_;
    if (enable_spatial_filter_) {
        state["spatial_magnitude"] = spatial_mag_;
        state["spatial_alpha"] = spatial_alpha_;
        state["spatial_delta"] = spatial_delta_;
        state["spatial_hole_fill"] = spatial_hole_fill_;
    }
    state["enable_temporal_filter"] = enable_temporal_filter_;
    if (enable_temporal_filter_) {
        state["temporal_alpha"] = temporal_alpha_;
        state["temporal_delta"] = temporal_delta_;
        state["temporal_hole_fill"] = temporal_persistency_;
    }
    state["enable_hole_fill_filter"] = enable_hole_fill_filter_;
    if (enable_hole_fill_filter_) {
        state["hole_fill_option"] = hole_fill_option_;
    }

    std::string stateSerialized = state.dump(4);

    return stateSerialized;
}

void RealsenseCamera::SetState(std::string &&json_serialized)
{
    using namespace nlohmann;

    json state = json::parse(json_serialized);

    bool tmpColorEnabled, tmpDepthEnabled;

    if (state.contains("color_res_idx"))
        color_mode_idx_ = state["color_res_idx"].get<int>();
    if (state.contains("color_fps_idx"))
        color_fps_idx_ = state["color_fps_idx"].get<int>();
    if (state.contains("color_enabled"))
        tmpColorEnabled = state["color_enabled"].get<bool>();
    if (state.contains("depth_res_idx"))
        depth_mode_idx_ = state["depth_res_idx"].get<int>();
    if (state.contains("depth_fps_idx"))
        depth_fps_idx_ = state["depth_fps_idx"].get<int>();
    if (state.contains("depth_enabled"))
        tmpDepthEnabled = state["depth_enabled"].get<bool>();
    if (state.contains("color_auto_exposure"))
        color_controls_.auto_exposure = state["color_auto_exposure"].get<bool>();
    if (state.contains("color_exposure"))
        color_controls_.exposure = state["color_exposure"].get<int>();
    if (state.contains("color_gain"))
        color_controls_.gain = state["color_gain"].get<int>();
    if (state.contains("depth_auto_exposure"))
        depth_controls_.auto_exposure = state["depth_auto_exposure"].get<bool>();
    if (state.contains("depth_exposure"))
        depth_controls_.exposure = state["depth_exposure"].get<int>();
    if (state.contains("depth_gain"))
        depth_controls_.gain = state["depth_gain"].get<int>();
    if (state.contains("laser_power"))
        laser_power_ = state["laser_power"].get<int>();
    if (state.contains("depth_preset"))
        preset_idx_ = state["depth_preset"].get<int>();
    if (state.contains("enable_dec_filter"))
        enable_decimation_filter_ = state["enable_dec_filter"].get<bool>();
    if (state.contains("dec_magnitude"))
        decimation_amount_ = state["dec_magnitude"].get<int>();
    if (state.contains("enable_threshold_filter"))
        enable_threshold_filter_ = state["enable_threshold_filter"].get<bool>();
    if (state.contains("thresh_min"))
        thresh_min_ = state["thresh_min"].get<int>();
    if (state.contains("thresh_max"))
        thresh_max_ = state["thresh_max"].get<int>();
    if (state.contains("enable_spatial_filter"))
        enable_spatial_filter_ = state["enable_spatial_filter"].get<bool>();
    if (state.contains("spatial_magnitude"))
        spatial_mag_ = state["spatial_magnitude"].get<int>();
    if (state.contains("spatial_alpha"))
        spatial_alpha_ = state["spatial_alpha"].get<float>();
    if (state.contains("spatial_delta"))
        spatial_delta_ = state["spatial_delta"].get<int>();
    if (state.contains("spatial_hole_fill"))
        spatial_hole_fill_ = state["spatial_hole_fill"].get<int>();
    if (state.contains("enable_temporal_filter"))
        enable_temporal_filter_ = state["enable_temporal_filter"].get<bool>();
    if (state.contains("temporal_alpha"))
        temporal_alpha_ = state["temporal_alpha"].get<float>();
    if (state.contains("temporal_delta"))
        temporal_delta_ = state["temporal_delta"].get<int>();
    if (state.contains("temporal_hole_fill"))
        temporal_persistency_ = state["temporal_hole_fill"].get<int>();
    if (state.contains("enable_hole_fill_filter"))
        enable_hole_fill_filter_ = state["enable_hole_fill_filter"].get<bool>();
    if (state.contains("hole_fill_option"))
        hole_fill_option_ = state["hole_fill_option"].get<int>();
    if (state.contains("align_color_to_depth"))
        align_color_to_depth_ = state["align_color_to_depth"].get<bool>();
    if (state.contains("align_depth_to_color"))
        align_depth_to_color_ = state["align_depth_to_color"].get<bool>();

    SetFilterOptions();

    if (state.contains("rs_serial")) {
        rs_cam_serial_ = state["rs_serial"].get<std::string>();
        if (!rs_cam_serial_.empty()) {
            first_list_refresh_ = false;
            RefreshDeviceList_();
            for (int i = 0; i < devices_.size(); i++) {
                auto dev = devices_[i];
                std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                if (serial == rs_cam_serial_) {
                    rs_cam_idx_ = i + 1;
                    enable_color_ = tmpColorEnabled;
                    enable_depth_ = tmpDepthEnabled;
                    should_set_params_ = true;
                    init_once_ = true;
                    break;
                }
            }
        }
    }

}
