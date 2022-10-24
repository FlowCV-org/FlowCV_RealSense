//
// FlowCV Plugin - Realsense Camera
// Written By Richard Wardlow
//

#include "realsense_plugin.hpp"
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
    SetComponentVersion_("0.2.0");
    SetInstanceCount(global_inst_counter);
    global_inst_counter++;

    // 0 inputs
    SetInputCount_( 0 );

    // 3 outputs
    SetOutputCount_( 3, {"color", "depth", "metadata"}, {IoType::Io_Type_CvMat, IoType::Io_Type_CvMat, IoType::Io_Type_JSON} );

    // Skip initial instance which is for plugin adding/checking
    if (global_inst_counter >= 2) {
        camera_ = std::make_unique<realsense_camera>();
    }

    selected_camera_idx_ = 0;
    color_cfg_idx_ = 0;
    color_fps_idx_ = 0;
    depth_cfg_idx_ = 0;
    depth_fps_idx_ = 0;
    color_fps_ = 30;
    depth_fps_ = 30;
    enable_color_ = false;
    enable_depth_ = false;
    alignment_mode_ = 0;

    SetEnabled(true);

}

void RealsenseCamera::Process_( SignalBus const& inputs, SignalBus& outputs )
{

    std::lock_guard<std::mutex> lck(io_mutex_);
    camera_->ProcessStreams();
    if (!camera_->IsReconfiguring()) {
        if (!camera_->GetFrame(RS2_STREAM_COLOR).empty()) {
            outputs.SetValue(0, camera_->GetFrame(RS2_STREAM_COLOR));
        }
        if (!camera_->GetFrame(RS2_STREAM_DEPTH).empty()) {
            outputs.SetValue(1, camera_->GetFrame(RS2_STREAM_DEPTH));
        }
        if (!camera_->GetMetaData().empty())
            outputs.SetValue(2, camera_->GetMetaData());
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

    if (interface == (int)FlowCV::GuiInterfaceType_Controls) {
        if (ImGui::Button(CreateControlString("Refresh RS Cam List", GetInstanceName()).c_str())) {
            camera_->RefreshDeviceList();
        }
        ImGui::Separator();
        auto cam_list = camera_->GetDeviceList();
        if (ImGui::Combo(CreateControlString("RS Cameras", GetInstanceName()).c_str(), &selected_camera_idx_, [](void* data, int idx, const char** out_text) {
            *out_text = ((const std::vector<std::string>*)data)->at(idx).c_str();
            return true;
        }, (void*)&cam_list, (int)cam_list.size())) {
            enable_color_ = false;
            enable_depth_ = false;
            camera_->InitCamera(selected_camera_idx_);
        }
        if (selected_camera_idx_ > 0) {
            //
            // Common Controls
            //
            if (enable_color_ && enable_depth_) {
                ImGui::SetNextItemWidth(150);
                if (ImGui::Combo(CreateControlString("Stream Alignment", GetInstanceName()).c_str(), &alignment_mode_, "Alignment Off\0Align Depth to Color\0Align Color to Depth\0\0")) {
                    camera_->SetStreamAlignment((StreamAlignment) alignment_mode_);
                }
            }
            //
            // Color Section
            //
            auto color_cfg_list = camera_->GetStreamConfigList(rs2_stream::RS2_STREAM_COLOR);
            if (ImGui::Checkbox(CreateControlString("Enable Color Sensor", GetInstanceName()).c_str(), &enable_color_)) {
                color_cfg_list->at(color_cfg_idx_).fps_idx = color_fps_idx_;
                if (enable_color_)
                    camera_->EnableStream(color_cfg_list->at(color_cfg_idx_));
                else
                    camera_->DisableStream(RS2_STREAM_COLOR);
            }
            ImGui::SetNextItemWidth(100);
            if (ImGui::Combo(CreateControlString("Color Resolution", GetInstanceName()).c_str(), &color_cfg_idx_, [](void* data, int idx, const char** out_text) {
                *out_text = ((const std::vector<StreamConfig>*)data)->at(idx).str_resolution.c_str();
                return true;
            }, (void*)color_cfg_list, (int)color_cfg_list->size())) {
                for (int i = 0; i < color_cfg_list->at(color_cfg_idx_).fps_list.size(); i++) {
                    if (color_cfg_list->at(color_cfg_idx_).fps_list.at(i) == color_fps_) {
                        color_fps_idx_ = i;
                        break;
                    }
                }
                color_cfg_list->at(color_cfg_idx_).fps_idx = color_fps_idx_;
                if (enable_color_) {
                    camera_->EnableStream(color_cfg_list->at(color_cfg_idx_));
                }
            }
            ImGui::SetNextItemWidth(100);
            std::vector<std::string> cFpsList;
            for (int i = 0; i < color_cfg_list->at(color_cfg_idx_).fps_list.size(); i++) {
                if (color_cfg_list->at(color_cfg_idx_).fps_list.at(i) == color_fps_)
                    color_fps_idx_ = i;
                std::string fps = std::to_string(color_cfg_list->at(color_cfg_idx_).fps_list.at(i));
                cFpsList.emplace_back(fps);
            }
            ImGui::SetNextItemWidth(100);
            if (ImGui::Combo(CreateControlString("Color FPS", GetInstanceName()).c_str(), &color_fps_idx_, [](void* data, int idx, const char** out_text) {
                *out_text = ((const std::vector<std::string>*)data)->at(idx).c_str();
                return true;
            }, (void*)&cFpsList, (int)cFpsList.size())) {
                color_cfg_list->at(color_cfg_idx_).fps_idx = color_fps_idx_;
                color_fps_ = color_cfg_list->at(color_cfg_idx_).fps_list.at(color_fps_idx_);
                if (enable_color_) {
                    camera_->EnableStream(color_cfg_list->at(color_cfg_idx_));
                }
            }
            if (enable_color_) {
                if (ImGui::TreeNode("Color Controls")) {
                    auto color_props = camera_->GetPropertyList(RS2_STREAM_COLOR);
                    if (ImGui::Button(CreateControlString("Restore Color Defaults", GetInstanceName()).c_str())) {
                        camera_->ResetProperties(RS2_STREAM_COLOR);
                    }
                    ImGui::Separator();
                    for (int i = 0; i < color_props->size(); i++) {
                        if (color_props->at(i).range.min == 0 && color_props->at(i).range.max == 1) { // Checkbox Control
                            bool tmpVal = (bool)color_props->at(i).value;
                            if (ImGui::Checkbox(CreateControlString(color_props->at(i).str_prop.c_str(), GetInstanceName()).c_str(), &tmpVal)) {
                                color_props->at(i).value = (int)tmpVal;
                                camera_->SetProperty(RS2_STREAM_COLOR, color_props->at(i).prop);
                                if (color_props->at(i).prop == RS2_OPTION_ENABLE_AUTO_EXPOSURE) {
                                    for (auto & prop : *color_props) { // Set dependency prop value
                                        if (prop.prop == RS2_OPTION_EXPOSURE)
                                            prop.value = (int)prop.range.def;
                                    }
                                }
                            }
                        }
                        else if (color_props->at(i).is_list) { // Combo List
                            ImGui::SetNextItemWidth(100);
                            if (ImGui::Combo(CreateControlString(color_props->at(i).str_prop.c_str(), GetInstanceName()).c_str(),
                                             &color_props->at(i).value, [](void* data, int idx, const char** out_text) {
                                *out_text = ((const std::vector<std::string>*)data)->at(idx).c_str();
                                return true;
                            }, (void*)&color_props->at(i).opt_list, (int)color_props->at(i).opt_list.size())) {
                                camera_->SetProperty(RS2_STREAM_COLOR, color_props->at(i).prop);
                            }
                        }
                        else { // Int Drag Control
                            ImGui::SetNextItemWidth(100);
                            if (ImGui::DragInt(CreateControlString(color_props->at(i).str_prop.c_str(), GetInstanceName()).c_str(),
                                               &color_props->at(i).value, color_props->at(i).range.step, (int)color_props->at(i).range.min, (int)color_props->at(i).range.max)) {
                                if (color_props->at(i).value < (int)color_props->at(i).range.min)
                                    color_props->at(i).value = (int)color_props->at(i).range.min;
                                else if (color_props->at(i).value > (int)color_props->at(i).range.max)
                                    color_props->at(i).value = (int)color_props->at(i).range.max;
                                camera_->SetProperty(RS2_STREAM_COLOR, color_props->at(i).prop);
                            }
                        }
                    }
                    ImGui::TreePop();
                }
            }
            ImGui::Separator();
            //
            // Depth Section
            //
            auto depth_cfg_list = camera_->GetStreamConfigList(rs2_stream::RS2_STREAM_DEPTH);
            if (ImGui::Checkbox(CreateControlString("Enable Depth Sensor", GetInstanceName()).c_str(), &enable_depth_)) {
                depth_cfg_list->at(depth_cfg_idx_).fps_idx = depth_fps_idx_;
                if (enable_depth_)
                    camera_->EnableStream(depth_cfg_list->at(depth_cfg_idx_));
                else
                    camera_->DisableStream(RS2_STREAM_DEPTH);
            }
            ImGui::SetNextItemWidth(100);
            if (ImGui::Combo(CreateControlString("Depth Resolution", GetInstanceName()).c_str(), &depth_cfg_idx_, [](void* data, int idx, const char** out_text) {
                *out_text = ((const std::vector<StreamConfig>*)data)->at(idx).str_resolution.c_str();
                return true;
            }, (void*)depth_cfg_list, (int)depth_cfg_list->size())) {
                for (int i = 0; i < depth_cfg_list->at(depth_cfg_idx_).fps_list.size(); i++) {
                    if (depth_cfg_list->at(depth_cfg_idx_).fps_list.at(i) == depth_fps_) {
                        depth_fps_idx_ = i;
                        break;
                    }
                }
                depth_cfg_list->at(depth_cfg_idx_).fps_idx = depth_fps_idx_;
                if (enable_depth_) {
                    camera_->EnableStream(depth_cfg_list->at(depth_cfg_idx_));
                }
            }
            ImGui::SetNextItemWidth(100);
            std::vector<std::string> dFpsList;
            for (int i = 0; i < depth_cfg_list->at(depth_cfg_idx_).fps_list.size(); i++) {
                if (depth_cfg_list->at(depth_cfg_idx_).fps_list.at(i) == depth_fps_)
                    depth_fps_idx_ = i;
                std::string fps = std::to_string(depth_cfg_list->at(depth_cfg_idx_).fps_list.at(i));
                dFpsList.emplace_back(fps);
            }
            ImGui::SetNextItemWidth(100);
            if (ImGui::Combo(CreateControlString("Depth FPS", GetInstanceName()).c_str(), &depth_fps_idx_, [](void* data, int idx, const char** out_text) {
                *out_text = ((const std::vector<std::string>*)data)->at(idx).c_str();
                return true;
            }, (void*)&dFpsList, (int)dFpsList.size())) {
                depth_cfg_list->at(depth_cfg_idx_).fps_idx = depth_fps_idx_;
                depth_fps_ = depth_cfg_list->at(depth_cfg_idx_).fps_list.at(depth_fps_idx_);
                if (enable_depth_) {
                    camera_->EnableStream(depth_cfg_list->at(depth_cfg_idx_));
                }
            }
            if (enable_depth_) {
                if (ImGui::TreeNode("Depth Controls")) {
                    auto depth_props = camera_->GetPropertyList(RS2_STREAM_DEPTH);
                    if (ImGui::Button(CreateControlString("Restore Depth Defaults", GetInstanceName()).c_str())) {
                        camera_->ResetProperties(RS2_STREAM_DEPTH);
                    }
                    ImGui::Separator();
                    for (int i = 0; i < depth_props->size(); i++) {
                        if (depth_props->at(i).range.min == 0 && depth_props->at(i).range.max == 1) { // Checkbox Control
                            bool tmpVal = (bool)depth_props->at(i).value;
                            if (ImGui::Checkbox(CreateControlString(depth_props->at(i).str_prop.c_str(), GetInstanceName()).c_str(), &tmpVal)) {
                                depth_props->at(i).value = (int) tmpVal;
                                camera_->SetProperty(RS2_STREAM_DEPTH, depth_props->at(i).prop);
                            }
                        }
                        else if (depth_props->at(i).is_list) { // Combo List
                            ImGui::SetNextItemWidth(100);
                            if (ImGui::Combo(CreateControlString(depth_props->at(i).str_prop.c_str(), GetInstanceName()).c_str(),
                                             &depth_props->at(i).value, [](void* data, int idx, const char** out_text) {
                                    *out_text = ((const std::vector<std::string>*)data)->at(idx).c_str();
                                    return true;
                                }, (void*)&depth_props->at(i).opt_list, (int)depth_props->at(i).opt_list.size())) {
                                camera_->SetProperty(RS2_STREAM_DEPTH, depth_props->at(i).prop);
                            }
                        }
                        else { // Int Drag Control
                            ImGui::SetNextItemWidth(100);
                            if (ImGui::DragInt(CreateControlString(depth_props->at(i).str_prop.c_str(), GetInstanceName()).c_str(),
                                               &depth_props->at(i).value, depth_props->at(i).range.step, (int)depth_props->at(i).range.min, (int)depth_props->at(i).range.max)) {
                                if (depth_props->at(i).value < (int)depth_props->at(i).range.min)
                                    depth_props->at(i).value = (int)depth_props->at(i).range.min;
                                else if (depth_props->at(i).value > (int)depth_props->at(i).range.max)
                                    depth_props->at(i).value = (int)depth_props->at(i).range.max;
                                camera_->SetProperty(RS2_STREAM_DEPTH, depth_props->at(i).prop);
                            }
                        }
                    }
                    ImGui::TreePop();
                }
                if (ImGui::TreeNode("Post-Processing")) {
                    auto filter_list = camera_->GetFilterList(RS2_STREAM_DEPTH);
                    for (auto &filt : *filter_list) {
                        if (ImGui::TreeNode(filt.filter_name.c_str())) {
                            std::string filtEnable = "Enable ";
                            filtEnable += filt.filter_name;
                            ImGui::Checkbox(CreateControlString(filtEnable.c_str(), GetInstanceName()).c_str(), &filt.is_enabled);
                            if (filt.is_enabled) {
                                if (!filt.supported_options.empty()) {
                                    for (auto &option_pair : filt.supported_options) {
                                        FilterOptions &filtOpt = option_pair.second;
                                        ImGui::SetNextItemWidth(100);
                                        if (filtOpt.is_list) {
                                            ImGui::SetNextItemWidth(100);
                                            int tmpVal = (int)filtOpt.value;
                                            if (ImGui::Combo(CreateControlString(filtOpt.name.c_str(), GetInstanceName()).c_str(),
                                                             &tmpVal, [](void* data, int idx, const char** out_text) {
                                                    *out_text = ((const std::vector<std::string>*)data)->at(idx).c_str();
                                                    return true;
                                                }, (void*)&filtOpt.opt_list, (int)filtOpt.opt_list.size())) {
                                                filtOpt.value = (float)tmpVal;
                                                filt.filter.set_option(option_pair.first, filtOpt.value);
                                            }
                                        }
                                        else if (filtOpt.is_int) {
                                            int tmpVal = (int)filtOpt.value;
                                            if (ImGui::DragInt(CreateControlString(filtOpt.name.c_str(), GetInstanceName()).c_str(),
                                                                 &tmpVal, filtOpt.range.step, (int)filtOpt.range.min, (int)filtOpt.range.max)) {
                                                filtOpt.value = (float)tmpVal;
                                                if (filtOpt.value < filtOpt.range.min)
                                                    filtOpt.value = filtOpt.range.min;
                                                else if (filtOpt.value > filtOpt.range.max)
                                                    filtOpt.value = filtOpt.range.max;
                                                filt.filter.set_option(option_pair.first, filtOpt.value);
                                            }
                                        }
                                        else {
                                            if (ImGui::DragFloat(CreateControlString(filtOpt.name.c_str(), GetInstanceName()).c_str(),
                                                             &filtOpt.value, filtOpt.range.step, filtOpt.range.min, filtOpt.range.max)) {
                                                if (filtOpt.value < filtOpt.range.min)
                                                    filtOpt.value = filtOpt.range.min;
                                                else if (filtOpt.value > filtOpt.range.max)
                                                    filtOpt.value = filtOpt.range.max;
                                                filt.filter.set_option(option_pair.first, filtOpt.value);
                                            }
                                        }
                                    }
                                }
                            }
                            ImGui::TreePop();
                        }
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

    if (selected_camera_idx_ > 0) {
        state["cam_idx"] = selected_camera_idx_;
        state["rs_serial"] = camera_->GetDeviceSerial(selected_camera_idx_);
        state["color_enabled"] = enable_color_;
        if (enable_color_ && enable_depth_)
            state["alignment"] = alignment_mode_;
        if (enable_color_) {
            auto color_cfg_list = camera_->GetStreamConfigList(rs2_stream::RS2_STREAM_COLOR);
            auto color_props = camera_->GetPropertyList(RS2_STREAM_COLOR);
            state["color_fps_idx"] = color_cfg_list->at(color_cfg_idx_).fps_idx;
            state["color_fps"] = color_cfg_list->at(color_cfg_idx_).fps_list.at(color_cfg_list->at(color_cfg_idx_).fps_idx);
            state["color_res_idx"] = color_cfg_idx_;
            nlohmann::json color_controls;
            for(const auto &prop : *color_props) {
                color_controls[prop.str_prop] = prop.value;
            }
            state["color_controls"] = color_controls;
        }
        state["depth_enabled"] = enable_depth_;
        if (enable_depth_) {
            auto depth_cfg_list = camera_->GetStreamConfigList(rs2_stream::RS2_STREAM_DEPTH);
            auto depth_props = camera_->GetPropertyList(RS2_STREAM_DEPTH);
            state["depth_fps_idx"] = depth_cfg_list->at(depth_cfg_idx_).fps_idx;
            state["depth_fps"] = depth_cfg_list->at(color_cfg_idx_).fps_list.at(depth_cfg_list->at(depth_cfg_idx_).fps_idx);
            state["depth_res_idx"] = depth_cfg_idx_;
            nlohmann::json depth_controls;
            for(const auto &prop : *depth_props) {
                depth_controls[prop.str_prop] = prop.value;
            }
            state["depth_controls"] = depth_controls;
            nlohmann::json depth_filtering;
            auto depth_filt = camera_->GetFilterList(RS2_STREAM_DEPTH);
            for (auto &filt : *depth_filt) {
                if (filt.is_enabled) {
                    depth_filtering[filt.filter_name] = filt.is_enabled;
                    for (auto &option_pair: filt.supported_options) {
                        FilterOptions &filtOpt = option_pair.second;
                        depth_filtering[filtOpt.name] = filtOpt.value;
                    }
                }
            }
            state["depth_filtering"] = depth_filtering;
        }
    }
    std::string stateSerialized = state.dump(4);

    return stateSerialized;
}

void RealsenseCamera::SetState(std::string &&json_serialized)
{
    using namespace nlohmann;

    json state = json::parse(json_serialized);

    if (state.contains("cam_idx"))
        selected_camera_idx_ = state["cam_idx"].get<int>();
    if (selected_camera_idx_ > 0) {
        camera_->RefreshDeviceList();
        bool cam_exists = false;
        std::string saved_serial;
        if (state.contains("rs_serial"))
            saved_serial = state["rs_serial"].get<std::string>();

        if (!saved_serial.empty()) {
            std::string idx_serial = camera_->GetDeviceSerial(selected_camera_idx_);
            if (idx_serial != saved_serial) {
                // Attempt to find matching serial device
                for (int i = 0; i < camera_->GetDeviceCount(); i++) {
                    std::string check = camera_->GetDeviceSerial(i);
                    if (check == saved_serial) {
                        selected_camera_idx_ = i;
                        cam_exists = true;
                        break;
                    }
                }
            } else {
                cam_exists = true;
            }
            if (cam_exists) {
                std::lock_guard<std::mutex> lck(io_mutex_);
                camera_->InitCamera(selected_camera_idx_);
                if (state.contains("alignment"))
                    alignment_mode_ = state["alignment"].get<int>();
                else
                    alignment_mode_ = 0;
                if (state.contains("color_enabled"))
                    enable_color_ = state["color_enabled"].get<bool>();
                if (enable_color_) {
                    auto color_cfg_list = camera_->GetStreamConfigList(rs2_stream::RS2_STREAM_COLOR);
                    auto color_props = camera_->GetPropertyList(RS2_STREAM_COLOR);
                    if (state.contains("color_res_idx"))
                        color_cfg_idx_ = state["color_res_idx"].get<int>();
                    if (state.contains("color_fps_idx"))
                        color_fps_idx_ = state["color_fps_idx"].get<int>();
                    if (state.contains("color_fps"))
                        color_fps_ = state["color_fps"].get<int>();
                    color_cfg_list->at(color_cfg_idx_).fps_idx = color_fps_idx_;
                    camera_->EnableStream(color_cfg_list->at(color_cfg_idx_));
                    if (state.contains("color_controls")) {
                        for (auto &prop : *color_props) {
                            if (state["color_controls"].contains(prop.str_prop)) {
                                prop.value = state["color_controls"][prop.str_prop].get<int>();
                                camera_->SetProperty(RS2_STREAM_COLOR, prop.prop, true);
                            }
                        }
                        // Fix for Auto Enable Options
                        for (auto &prop : *color_props) {
                            if (prop.prop == RS2_OPTION_ENABLE_AUTO_EXPOSURE) {
                                if (state["color_controls"].contains(prop.str_prop)) {
                                    prop.value = state["color_controls"][prop.str_prop].get<int>();
                                    if (prop.value)
                                        camera_->SetProperty(RS2_STREAM_COLOR, prop.prop, true);
                                }
                            }
                            if (prop.prop == RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE) {
                                if (state["color_controls"].contains(prop.str_prop)) {
                                    prop.value = state["color_controls"][prop.str_prop].get<int>();
                                    if (prop.value)
                                        camera_->SetProperty(RS2_STREAM_COLOR, prop.prop, true);
                                }
                            }
                        }
                    }
                }
                if (state.contains("depth_enabled"))
                    enable_depth_ = state["depth_enabled"].get<bool>();
                if (enable_depth_) {
                    auto depth_cfg_list = camera_->GetStreamConfigList(rs2_stream::RS2_STREAM_DEPTH);
                    auto depth_props = camera_->GetPropertyList(RS2_STREAM_DEPTH);
                    if (state.contains("depth_res_idx"))
                        depth_cfg_idx_ = state["depth_res_idx"].get<int>();
                    if (state.contains("depth_fps_idx"))
                        depth_fps_idx_ = state["depth_fps_idx"].get<int>();
                    if (state.contains("depth_fps"))
                        depth_fps_ = state["depth_fps"].get<int>();
                    depth_cfg_list->at(depth_cfg_idx_).fps_idx = depth_fps_idx_;
                    camera_->EnableStream(depth_cfg_list->at(depth_cfg_idx_));
                    if (state.contains("depth_controls")) {
                        for (auto &prop : *depth_props) {
                            if (state["depth_controls"].contains(prop.str_prop)) {
                                prop.value = state["depth_controls"][prop.str_prop].get<int>();
                                camera_->SetProperty(RS2_STREAM_DEPTH, prop.prop, true);
                            }
                        }
                        // Fix for Auto Enable Options
                        for (auto &prop : *depth_props) {
                            if (prop.prop == RS2_OPTION_ENABLE_AUTO_EXPOSURE) {
                                if (state["depth_controls"].contains(prop.str_prop)) {
                                    prop.value = state["depth_controls"][prop.str_prop].get<int>();
                                    if (prop.value)
                                        camera_->SetProperty(RS2_STREAM_DEPTH, prop.prop, true);
                                }
                            }
                            if (prop.prop == RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE) {
                                if (state["depth_controls"].contains(prop.str_prop)) {
                                    prop.value = state["depth_controls"][prop.str_prop].get<int>();
                                    if (prop.value)
                                        camera_->SetProperty(RS2_STREAM_DEPTH, prop.prop, true);
                                }
                            }
                        }
                    }
                    if (state.contains("depth_filtering")) {
                        auto depth_filt = camera_->GetFilterList(RS2_STREAM_DEPTH);
                        for (auto &filt : *depth_filt) {
                            if (state["depth_filtering"].contains(filt.filter_name)) {
                                filt.is_enabled = state["depth_filtering"][filt.filter_name].get<bool>();
                                if (filt.is_enabled) {
                                    for (auto &option_pair: filt.supported_options) {
                                        FilterOptions &filtOpt = option_pair.second;
                                        if (state["depth_filtering"].contains(filtOpt.name)) {
                                            filtOpt.value = state["depth_filtering"][filtOpt.name].get<float>();
                                            filt.filter.set_option(option_pair.first, filtOpt.value);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    camera_->SetStreamAlignment((StreamAlignment)alignment_mode_);
                }
            }
        }
        if (!cam_exists)
            selected_camera_idx_ = 0;
    }
}
