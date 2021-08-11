// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/*
 * Vehicle class implementation.
 */

#include <ugcs/vsm/vehicle.h>
#include <ugcs/vsm/properties.h>
#include <ugcs/vsm/log.h>
#include <ugcs/vsm/actions.h>

#include <iostream>

using namespace ugcs::vsm;

std::hash<Vehicle*> Vehicle::Hasher::hasher;

Vehicle::Vehicle(
    proto::Device_type type,
    Request_processor::Ptr proc,
    Request_completion_context::Ptr comp):
    Device(type, proc, comp)
{
    Property::Ptr prop;

// Create flight controller
    flight_controller = Add_subsystem(proto::SUBSYSTEM_TYPE_FLIGHT_CONTROLLER);

    // Create telemetry
#define ADD_T(x) t_##x = flight_controller->Add_telemetry(#x);
#define ADD_TS(x, y) t_##x = flight_controller->Add_telemetry(#x, y);

    ADD_TS(altitude_origin, proto::FIELD_SEMANTIC_ALTITUDE_AMSL);
    ADD_TS(is_armed, proto::FIELD_SEMANTIC_BOOL);
    ADD_T(vertical_speed);
    ADD_T(control_mode);
    ADD_T(main_current);
    ADD_TS(downlink_present, proto::FIELD_SEMANTIC_BOOL);
    ADD_T(gcs_link_quality);
    ADD_T(satellite_count);
    ADD_T(gps_fix);
    ADD_T(rc_link_quality);
    ADD_TS(uplink_present, proto::FIELD_SEMANTIC_BOOL);
    ADD_T(altitude_raw);
    ADD_T(altitude_amsl);
    ADD_T(altitude_agl);
    ADD_T(air_speed);
    ADD_T(course);
    ADD_T(ground_speed);
    ADD_T(heading);
    ADD_T(latitude);
    ADD_T(longitude);
    ADD_T(pitch);
    ADD_T(roll);
    ADD_T(main_voltage);
    ADD_TS(home_latitude, proto::FIELD_SEMANTIC_LATITUDE);
    ADD_TS(home_longitude, proto::FIELD_SEMANTIC_LONGITUDE);
    ADD_TS(home_altitude_amsl, proto::FIELD_SEMANTIC_ALTITUDE_AMSL);
    ADD_TS(home_altitude_raw, proto::FIELD_SEMANTIC_ALTITUDE_RAW);

    ADD_T(flight_mode);
    ADD_T(native_flight_mode);
    ADD_T(autopilot_status);
    ADD_TS(current_command, Property::VALUE_TYPE_INT);
    ADD_TS(current_mission_id, Property::VALUE_TYPE_INT);
    ADD_TS(target_latitude, proto::FIELD_SEMANTIC_LATITUDE);
    ADD_TS(target_longitude, proto::FIELD_SEMANTIC_LONGITUDE);
    ADD_TS(target_altitude_amsl, proto::FIELD_SEMANTIC_ALTITUDE_AMSL);
    ADD_TS(target_altitude_raw, proto::FIELD_SEMANTIC_ALTITUDE_RAW);
    ADD_TS(fence_enabled, proto::FIELD_SEMANTIC_BOOL);

    ADD_TS(health_rangefinder, proto::FIELD_SEMANTIC_BOOL);

    // Create command definitions.

    c_arm = flight_controller->Add_command("arm", false);

    c_auto = flight_controller->Add_command("auto", false);

    c_direct_vehicle_control = flight_controller->Add_command("direct_vehicle_control", false);
    prop = c_direct_vehicle_control->Add_parameter("pitch", Property::VALUE_TYPE_FLOAT);
    prop->Max_value()->Set_value(1);
    prop->Min_value()->Set_value(-1);
    prop = c_direct_vehicle_control->Add_parameter("roll", Property::VALUE_TYPE_FLOAT);
    prop->Max_value()->Set_value(1);
    prop->Min_value()->Set_value(-1);
    prop = c_direct_vehicle_control->Add_parameter("yaw", Property::VALUE_TYPE_FLOAT);
    prop->Max_value()->Set_value(1);
    prop->Min_value()->Set_value(-1);
    prop = c_direct_vehicle_control->Add_parameter("throttle", Property::VALUE_TYPE_FLOAT);
    prop->Max_value()->Set_value(1);
    prop->Min_value()->Set_value(-1);

    c_disarm = flight_controller->Add_command("disarm", false);

    c_emergency_land = flight_controller->Add_command("emergency_land", false);

    c_guided = flight_controller->Add_command("guided", false);

    c_joystick = flight_controller->Add_command("joystick", false);

    c_land_command = flight_controller->Add_command("land_command", false);

    c_manual = flight_controller->Add_command("manual", false);

    c_mission_upload = flight_controller->Add_command("mission_upload", false);
    c_mission_upload->Add_parameter("altitude_origin");
    c_mission_upload->Add_parameter("name");
    c_mission_upload->Add_parameter("safe_altitude", proto::FIELD_SEMANTIC_ALTITUDE_AMSL);
    c_mission_upload->Add_parameter("rth_wait_altitude", proto::FIELD_SEMANTIC_ALTITUDE_AMSL);

    c_mission_clear = flight_controller->Add_command("mission_clear", false); 

    // Derived vehicle should set supported enum values for these actions
    // possibly from proto::Failsafe_action
    p_rc_loss_action = c_mission_upload->Add_parameter("rc_loss_action", Property::VALUE_TYPE_ENUM);
    p_gps_loss_action = c_mission_upload->Add_parameter("gps_loss_action", Property::VALUE_TYPE_ENUM);
    p_low_battery_action = c_mission_upload->Add_parameter("low_battery_action", Property::VALUE_TYPE_ENUM);
    p_rth_action = c_mission_upload->Add_parameter("rth_action", Property::VALUE_TYPE_ENUM);
    // Additional parameters should be added by derived classes.

    c_pause = flight_controller->Add_command("mission_pause", true);
    c_pause->Add_parameter("additional_altitude", Property::VALUE_TYPE_FLOAT);

    c_resume = flight_controller->Add_command("mission_resume", false);

    c_rth = flight_controller->Add_command("return_to_home", false);

    c_takeoff_command = flight_controller->Add_command("takeoff_command", false);
    c_takeoff_command->Add_parameter("relative_altitude", proto::FIELD_SEMANTIC_ALTITUDE_RAW);

    c_waypoint = flight_controller->Add_command("waypoint", false);
    c_waypoint->Add_parameter("latitude");
    c_waypoint->Add_parameter("longitude");
    c_waypoint->Add_parameter("altitude_amsl");
    c_waypoint->Add_parameter("acceptance_radius");
    c_waypoint->Add_parameter("altitude_origin");
    c_waypoint->Add_parameter("ground_speed");
    c_waypoint->Add_parameter("vertical_speed");
    c_waypoint->Add_parameter("heading");

    c_set_servo = flight_controller->Add_command("set_servo", true);
    c_set_servo->Add_parameter("servo_id", Property::VALUE_TYPE_INT);
    c_set_servo->Add_parameter("pwm", Property::VALUE_TYPE_INT);

    c_repeat_servo = flight_controller->Add_command("repeat_servo", true);
    c_repeat_servo->Add_parameter("servo_id", Property::VALUE_TYPE_INT);
    c_repeat_servo->Add_parameter("pwm", Property::VALUE_TYPE_INT);
    c_repeat_servo->Add_parameter("count", Property::VALUE_TYPE_INT);
    c_repeat_servo->Add_parameter("delay", Property::VALUE_TYPE_FLOAT);

    c_set_fence = flight_controller->Add_command("set_fence", false);
    c_set_fence->Add_parameter("altitude_origin");
    c_set_fence->Add_parameter("altitude_amsl");
    c_set_fence->Add_parameter("altitude_raw");
    c_set_fence->Add_parameter("radius", Property::VALUE_TYPE_FLOAT);
    c_set_fence->Add_parameter("longitudes", Property::VALUE_TYPE_LIST);
    c_set_fence->Add_parameter("latitudes", Property::VALUE_TYPE_LIST);

    c_trigger_calibration = flight_controller->Add_command("trigger_calibration", false);

    c_trigger_reboot = flight_controller->Add_command("trigger_reboot", false);

    c_write_parameter = flight_controller->Add_command("write_parameter", false);
    c_write_parameter->Add_parameter("name", proto::FIELD_SEMANTIC_STRING);
    c_write_parameter->Add_parameter("value", proto::FIELD_SEMANTIC_ANY);

// Legacy mission items

    c_move = flight_controller->Add_command("move", true);
    p_wp_turn_type = c_move->Add_parameter("turn_type", Property::VALUE_TYPE_ENUM);
    p_wp_turn_type->Add_enum("stop_and_turn", proto::TURN_TYPE_STOP_AND_TURN);
    p_wp_turn_type->Add_enum("straight", proto::TURN_TYPE_STRAIGHT);
    p_wp_turn_type->Add_enum("spline", proto::TURN_TYPE_SPLINE);
    p_wp_turn_type->Add_enum("bank_turn", proto::TURN_TYPE_BANK_TURN);
    c_move->Add_parameter("latitude");
    c_move->Add_parameter("longitude");
    c_move->Add_parameter("altitude_amsl");
    c_move->Add_parameter("acceptance_radius");
    c_move->Add_parameter("loiter_radius", Property::VALUE_TYPE_FLOAT);
    c_move->Add_parameter("wait_time", Property::VALUE_TYPE_FLOAT);
    c_move->Add_parameter("heading");
    c_move->Add_parameter("ground_elevation");
    c_move->Add_parameter("follow_terrain", Property::VALUE_TYPE_BOOL);

    c_wait = flight_controller->Add_command("wait", true);
    c_wait->Add_parameter("time", Property::VALUE_TYPE_FLOAT);

    c_set_speed = flight_controller->Add_command("set_speed", true);
    c_set_speed->Add_parameter("ground_speed");
    c_set_speed->Add_parameter("vertical_speed");

    c_set_home = flight_controller->Add_command("set_home", true);
    c_set_home->Add_parameter("latitude");
    c_set_home->Add_parameter("longitude");
    c_set_home->Add_parameter("altitude_amsl");
    c_set_home->Add_parameter("ground_elevation");

    c_set_poi = flight_controller->Add_command("set_poi", true);
    c_set_poi->Add_parameter("latitude");
    c_set_poi->Add_parameter("longitude");
    c_set_poi->Add_parameter("altitude_amsl");
    c_set_poi->Add_parameter("active", Property::VALUE_TYPE_BOOL);

    c_set_heading = flight_controller->Add_command("set_heading", true);
    c_set_heading->Add_parameter("heading");

    c_set_relative_heading = flight_controller->Add_command("set_relative_heading", false);
    c_set_relative_heading->Add_parameter("relative_heading", ugcs::vsm::Property::VALUE_TYPE_FLOAT);

    c_panorama =flight_controller->Add_command("panorama", true);
    prop = c_panorama->Add_parameter("mode", Property::VALUE_TYPE_ENUM);
    prop->Add_enum("photo", proto::PANORAMA_MODE_PHOTO);
    prop->Add_enum("video", proto::PANORAMA_MODE_VIDEO);
    c_panorama->Add_parameter("angle", Property::VALUE_TYPE_FLOAT);
    c_panorama->Add_parameter("step", Property::VALUE_TYPE_FLOAT);
    c_panorama->Add_parameter("delay", Property::VALUE_TYPE_FLOAT);
    c_panorama->Add_parameter("speed", Property::VALUE_TYPE_FLOAT);

    c_takeoff_mission = flight_controller->Add_command("takeoff_mission", true);
    c_takeoff_mission->Add_parameter("latitude");
    c_takeoff_mission->Add_parameter("longitude");
    c_takeoff_mission->Add_parameter("altitude_amsl");
    c_takeoff_mission->Add_parameter("acceptance_radius");
    c_takeoff_mission->Add_parameter("heading");
    c_takeoff_mission->Add_parameter("climb_rate");
    c_takeoff_mission->Add_parameter("ground_elevation");

    c_land_mission = flight_controller->Add_command("land_mission", true);
    c_land_mission->Add_parameter("latitude");
    c_land_mission->Add_parameter("longitude");
    c_land_mission->Add_parameter("altitude_amsl");
    c_land_mission->Add_parameter("acceptance_radius");
    c_land_mission->Add_parameter("heading");
    c_land_mission->Add_parameter("descent_rate");
    c_land_mission->Add_parameter("ground_elevation");
    c_land_mission->Add_parameter("follow_terrain", Property::VALUE_TYPE_BOOL);

// Create primary camera. (Derived vehicles can add other cameras and/or add new commands to this one)
    primary_camera = Add_subsystem(proto::SUBSYSTEM_TYPE_CAMERA);

    c_camera_video_source = primary_camera->Add_command("select_as_video_source", false);

    c_camera_power = primary_camera->Add_command("camera_power", false);
    prop = c_camera_power->Add_parameter("power_state", Property::VALUE_TYPE_ENUM);
    prop->Add_enum("on", static_cast<int>(proto::CAMERA_POWER_STATE_ON));
    prop->Add_enum("off", static_cast<int>(proto::CAMERA_POWER_STATE_OFF));
    prop->Add_enum("toggle", static_cast<int>(proto::CAMERA_POWER_STATE_TOGGLE));

    c_camera_trigger_command = primary_camera->Add_command("camera_trigger_command", false);
    prop = c_camera_trigger_command->Add_parameter("trigger_state", Property::VALUE_TYPE_ENUM);
    prop->Add_enum("single_shot", static_cast<int>(proto::CAMERA_COMMAND_TRIGGER_STATE_SINGLE_SHOT));
    prop->Add_enum("video_start", static_cast<int>(proto::CAMERA_COMMAND_TRIGGER_STATE_VIDEO_START));
    prop->Add_enum("video_stop", static_cast<int>(proto::CAMERA_COMMAND_TRIGGER_STATE_VIDEO_STOP));
    prop->Add_enum("video_toggle", static_cast<int>(proto::CAMERA_COMMAND_TRIGGER_STATE_VIDEO_TOGGLE));

    c_camera_trigger_mission = primary_camera->Add_command("camera_trigger_mission", true);
    prop = c_camera_trigger_mission->Add_parameter("state", Property::VALUE_TYPE_ENUM);
    prop->Add_enum("off", static_cast<int>(proto::CAMERA_MISSION_TRIGGER_STATE_OFF));
    prop->Add_enum("on", static_cast<int>(proto::CAMERA_MISSION_TRIGGER_STATE_ON));
    prop->Add_enum("serial_photo", static_cast<int>(proto::CAMERA_MISSION_TRIGGER_STATE_SERIAL_PHOTO));
    prop->Add_enum("single_photo", static_cast<int>(proto::CAMERA_MISSION_TRIGGER_STATE_SINGLE_PHOTO));

    c_camera_by_distance = primary_camera->Add_command("camera_trigger_by_distance", true);
    c_camera_by_distance->Add_parameter("distance", Property::VALUE_TYPE_FLOAT);
    c_camera_by_distance->Add_parameter("count", Property::VALUE_TYPE_INT);
    c_camera_by_distance->Add_parameter("delay", Property::VALUE_TYPE_FLOAT);

    c_camera_by_time = primary_camera->Add_command("camera_trigger_by_time", true);
    c_camera_by_time->Add_parameter("period", Property::VALUE_TYPE_FLOAT);
    c_camera_by_time->Add_parameter("count", Property::VALUE_TYPE_INT);
    c_camera_by_time->Add_parameter("delay", Property::VALUE_TYPE_FLOAT);

    c_payload_control = primary_camera->Add_command("payload_control", true);
    c_payload_control->Add_parameter("tilt", ugcs::vsm::proto::FIELD_SEMANTIC_PITCH);
    c_payload_control->Add_parameter("roll");
    c_payload_control->Add_parameter("yaw");
    c_payload_control->Add_parameter("zoom_level", ugcs::vsm::Property::VALUE_TYPE_FLOAT);

    t_video_stream_uri = primary_camera->Add_telemetry("video_stream_uri", proto::FIELD_SEMANTIC_STRING);

// Create gimbal.
    primary_gimbal = Add_subsystem(proto::SUBSYSTEM_TYPE_GIMBAL);

    c_direct_payload_control = primary_gimbal->Add_command("direct_payload_control", false);
    prop = c_direct_payload_control->Add_parameter("pitch", Property::VALUE_TYPE_FLOAT);
    prop->Max_value()->Set_value(1);
    prop->Min_value()->Set_value(-1);
    prop = c_direct_payload_control->Add_parameter("roll", Property::VALUE_TYPE_FLOAT);
    prop->Max_value()->Set_value(1);
    prop->Min_value()->Set_value(-1);
    prop = c_direct_payload_control->Add_parameter("yaw", Property::VALUE_TYPE_FLOAT);
    prop->Max_value()->Set_value(1);
    prop->Min_value()->Set_value(-1);
    prop = c_direct_payload_control->Add_parameter("zoom", Property::VALUE_TYPE_FLOAT);
    prop->Max_value()->Set_value(1);
    prop->Min_value()->Set_value(-1);

    t_gimbal_roll = primary_gimbal->Add_telemetry("roll", proto::FIELD_SEMANTIC_ROLL);
    t_gimbal_pitch = primary_gimbal->Add_telemetry("pitch", proto::FIELD_SEMANTIC_PITCH);
    t_gimbal_heading = primary_gimbal->Add_telemetry("heading", proto::FIELD_SEMANTIC_HEADING);

    auto props = ugcs::vsm::Properties::Get_instance().get();

    if (props->Exists("vehicle.command_try_count")) {
        command_try_count = props->Get_int("vehicle.command_try_count");
        if (command_try_count < 1) {
            command_try_count = DEFAULT_COMMAND_TRY_COUNT;
        } else {
            LOG_INFO("[%s:%s] Setting command try count to %d",
                    model_name.c_str(),
                    serial_number.c_str(),
                    command_try_count);
        }
    }

    if (props->Exists("vehicle.command_timeout")) {
        command_timeout =
            std::chrono::milliseconds(
                static_cast<int>(props->Get_float("vehicle.command_timeout") * 1000));
        if (command_timeout.count() < 1) {
            command_timeout = DEFAULT_COMMAND_TIMEOUT;
        } else {
            LOG_INFO("[%s:%s] Setting command timeout to %" PRId64 " ms",
                    model_name.c_str(),
                    serial_number.c_str(),
                    command_timeout.count());
        }
    }

    if (props->Exists("vehicle.serial_prefix")) {
        vehicle_serial_prefix = props->Get("vehicle.serial_prefix");
    }
}

void
Vehicle::Reset_altitude_origin()
{
    CREATE_COMMIT_SCOPE;
    if (current_altitude_origin) {
        t_altitude_origin->Set_value(*current_altitude_origin);
    } else {
        t_altitude_origin->Set_value_na();
    }
    t_altitude_origin->Set_changed();   // Force sending even if value has not changed.
}

void
Vehicle::Set_altitude_origin(float altitude_amsl)
{
    CREATE_COMMIT_SCOPE;
    current_altitude_origin = altitude_amsl;
    t_altitude_origin->Set_value(altitude_amsl);
    t_altitude_origin->Set_changed();   // Force sending even if value has not changed.
}

void
Vehicle::Handle_ucs_command(
    Ucs_request::Ptr ucs_request)
{
    if (ucs_request->request.device_commands_size() != 1) {
        Command_failed(
            ucs_request,
            "Only one command allowed in ucs message, got " +
                std::to_string(ucs_request->request.device_commands_size()));
        return;
    }

    auto completion_handler = Make_callback(
        &Vehicle::Command_completed,
        Shared_from_this(),
        Vehicle_request::Result::NOK,
        std::string(),
        ucs_request);

    auto &vsm_cmd = ucs_request->request.device_commands(0);
    auto cmd = Get_command(vsm_cmd.command_id());

    if (cmd) {
        VEHICLE_LOG_INF((*this), "COMMAND %s", Dump_command(vsm_cmd).c_str());
    } else {
        Command_failed(
                ucs_request,
                "Unknown command id " + std::to_string(vsm_cmd.command_id()),
                ugcs::vsm::proto::STATUS_INVALID_COMMAND);
        return;
    }

    Vehicle_task_request::Ptr task = nullptr;
    Property_list params;

    try {
        params = cmd->Build_parameter_list(vsm_cmd);
        if (cmd == c_mission_upload || cmd == c_get_native_route) {
            std::string route_name;
            if (params.Get_value("name", route_name)) {
                VEHICLE_LOG_INF((*this), "Route name : %s", route_name.c_str());
            }
            task = Vehicle_task_request::Create(
                completion_handler,
                completion_ctx,
                vsm_cmd.sub_commands_size());

            float altitude_origin;
            if (params.Get_value("altitude_origin", altitude_origin)) {
                LOG("Altitude origin: %f", altitude_origin);
                task->payload.Set_takeoff_altitude(altitude_origin);
            } else {
                VSM_EXCEPTION(Action::Format_exception, "Altitude origin not present in mission");
            }

            task->payload.attributes = Task_attributes_action::Create(params);
            if (cmd == c_get_native_route) {
                task->payload.return_native_route = true;
                params.at("use_crlf")->Get_value(task->payload.use_crlf_in_native_route);
            }

            auto item_count = 0;
            for (int i = 0; i < vsm_cmd.sub_commands_size(); i++) {
                auto vsm_scmd = vsm_cmd.sub_commands(i);
                auto cmd = Get_command(vsm_scmd.command_id());
                if (cmd) {
                    VEHICLE_LOG_INF((*this), "MISSION item %s", Dump_command(vsm_scmd).c_str());
                    params = cmd->Build_parameter_list(vsm_scmd);
                    if (!cmd->Is_mission_item()) {
                        VSM_EXCEPTION(Action::Format_exception, "Command not allowed in mission");
                    }
                    Action::Ptr action = nullptr;
                    float tmp = std::nanf("");
                    if (cmd == c_set_parameter) {
                        // Used only by arduplane params ("LANDING_FLARE_TIME" and friends)
                        task->payload.parameters = std::move(params);
                    } else if (cmd == c_move) {
                        action = Move_action::Create(params);
                    } else if (cmd == c_land_mission) {
                        action = Landing_action::Create(params);
                    } else if (cmd == c_takeoff_mission) {
                        action = Takeoff_action::Create(params);
                    } else if (cmd == c_wait) {
                        if (!params.at("time")->Get_value(tmp)) {
                            tmp = -1;
                        }
                        action = Wait_action::Create(tmp);
                    } else if (cmd == c_pause) {
                        action = Wait_action::Create(-1);
                    } else if (cmd == c_set_speed) {
                        action = Change_speed_action::Create(params);
                    } else if (cmd == c_set_home) {
                        action = Set_home_action::Create(params);
                    } else if (cmd == c_set_poi) {
                        action = Poi_action::Create(params);
                    } else if (cmd == c_set_heading) {
                        params.at("heading")->Get_value(tmp);
                        action = Heading_action::Create(tmp);
                    } else if (cmd == c_panorama) {
                        action = Panorama_action::Create(params);
                    } else if (cmd == c_camera_trigger_mission) {
                        action = Camera_trigger_action::Create(params);
                    } else if (cmd == c_camera_by_time) {
                        action = Camera_series_by_time_action::Create(params);
                    } else if (cmd == c_camera_by_distance) {
                        action = Camera_series_by_distance_action::Create(params);
                    } else if (cmd == c_payload_control) {
                        action = Camera_control_action::Create(params);
                    } else if (cmd == c_set_servo) {
                        action = Set_servo_action::Create(params);
                    } else if (cmd == c_repeat_servo) {
                        action = Repeat_servo_action::Create(params);
                    } else if (cmd == c_transition_fixed) {
                        action = Vtol_transition_action::Create(Vtol_transition_action::FIXED);
                    } else if (cmd == c_transition_vtol) {
                        action = Vtol_transition_action::Create(Vtol_transition_action::VTOL);
                    } else {
                        VSM_EXCEPTION(
                            Action::Format_exception,
                            "Unsupported mission item '%s'", cmd->Get_name().c_str());
                    }
                    if (action) {
                        action->Set_id(item_count);
                        task->payload.actions.push_back(action);
                    }
                } else {
                    VSM_EXCEPTION(Action::Format_exception, "Unregistered mission item %d", vsm_scmd.command_id());
                }
                item_count++;
            }
            task->payload.ucs_response = ucs_request->response;
            Submit_vehicle_request(task);
        } else {
            Vehicle_command::Type ctype;
            if (cmd == c_arm) {
                ctype = Vehicle_command::Type::ARM;
            } else if (cmd == c_auto) {
                ctype = Vehicle_command::Type::AUTO_MODE;
            } else if (cmd == c_direct_vehicle_control) {
                ctype = Vehicle_command::Type::DIRECT_VEHICLE_CONTROL;
            } else if (cmd == c_disarm) {
                ctype = Vehicle_command::Type::DISARM;
            } else if (cmd == c_guided) {
                ctype = Vehicle_command::Type::GUIDED_MODE;
            } else if (cmd == c_joystick) {
                ctype = Vehicle_command::Type::JOYSTICK_CONTROL_MODE;
            } else if (cmd == c_land_command) {
                ctype = Vehicle_command::Type::LAND;
            } else if (cmd == c_takeoff_command) {
                ctype = Vehicle_command::Type::TAKEOFF;
            } else if (cmd == c_manual) {
                ctype = Vehicle_command::Type::MANUAL_MODE;
            } else if (cmd == c_pause) {
                ctype = Vehicle_command::Type::PAUSE_MISSION;
            } else if (cmd == c_resume) {
                ctype = Vehicle_command::Type::RESUME_MISSION;
            } else if (cmd == c_rth) {
                ctype = Vehicle_command::Type::RETURN_HOME;
            } else if (cmd == c_waypoint) {
                ctype = Vehicle_command::Type::WAYPOINT;
            } else if (cmd == c_emergency_land) {
                ctype = Vehicle_command::Type::EMERGENCY_LAND;
            } else if (cmd == c_camera_trigger_command) {
                ctype = Vehicle_command::Type::CAMERA_TRIGGER;
            } else if (cmd == c_direct_payload_control) {
                ctype = Vehicle_command::Type::DIRECT_PAYLOAD_CONTROL;
            } else if (cmd == c_camera_power) {
                ctype = Vehicle_command::Type::CAMERA_POWER;
            } else if (cmd == c_camera_video_source) {
                ctype = Vehicle_command::Type::CAMERA_VIDEO_SOURCE;
            } else {
                ucs_request->Complete(
                    ugcs::vsm::proto::STATUS_INVALID_COMMAND,
                    "Unsupported command. Only legacy commands supported for now.");
                return;
            }
            auto task = Vehicle_command_request::Create(completion_handler, completion_ctx, ctype, params);
            Submit_vehicle_request(task);
        }
    } catch (const std::exception& ex) {
        ucs_request->Complete(ugcs::vsm::proto::STATUS_INVALID_COMMAND, ex.what());
        if (task) {
            task->Abort();
        }
    }
}

void
Vehicle::Command_completed(
    ugcs::vsm::Vehicle_request::Result result,
    const std::string& status_text,
    Ucs_request::Ptr ucs_request)
{
    if (result == Vehicle_request::Result::OK) {
        if (ucs_request->response) {
            VEHICLE_LOG_INF((*this), "COMMAND OK");
        }
        ucs_request->Complete();
    } else {
        VEHICLE_LOG_WRN((*this), "COMMAND FAILED: %s", status_text.c_str());
        ucs_request->Complete(ugcs::vsm::proto::STATUS_FAILED, status_text);
    }
}

void
Vehicle::Command_succeeded(
    Ucs_request::Ptr ucs_request)
{
    if (ucs_request && !ucs_request->Is_completed()) {
        if (ucs_request->response) {
            VEHICLE_LOG_WRN((*this), "COMMAND OK");
        }
        ucs_request->Complete();
    }
}

void
Vehicle::Command_failed(
    Ucs_request::Ptr ucs_request,
    const std::string& status_text,
    proto::Status_code code)
{
    if (ucs_request && !ucs_request->Is_completed()) {
        VEHICLE_LOG_WRN((*this), "COMMAND FAILED: %s", status_text.c_str());
        ucs_request->Complete(code, status_text);
    }
}

void
Vehicle::Handle_vehicle_request(Vehicle_task_request::Handle)
{
    LOG_DEBUG(
        "Mission to vehicle [%s:%s] is ignored.",
        Get_serial_number().c_str(), Get_model_name().c_str());
}

void
Vehicle::Handle_vehicle_request(Vehicle_command_request::Handle)
{
    LOG_DEBUG("Command for vehicle [%s:%s] is ignored.", Get_serial_number().c_str(),
            Get_model_name().c_str());
}

void
Vehicle::Set_serial_number(const std::string& s)
{

    if (vehicle_serial_prefix.size()) {
        Set_property("vehicle_serial", vehicle_serial_prefix + s);
    } else {
        Set_property("vehicle_serial", s);
    }
    serial_number = s;
}

const std::string&
Vehicle::Get_serial_number() const
{
    return serial_number;
}

void
Vehicle::Set_frame_type(const std::string& s)
{
    Set_property("frame_type", s);
    frame_type = s;
}

const std::string&
Vehicle::Get_frame_type() const
{
    return frame_type;
}

void
Vehicle::Set_model_name(const std::string& s)
{
    Set_property("vehicle_name", s);
    model_name = s;
}

const std::string&
Vehicle::Get_model_name() const
{
    return model_name;
}

proto::Vehicle_type
Vehicle::Get_vehicle_type() const
{
    return vehicle_type;
}

void
Vehicle::Set_vehicle_type(proto::Vehicle_type type)
{
    Set_property("vehicle_type", type);
    vehicle_type = type;
}

bool
Vehicle::Is_vehicle_type(proto::Vehicle_type type)
{
    return vehicle_type == type;
}

void
Vehicle::Set_port_name(const std::string& s)
{
    Set_property("port_name", s);
    port_name = s;
}

const std::string&
Vehicle::Get_port_name() const
{
    return port_name;
}

void
Vehicle::Set_rc_loss_actions(std::initializer_list<proto::Failsafe_action> actions)
{
    Set_failsafe_actions(p_rc_loss_action, actions);
}

void
Vehicle::Set_low_battery_actions(std::initializer_list<proto::Failsafe_action> actions)
{
    Set_failsafe_actions(p_low_battery_action, actions);
}

void
Vehicle::Set_gps_loss_actions(std::initializer_list<proto::Failsafe_action> actions)
{
    Set_failsafe_actions(p_gps_loss_action, actions);
}

void
Vehicle::Set_rth_actions(std::initializer_list<proto::Rth_action> actions)
{
    for (auto a : actions) {
        switch (a) {
        case proto::RTH_ACTION_WAIT:
            p_rth_action->Add_enum("wait", a);
            break;
        case proto::RTH_ACTION_LAND:
            p_rth_action->Add_enum("land", a);
            break;
        }
    }
    auto i = actions.begin();
    if (i != actions.end()) {
        p_rth_action->Default_value()->Set_value(*i);
    }
}

void
Vehicle::Command_map::Fill_command_mapping_response(Proto_msg_ptr msg)
{
    if (msg) {
        auto res = msg->mutable_device_response();
        res->set_mission_id(Get_route_id());
        for (auto i : mission_command_map) {
            auto m = res->add_mission_command_map();
            m->set_vehicle_command_id(i.first);
            m->set_mission_command_idx(i.second);
        }
    }
}

void
Vehicle::Command_map::Reset()
{
    mission_command_map.clear();
    mission_id.Reset();
    current_mission_command = -1;
}

void
Vehicle::Command_map::Set_current_command(int mission_command_id)
{
    current_mission_command = mission_command_id;
}

void
Vehicle::Command_map::Add_command_mapping(int vehicle_specific_id)
{
    if (current_mission_command >= 0) {
        mission_command_map.emplace(vehicle_specific_id, current_mission_command);
    }
}


void
Vehicle::Command_map::Accumulate_route_id(uint32_t hash)
{
    mission_id.Add_int(hash);
}

void
Vehicle::Command_map::Set_secondary_id(uint32_t id)
{
    secondary_id = id;
}

uint32_t
Vehicle::Command_map::Get_route_id()
{
    return mission_id.Get() + secondary_id;
}

bool
Vehicle::Is_flight_mode(ugcs::vsm::proto::Flight_mode m)
{
    return (current_flight_mode && *current_flight_mode == m);
}

bool
Vehicle::Is_control_mode(ugcs::vsm::proto::Control_mode m)
{
    int cmode;
    if (t_control_mode && t_control_mode->Get_value(cmode)) {
        return cmode == m;
    } else {
        return false;
    }
}

void
Vehicle::Set_autopilot_type(const std::string& type)
{
    flight_controller->Set_property("autopilot_type", type);
}

void
Vehicle::Set_autopilot_serial(const std::string& serial)
{
    flight_controller->Set_property("autopilot_serial", serial);
}
