// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file vehicle.h
 *
 * Vehicle interface representation.
 */

#ifndef _UGCS_VSM_VEHICLE_H_
#define _UGCS_VSM_VEHICLE_H_

#include <ugcs/vsm/request_worker.h>
#include <ugcs/vsm/vehicle_requests.h>
#include <ugcs/vsm/mavlink.h>
#include <ugcs/vsm/enum_set.h>
#include <ugcs/vsm/device.h>
#include <ugcs/vsm/crc32.h>

#include <stdint.h>
#include <memory>
#include <unordered_map>
#include <string>
#include <utility>
#include <vector>

namespace ugcs {
namespace vsm {

static constexpr int DEFAULT_COMMAND_TRY_COUNT = 3;
static constexpr std::chrono::milliseconds DEFAULT_COMMAND_TIMEOUT(1000);

/** Base class for user-defined vehicles. It contains interface to SDK services
 * which can be used as base class methods calls, and abstract interface which
 * must be implemented by the device.
 * Instance creation should be done by {@link Vehicle::Create()} method.
 */
class Vehicle: public Device {
    DEFINE_COMMON_CLASS(Vehicle, Device)

public:
    /**
     * Constructor for a base class.
     * Create common telemetry fields and commands.
     * @param create_thread If @a true, then separate thread is automatically
     * created for vehicle instance which allows to using blocking methods
     * in the context of this vehicle without blocking other vehicles, otherwise
     * vehicle instance thread is not created and user is supposed to call
     * @ref Vehicle::Process_requests method to process requests pending for
     * this vehicle. It is recommended to leave this value as default, i.e.
     * "true".
     */
    Vehicle(
        proto::Device_type type = proto::DEVICE_TYPE_VEHICLE,
        Request_processor::Ptr processor = nullptr,
        Request_completion_context::Ptr completion_ctx = nullptr);

    /** Make sure class is polymorphic. */
    virtual
    ~Vehicle() {}

    /** Disable copying. */
    Vehicle(const Vehicle &) = delete;

    /** Process requests assigned to vehicle in user thread, i.e. the thread
     * which calls this method. Supposed to be called only when vehicle is
     * created without its own thread.
     *
     */
    void
    Process_requests();

    /** Set serial number */
    void
    Set_serial_number(const std::string&);

    /** Get serial number of the vehicle. */
    const std::string&
    Get_serial_number() const;

    /** Get model name of the vehicle */
    const std::string&
    Get_model_name() const;

    /** Set model name of the vehicle */
    void
    Set_model_name(const std::string&);

    /** Set port name */
    void
    Set_port_name(const std::string&);

    /** Get port name the vehicle is connected to. */
    const std::string&
    Get_port_name() const;

    /** Get frame type of the vehicle */
    const std::string&
    Get_frame_type() const;

    /** Set frame type of the vehicle */
    void
    Set_frame_type(const std::string&);

    // Ground/Plane/Copter/VTOL...
    proto::Vehicle_type
    Get_vehicle_type() const;

    // Ground/Plane/Copter/VTOL...
    void
    Set_vehicle_type(proto::Vehicle_type);

    bool
    Is_vehicle_type(proto::Vehicle_type type);

    bool
    Is_copter() {
        return vehicle_type == proto::VEHICLE_TYPE_HELICOPTER || vehicle_type == proto::VEHICLE_TYPE_MULTICOPTER;
    }


    /** Hasher for Vehicle shared pointer. Used when vehicle pointer is
     * stored in some container. */
    class Hasher {
    public:
        /** Get hash value. */
        size_t
        operator()(const Vehicle::Ptr& ptr) const {
            return hasher(ptr.get());
        }
    private:
        /** Standard pointer hasher. */
        static std::hash<Vehicle*> hasher;
    };

protected:
    Optional<proto::Flight_mode> current_flight_mode;

    Property::Ptr t_control_mode = nullptr;
    Property::Ptr t_is_armed = nullptr;
    Property::Ptr t_uplink_present = nullptr;
    Property::Ptr t_downlink_present = nullptr;
    Property::Ptr t_main_voltage = nullptr;
    Property::Ptr t_main_current = nullptr;
    Property::Ptr t_gcs_link_quality = nullptr;
    Property::Ptr t_rc_link_quality = nullptr;
    Property::Ptr t_latitude = nullptr;
    Property::Ptr t_longitude = nullptr;
    Property::Ptr t_altitude_raw = nullptr;
    Property::Ptr t_altitude_amsl = nullptr;
    Property::Ptr t_altitude_agl = nullptr;
    Property::Ptr t_ground_speed = nullptr;
    Property::Ptr t_air_speed = nullptr;
    Property::Ptr t_course = nullptr;
    Property::Ptr t_vertical_speed = nullptr;
    Property::Ptr t_pitch = nullptr;
    Property::Ptr t_roll = nullptr;
    Property::Ptr t_heading = nullptr;
    Property::Ptr t_health_rangefinder = nullptr;
    Property::Ptr t_gps_fix = nullptr;
    Property::Ptr t_satellite_count = nullptr;
    Property::Ptr t_altitude_origin = nullptr;
    Property::Ptr t_home_altitude_amsl = nullptr;
    Property::Ptr t_home_altitude_raw = nullptr;
    Property::Ptr t_home_latitude = nullptr;
    Property::Ptr t_home_longitude = nullptr;
    Property::Ptr t_target_altitude_amsl = nullptr;
    Property::Ptr t_target_altitude_raw = nullptr;
    Property::Ptr t_target_latitude = nullptr;
    Property::Ptr t_target_longitude = nullptr;
    Property::Ptr t_current_command = nullptr;
    Property::Ptr t_current_mission_id = nullptr;
    Property::Ptr t_flight_mode = nullptr;
    Property::Ptr t_autopilot_status = nullptr;
    Property::Ptr t_native_flight_mode = nullptr;
    Property::Ptr t_fence_enabled = nullptr;
    Property::Ptr t_vibration_x = nullptr;
    Property::Ptr t_vibration_y = nullptr;
    Property::Ptr t_vibration_z = nullptr;
    Property::Ptr t_video_stream_uri = nullptr;

    Property::Ptr t_gimbal_roll = nullptr;
    Property::Ptr t_gimbal_pitch = nullptr;
    Property::Ptr t_gimbal_heading = nullptr;

    Vsm_command::Ptr c_mission_upload = nullptr;
    Vsm_command::Ptr c_auto = nullptr;
    Vsm_command::Ptr c_arm = nullptr;
    Vsm_command::Ptr c_disarm = nullptr;
    Vsm_command::Ptr c_waypoint = nullptr;
    Vsm_command::Ptr c_guided = nullptr;
    Vsm_command::Ptr c_manual = nullptr;
    Vsm_command::Ptr c_pause = nullptr;
    Vsm_command::Ptr c_resume = nullptr;
    Vsm_command::Ptr c_rth = nullptr;
    Vsm_command::Ptr c_land_command = nullptr;
    Vsm_command::Ptr c_joystick = nullptr;
    Vsm_command::Ptr c_direct_vehicle_control = nullptr;
    Vsm_command::Ptr c_takeoff_command = nullptr;
    Vsm_command::Ptr c_emergency_land = nullptr;
    Vsm_command::Ptr c_camera_trigger_command = nullptr;
    Vsm_command::Ptr c_adsb_set_ident = nullptr;
    Vsm_command::Ptr c_adsb_set_mode = nullptr;
    Vsm_command::Ptr c_direct_payload_control = nullptr;
    Vsm_command::Ptr c_camera_power = nullptr;
    Vsm_command::Ptr c_camera_video_source = nullptr;
    Vsm_command::Ptr c_adsb_set_parameter = nullptr;
    Vsm_command::Ptr c_set_servo = nullptr;
    Vsm_command::Ptr c_repeat_servo = nullptr;
    Vsm_command::Ptr c_set_fence = nullptr;
    Vsm_command::Ptr c_trigger_calibration = nullptr;
    Vsm_command::Ptr c_trigger_reboot = nullptr;
    Vsm_command::Ptr c_get_native_route = nullptr;
    Vsm_command::Ptr c_set_relative_heading = nullptr;
    Vsm_command::Ptr c_write_parameter = nullptr;
    Vsm_command::Ptr c_mission_clear = nullptr;

    Vsm_command::Ptr c_wait = nullptr;
    Vsm_command::Ptr c_move = nullptr;
    Vsm_command::Ptr c_set_speed = nullptr;
    Vsm_command::Ptr c_set_home = nullptr;
    Vsm_command::Ptr c_set_poi = nullptr;
    Vsm_command::Ptr c_set_heading = nullptr;
    Vsm_command::Ptr c_panorama = nullptr;
    Vsm_command::Ptr c_camera_trigger_mission = nullptr;
    Vsm_command::Ptr c_camera_by_time = nullptr;
    Vsm_command::Ptr c_camera_by_distance = nullptr;
    Vsm_command::Ptr c_land_mission = nullptr;
    Vsm_command::Ptr c_takeoff_mission = nullptr;
    Vsm_command::Ptr c_set_parameter = nullptr;
    Vsm_command::Ptr c_payload_control = nullptr;
    Vsm_command::Ptr c_transition_fixed = nullptr;
    Vsm_command::Ptr c_transition_vtol = nullptr;
    Vsm_command::Ptr c_wait_until = nullptr;

    Property::Ptr p_rc_loss_action = nullptr;
    Property::Ptr p_gps_loss_action = nullptr;
    Property::Ptr p_low_battery_action = nullptr;
    Property::Ptr p_rth_action = nullptr;
    Property::Ptr p_wp_turn_type = nullptr;

    int command_try_count = DEFAULT_COMMAND_TRY_COUNT;
    std::chrono::milliseconds command_timeout = DEFAULT_COMMAND_TIMEOUT;
    
    // Predefined subsytems
    Subsystem::Ptr flight_controller;
    Subsystem::Ptr primary_camera;
    Subsystem::Ptr primary_gimbal;
    Subsystem::Ptr adsb_transponder;
    Subsystem::Ptr winch_controller;

    /** Vehicle enable event handler. Can be overridden by derived class,
     * if necessary.
     */
    virtual void
    On_enable()
    {};

    /** Vehicle disable event handler. Can be overridden by derived class,
     * if necessary.
     */
    virtual void
    On_disable()
    {};

    // Sets available FS actions on RC loss with the first one as default.
    void
    Set_rc_loss_actions(std::initializer_list<proto::Failsafe_action> actions);

    // Sets available FS actions on GPS loss with the first one as default.
    void
    Set_gps_loss_actions(std::initializer_list<proto::Failsafe_action> actions);

    // Sets available FS actions on low battery with the first one as default.
    void
    Set_low_battery_actions(std::initializer_list<proto::Failsafe_action> actions);

    // Sets available actions after RTH with the first one as default.
    void
    Set_rth_actions(std::initializer_list<proto::Rth_action> actions);

    /** Tell server that current altitude origin must be dropped.
     * (calibration needed to match reported altitude to real world)
     *
     * Use this when VSM knows that currently reported Rel_altitude
     * changed unexpectedly. For example if vehicle resets the reported
     * altitude on ARM. */
    void
    Reset_altitude_origin();

    /** Tell server that Vehicle knows its altitude_origin.
     * Use this when VSM knows that reported Rel_altitude origin
     * has changed. Ardupilot does that on home location change. */
    void
    Set_altitude_origin(float altitude_amsl);

    /*
     * Below are methods which are called by VSM SDK in vehicle context and
     * should be overridden by user code.
     */

    /**
     * Message from ucs arrived
     */

    // Translates incoming message to old style Vehicle_request
    virtual void
    Handle_ucs_command(
        Ucs_request::Ptr ucs_request);

    // old style completion handler
    void
    Command_completed(
        Vehicle_request::Result result,
        const std::string& status_text,
        Ucs_request::Ptr ucs_request);

    void
    Command_failed(
        Ucs_request::Ptr ucs_request,
        const std::string& status_text,
        proto::Status_code code = proto::STATUS_FAILED);

    void
    Command_succeeded(
        Ucs_request::Ptr ucs_request);

    /**
     * Task has arrived from UCS and should be uploaded to the vehicle.
     */
    virtual void
    Handle_vehicle_request(Vehicle_task_request::Handle request);

    /**
     * UCS sending a command to the vehicle.
     */
    virtual void
    Handle_vehicle_request(Vehicle_command_request::Handle request);

    /* End of methods called by VSM SDK. */

    /* Below are methods which should be called by user code from derived
     * vehicle class. */

    // Convenience function to check current flight mode.
    bool
    Is_flight_mode(proto::Flight_mode);

    // Return true if vehicle currently is in given control mode m
    bool
    Is_control_mode(proto::Control_mode m);

    // Mission command mapping interface. Used to support current command reporting
    // During mission flight.
    //
    // Here is how it works:
    // 1. mission_upload command contains a list of sub_commands. VSM enumerates it as zero-based.
    //    This becomes the mission_command_id
    // 2. Each mission command can produce any number of vehicle specific commands.
    // 3. The mapping is sent back to server as a response to mission_upload command together with generated mission_id.
    // 4. VSM reports current_vehicle_command to the server during mission flight.
    // 5. Server uses this mapping maps vehicle_specific commands back to mission_commands.
    // 6. mission_id is used to synchronize the command mapping with the server when
    //    the mapping is unknown to VSM (eg. after restart).
    // 7. Server uses the command map only if the map exists and the reported mission_id in telemetry is equal to the
    //    mission_id received together with the map.
    // 8. If server receives different mission_id it drops the current mapping.
    class Command_map {
    public:
        Command_map() : mission_id() {}

        // Clear the command mapping and reset mission_id.
        void
        Reset();

        // Set the current_mission_command id as received from ucs.
        // All vehicle specific commands produced by this mission command
        // will map to this ID.
        void
        Set_current_command(int mission_command_id);

        // Map current_mission_command to this vehicle_command_id
        // VSM must call this on each mission item it uploads to the vehicle.
        // Or more precisely: on each command the vehicle is going to report as current command.
        void
        Add_command_mapping(int vehicle_specific_id);

        // Use this function to build mission_id.
        // Used in two scenarios:
        // 1) To report newly uploaded mission_id to server.
        // 2) To calculate mission_id from downloaded mission from the vehicle.
        //    And report as telemetry when VSM has not seen the mission_upload.
        // CRC32 algorithm is used to create a 32bit hash.
        // IMPORTANT:
        // Mission ID is calculated as hash from uploaded vehicle commands
        // in such way that it can be recreated if vehicle supports mission
        // download.
        void
        Accumulate_route_id(uint32_t hash);

        // Get the generated mission_id.
        uint32_t
        Get_route_id();

        // Add additional value to mission id hash which can be changed without regenerating the
        // mission_command_map. Used to add Home Location to route hash.
        void
        Set_secondary_id(uint32_t);

        // Fill in the mission_upload response payload with accumulated map and mission_id.
        void
        Fill_command_mapping_response(Proto_msg_ptr response);

    private:
        // Mission mapper state.
        int current_mission_command = -1;
        // Current native command mapping to mission subcommands (zero based).
        std::unordered_map<int, int> mission_command_map;

        uint32_t secondary_id = 0;

        Crc32 mission_id;
    };

    /* End of user callable methods. */

    void
    Set_autopilot_type(const std::string&);

    void
    Set_autopilot_serial(const std::string&);

private:

    /** Submit vehicle request to be processed by this vehicle. Conversion to
     * appropriate user handle type is automatic.
     */
    template<class Request_ptr>
    void
    Submit_vehicle_request(Request_ptr vehicle_request)
    {
        using Request = typename Request_ptr::element_type;
        using Handle = typename Request::Handle;
        typedef void (Vehicle::*Handler)(Handle);

        Handler method = &Vehicle::Handle_vehicle_request;
        auto proc_handler = Make_callback(method,
                                          Shared_from_this(),
                                          Handle(vehicle_request));
        vehicle_request->request->Set_processing_handler(proc_handler);
        processor->Submit_request(vehicle_request->request);
    }

    /** If vehicle is able to know it own altitude origin.*/
    Optional<float> current_altitude_origin;

    /* Friend classes mostly for accessing system_id variable which we want
     * to hide from SDK user.
     */
    friend class Ucs_vehicle_ctx;
    friend class Ucs_transaction;
    friend class Ucs_mission_clear_all_transaction;
    friend class Ucs_task_upload_transaction;
    friend class Ucs_vehicle_command_transaction;
    friend class Cucs_processor;

    /** Type of the vehicle. */
    proto::Vehicle_type vehicle_type = proto::VEHICLE_TYPE_MULTICOPTER;

    /** Serial number of the vehicle. */
    std::string serial_number;

    /** Model of the vehicle, e.g. Arducopter, Mikrokopter etc. */
    std::string model_name;

    /** Port name the vehicle is connected to. ("COM12" or "127.0.0.1:5440")*/
    std::string port_name;

    /** Frame type */
    std::string frame_type;

    /** Vehicle serial prefix
     * Used as prefix serial (if reported) for all vehicles connected to this VSM. */
    std::string vehicle_serial_prefix;
};

/** Convenience vehicle logging macro. Vehicle should be given by value (no
 * copies will be made). */
#define VEHICLE_LOG(level_, vehicle_, fmt_, ...) \
    _LOG_WRITE_MSG(level_, "[%s:%s] " fmt_, \
            (vehicle_).Get_model_name().c_str(), \
            (vehicle_).Get_serial_number().c_str(), ## __VA_ARGS__)

/** Different level convenience vehicle logging macros. Vehicle should be given
 * by value (no copies will be made). */
// @{
#define VEHICLE_LOG_DBG(vehicle_, fmt_, ...) \
    VEHICLE_LOG(::ugcs::vsm::Log::Level::DEBUGGING, vehicle_, fmt_, ## __VA_ARGS__)

#define VEHICLE_LOG_INF(vehicle_, fmt_, ...) \
    VEHICLE_LOG(::ugcs::vsm::Log::Level::INFO, vehicle_, fmt_, ## __VA_ARGS__)

#define VEHICLE_LOG_WRN(vehicle_, fmt_, ...) \
    VEHICLE_LOG(::ugcs::vsm::Log::Level::WARNING, vehicle_, fmt_, ## __VA_ARGS__)

#define VEHICLE_LOG_ERR(vehicle_, fmt_, ...) \
    VEHICLE_LOG(::ugcs::vsm::Log::Level::ERROR, vehicle_, fmt_, ## __VA_ARGS__)
// @}

} /* namespace vsm */
} /* namespace ugcs */

#endif /* _UGCS_VSM_VEHICLE_H_ */
