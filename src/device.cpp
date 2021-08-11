// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <ugcs/vsm/device.h>
#include <ugcs/vsm/cucs_processor.h>

using namespace ugcs::vsm;

Ucs_request::Ucs_request(ugcs::vsm::proto::Vsm_message m):
    request(std::move(m))
{
}

void
Ucs_request::Complete(ugcs::vsm::proto::Status_code s, const std::string& description)
{
    if (!Is_completed()) {
        if (response) {
            response->mutable_device_response()->set_code(s);
            if (description.size()) {
                response->mutable_device_response()->set_status(description);
            }
        }
        Request::Complete();
    }
}

Device::Device(
    proto::Device_type type,
    Request_processor::Ptr proc,
    Request_completion_context::Ptr comp_ctx):
        device_type(type),
        begin_of_epoch(std::chrono::system_clock::now())
{
    if (proc && comp_ctx) {
        processor = proc;
        completion_ctx = comp_ctx;
    } else if (proc == nullptr && comp_ctx == nullptr) {
        processor = Request_processor::Create("Device processor");
        completion_ctx = Request_completion_context::Create("Device completion");
        worker = Request_worker::Create(
            "Vehicle worker",
            std::initializer_list<Request_container::Ptr>(
                {completion_ctx, processor}));
    } else {
        VSM_EXCEPTION(Internal_error_exception, "Both contexts must be provided");
    }
}

Device::~Device()
{
}

uint32_t
Device::Get_session_id()
{
    return my_handle;
}

void
Device::Enable()
{
    ASSERT(!is_enabled);
    if (worker) {
        completion_ctx->Enable();
        processor->Enable();
        worker->Enable();
    }
    is_enabled = true;
    // Make sure On_enable is always called in vehicle context/thread.
    auto req = ugcs::vsm::Request::Create();
    req->Set_processing_handler(
            Make_callback(
                [this](ugcs::vsm::Request::Ptr request)
                {
                    On_enable();
                    request->Complete();
                },
                req));
    processor->Submit_request(req);
    req->Wait_done(false);
}

void
Device::Disable()
{
    Unregister();
    ASSERT(is_enabled);
    is_enabled = false;
    // Make sure On_disable is always called in vehicle context/thread.
    auto req = ugcs::vsm::Request::Create();
    req->Set_processing_handler(
            Make_callback(
                [this](ugcs::vsm::Request::Ptr request)
                {
                    On_disable();
                    request->Complete();
                },
                req));
    processor->Submit_request(req);
    req->Wait_done(false);
    if (worker) {
        completion_ctx->Disable();
        processor->Disable();
        worker->Disable();
    }
    completion_ctx = nullptr;
    processor = nullptr;
    worker = nullptr;
}

/** Get default completion context of the vehicle. */
Request_completion_context::Ptr
Device::Get_completion_ctx()
{
    return completion_ctx;
}

Request_processor::Ptr
Device::Get_processing_ctx()
{
    return processor;
}

void
Device::Register()
{
    if (!my_handle) {
        my_handle = Get_unique_id();
        Cucs_processor::Get_instance()->Register_device(Shared_from_this());
        // Push pending telemetry if any.
        Commit_to_ucs();
    }
}

void
Device::Unregister()
{
    if (my_handle) {
        Cucs_processor::Get_instance()->Unregister_device(my_handle);
        my_handle = 0;
    }
}

bool
Device::Is_registered()
{
    return my_handle != 0;
}

void
Device::On_ucs_message(
    ugcs::vsm::proto::Vsm_message message,
    Response_sender completion_handler,
    ugcs::vsm::Request_completion_context::Ptr completion_ctx)
{
    auto request = Ucs_request::Create(std::move(message));

    if (completion_ctx) {
        request->stream_id = completion_handler.Get_arg<0>();
        request->response = completion_handler.Get_arg<1>();
        request->Set_completion_handler(completion_ctx, completion_handler);
    }

    request->Set_processing_handler(
        Make_callback(
            &Device::Handle_ucs_command,
            Shared_from_this(),
            request));

    processor->Submit_request(request);
}

void
Device::Send_ucs_message(Proto_msg_ptr msg)
{
    if (my_handle) {
        Cucs_processor::Get_instance()->Send_ucs_message(my_handle, msg);
    } else {
        LOG_ERR("Send while device not registered");
    }
}

void
Device::Handle_ucs_command(
    Ucs_request::Ptr ucs_request)
{
    ucs_request->Complete(ugcs::vsm::proto::STATUS_FAILED, "Not implemented");
}

void
Device::Report_progress(Ucs_request::Ptr request, float progress, const std::string& description)
{
    // Send only if request is valid.
    if (my_handle && request && !request->Is_completed() && request->response) {
        // Make a copy of response and send progress report.
        auto resp = std::make_shared<ugcs::vsm::proto::Vsm_message>(*request->response);
        resp->mutable_device_response()->set_code(proto::STATUS_IN_PROGRESS);
        if (progress > 0) {
            if (progress > 1.0) {
                progress = 1.0;
            }
            resp->mutable_device_response()->set_progress(progress);
        }
        if (description.size()) {
            resp->mutable_device_response()->set_status(description);
        }
        Cucs_processor::Get_instance()->Send_ucs_message(my_handle, resp, request->stream_id);
    }
}

void
Device::Commit_to_ucs(bool log_message)
{
    if (!my_handle) {
        // Do not send any telemetry if not registered thus
        // Avoiding resetting dirty flags on command avail and telemetry.
        return;
    }
    auto msg = std::make_shared<ugcs::vsm::proto::Vsm_message>();
    auto report = msg->mutable_device_status();

    for (auto sd : subsystems) {
        for (auto it : sd->telemetry_fields) {
            if (it->Is_changed()) {
                auto tf = report->add_telemetry_fields();
                it->Write_as_telemetry(tf);
                tf->set_ms_since_epoch(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        it->Get_update_time() - begin_of_epoch).count());
            }
        }

        for (auto it : sd->commands) {
            if (it.second->Is_capability_state_dirty()) {
                it.second->Set_capabilities(report->add_command_availability());
            }
        }
    }

    while (!device_status_messages.empty()) {
        report->add_status_messages(device_status_messages.front());
        device_status_messages.pop_front();
        report->add_messages_severity(ugcs::vsm::proto::Severity_code::SEVERITY_INF);
    }

    while (!device_warning_messages.empty()) {
        report->add_status_messages(device_warning_messages.front());
        device_warning_messages.pop_front();
        report->add_messages_severity(ugcs::vsm::proto::Severity_code::SEVERITY_WARN);
    }

    while (!device_critical_messages.empty()) {
        report->add_status_messages(device_critical_messages.front());
        device_critical_messages.pop_front();
        report->add_messages_severity(ugcs::vsm::proto::Severity_code::SEVERITY_ERR);
    }

    if (    report->telemetry_fields_size()
        ||  report->command_availability_size()
        ||  report->status_messages_size())
    {
        if (log_message) {
            LOG("%s", msg->SerializeAsString().c_str());
        }
        Send_ucs_message(msg);
    }
}

Vsm_command::Ptr
Device::Get_command(int id)
{
    for (auto sd : subsystems) {
        auto cit = sd->commands.find(id);
        if (cit != sd->commands.end()) {
            return cit->second;
        }
    }
    return nullptr;
}

void
Device::Add_status_message(const std::string& m)
{
    device_status_messages.push_back(m);
}

void
Device::Add_warning_message(const std::string& m)
{
    device_warning_messages.push_back(m);
}

void
Device::Add_critical_message(const std::string& m)
{
    device_critical_messages.push_back(m);
}

void
Device::Set_failsafe_actions(Property::Ptr p, std::initializer_list<proto::Failsafe_action> actions)
{
    for (auto a : actions) {
        switch (a) {
        case proto::FAILSAFE_ACTION_CONTINUE:
            p->Add_enum("continue", a);
            break;
        case proto::FAILSAFE_ACTION_WAIT:
            p->Add_enum("wait", a);
            break;
        case proto::FAILSAFE_ACTION_LAND:
            p->Add_enum("land", a);
            break;
        case proto::FAILSAFE_ACTION_RTH:
            p->Add_enum("rth", a);
            break;
        }
    }
    auto i = actions.begin();
    if (i != actions.end()) {
        p->Default_value()->Set_value(*i);
    }
}

std::string
Device::Dump_command(const ugcs::vsm::proto::Device_command & vsm_cmd)
{
    auto cmd = Get_command(vsm_cmd.command_id());
    std::string ret = cmd->Get_name() + " (" + std::to_string(vsm_cmd.command_id()) + ")";
    auto params = cmd->Build_parameter_list(vsm_cmd);
    for (auto p : params) {
        ret += "\n  " + p.second->Dump_value();
    }
    // Subcommands should be dumped separately because windows hangs on long debug strings.
    return ret;
}

Subsystem::Ptr
Device::Add_subsystem(proto::Subsystem_type type)
{
    auto sd = Subsystem::Create(type);
    subsystems.emplace_back(sd);
    return sd;
}

void
Device::Register(ugcs::vsm::proto::Vsm_message& msg)
{
    // Verify device validity
    switch (device_type) {
    case proto::DEVICE_TYPE_VEHICLE:
        for (auto p : {"vehicle_type"}) {
            if (properties.find(p) == properties.end()) {
                VSM_EXCEPTION(Invalid_param_exception, "Vehicle must provide parameter %s", p);
            }
        }
        break;
    case proto::DEVICE_TYPE_ADSB_VEHICLE:
        for (auto p : {"icao"}) {
            if (properties.find(p) == properties.end()) {
                VSM_EXCEPTION(Invalid_param_exception, "ADSB vehicle must provide parameter %s", p);
            }
        }
        break;
    case proto::DEVICE_TYPE_ADSB_RECEIVER:
    case proto::DEVICE_TYPE_VEHICLE_COMMAND_PROCESSOR:
    case proto::DEVICE_TYPE_RTK_BASE_STATION:
        break;
    }

    auto dev = msg.mutable_register_device();

    dev->set_begin_of_epoch(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            begin_of_epoch.time_since_epoch()).count());

    dev->set_type(device_type);

    for (auto prop : properties) {
        prop.second->Write_as_property(dev->add_properties());
    }

    for (auto subsystem : subsystems) {
        subsystem->Register(dev->add_subsystems());
    }
    // LOG("Register msg:%s", msg.SerializeAsString().c_str());
}
