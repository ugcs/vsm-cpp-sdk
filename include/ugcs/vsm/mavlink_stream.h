// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file mavlink_stream.h
 */
#ifndef _UGCS_VSM_MAVLINK_STREAM_H_
#define _UGCS_VSM_MAVLINK_STREAM_H_

#include <ugcs/vsm/io_stream.h>
#include <ugcs/vsm/mavlink_decoder.h>
#include <ugcs/vsm/mavlink_demuxer.h>
#include <ugcs/vsm/mavlink_encoder.h>
#include <tuple>
#include <queue>

namespace ugcs {
namespace vsm {

/** Convenience class for interpreting an I/O stream as a stream of
 * Mavlink messages. It is assumed, that only one such class at a time is
 * used with a given I/O stream.
 */
class Mavlink_stream: public std::enable_shared_from_this<Mavlink_stream>
{
    DEFINE_COMMON_CLASS(Mavlink_stream, Mavlink_stream)

public:
    /** Type of the appropriate Mavlink decoder. */
    typedef Mavlink_decoder Decoder;

    std::string name;

    /** Construct Mavlink stream using a I/O stream. */
    Mavlink_stream(Io_stream::Ref stream) :
        stream(stream), decoder()
    {
        name=stream->Get_name();
        LOG_DBG("MAVLINK_STREAM CREATED %s", name.c_str());
    }

    ~Mavlink_stream(){
        LOG_DBG("MAVLINK_STREAM DESTROYED %s", name.c_str());
    }

    /** Disable copy constructor. */
    Mavlink_stream(const Mavlink_stream&) = delete;

    /** Get underlying I/O stream. */
    Io_stream::Ref&
    Get_stream()
    {
        return stream;
    }

    /** Get underlying decoder. */
    Decoder&
    Get_decoder()
    {
        return decoder;
    }

    /** Get underlying demuxer. */
    Mavlink_demuxer&
    Get_demuxer()
    {
        return demuxer;
    }

    /** Bind decoder and demuxer, so that output of the decoder is automatically
     * passed to the demuxer.
     */
    void
    Bind_decoder_demuxer()
    {
        auto binder = [](Io_buffer::Ptr buffer, mavlink::MESSAGE_ID_TYPE message_id,
                Mavlink_demuxer::System_id system_id, uint8_t component_id, uint32_t request_id,
                Mavlink_stream::Ptr mav_stream)
        {
            mav_stream->demuxer.Demux(buffer, message_id, system_id, component_id, request_id);
        };

        decoder.Register_handler(
                Decoder::Make_decoder_handler(
                        binder, Shared_from_this()));
    }

    /** Toggle mavlink protocol v1/v2 for outgoing messages. */
    void
    Set_mavlink_v2(bool enable = true)
    {
        send_mavlink2 = enable;
    }

    /** Return true if this stream supports mavlink2 */
    bool
    Is_mavlink_v2()
    {
        return send_mavlink2;
    }

    /** Send Mavlink message to other end asynchronously. Timeout should be
     * always present, otherwise there is a chance to overflow the write queue
     * if underlying stream is write-blocked. Only non-temporal completion
     * contexts are allowed. */
    void
    Send_message(
            const mavlink::Payload_base& payload,
            uint8_t system_id,
            uint8_t component_id,
            const std::chrono::milliseconds& timeout,
            Operation_waiter::Timeout_handler timeout_handler,
            const Request_completion_context::Ptr& completion_ctx)
    {
        Send_message(
            payload,
            system_id,
            component_id,
            timeout,
            timeout_handler,
            completion_ctx,
            send_mavlink2);
    }

    /** Send Mavlink message to other end asynchronously. Timeout should be
     * always present, otherwise there is a chance to overflow the write queue
     * if underlying stream is write-blocked. Only non-temporal completion
     * contexts are allowed. */
    void
    Send_message(
            const mavlink::Payload_base& payload,
            uint8_t system_id,
            uint8_t component_id,
            const std::chrono::milliseconds& timeout,
            Operation_waiter::Timeout_handler timeout_handler,
            const Request_completion_context::Ptr& completion_ctx,
            bool mav2)
    {
        ASSERT(completion_ctx->Get_type() != Request_completion_context::Type::TEMPORAL);

        Io_buffer::Ptr buffer;
        if (mav2) {
            buffer = encoder.Encode_v2(payload, system_id, component_id);
        } else {
            buffer = encoder.Encode_v1(payload, system_id, component_id);
        }

        Operation_waiter waiter = stream->Write(
                buffer,
                Make_dummy_callback<void, Io_result>(),
                completion_ctx);
        waiter.Timeout(timeout, timeout_handler, true, completion_ctx);

        write_ops.emplace(std::move(waiter));
        Cleanup_write_ops();
    }

    /** Disable the class. Underlying I/O stream is freed, but not explicitly
     * closed, because this stream could be passed for further processing.
     * Unfinished write operations are aborted.
     */
    void
    Disable()
    {
        if (vehicle_counter > 0) {
            LOG_ERR("DISABLING MAV_STREAM %s WHILE V_COUNTER=%d", name.c_str(), vehicle_counter);
        } else {
            LOG_DBG("DISABLING MAV_STREAM %s", name.c_str());
        }
        decoder.Disable();
        demuxer.Disable();
        stream = nullptr;
        while (!write_ops.empty()) {
            write_ops.front().Abort();
            write_ops.pop();
        }
    }

    void Increment_vehicle_counter() {
        vehicle_counter++;
    }

    void Decrement_vehicle_counter() {
        vehicle_counter--;
    }

    int Get_vehicle_counter() {
        return vehicle_counter;
    }

private:

    int vehicle_counter {0};

    /** Underlying stream. */
    Io_stream::Ref stream;

    /** Decoder used with a stream. */
    Decoder decoder;

    /** Demuxer used with a stream. */
    Mavlink_demuxer demuxer;

    /** Encoder used with a stream. */
    Mavlink_encoder encoder;

    /** Removes completed write operations from the top of the queue. */
    void
    Cleanup_write_ops()
    {
        while (!write_ops.empty()) {
            if (write_ops.front().Is_done()) {
                write_ops.pop();
                continue;
            }
            /* Operations are completed sequentially. */
            break;
        }
    }

    /** Currently scheduled write operations. Some of them could be already
     * finished (i.e. done), so Cleanup_write_ops() should be called periodically
     * to remove them from the queue.
     */
    std::queue<Operation_waiter> write_ops;

    /** Send outgoing traffic in mavlink version 2 format */
    bool send_mavlink2 = false;
};

} /* namespace vsm */
} /* namespace ugcs */

#endif /* _UGCS_VSM_MAVLINK_STREAM_H_ */
