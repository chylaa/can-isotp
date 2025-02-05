/* Copyright (c) 2024 Michał Chyla
 * You may use, distribute and modify this code
 * under the terms of the MIT license.
 */

#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>
#include <functional>

#include "./isotpdef.h"

namespace CAN {

class IsoTp
{
    enum FrameType : uint8_t
    {
        SINGLE = 0,
        FIRST = 1,
        CONSECUTIVE = 2,
        FLOW = 3,
    };
    enum FlowControl : uint8_t
    {
        CONTINUE = 0,
        WAIT = 1,
        ABORT = 2,
    };
    template <typename ST>
    struct FlowControlData
    {
        FlowControl type;
        uint8_t blockSize;
        ST separationTime;
    };

    std::array<BYTE, __CANISOTP_MAX_MSG_DATA_SIZE> _messageBufferRxTx = {};
    size_t _messageBufferSize = 0;

    int _rxCompleteFrameExpectedSize = -1;
    int _rxConsecutiveFrameIndex = -1;
    int _txConsecutiveFrameIndex = -1;

    Standard _defaultCanStandard = Standard::CBFF;  // CAN 2.0A by default
    int _paddingByte = -1;                          // no padding by default

 public:
    enum class MsgStateResult { CONTINUE, FINALIZE_RX, FINALIZE_TX, FLOW_ABORT, ERROR, };

    std::function<void(const char *)> m_funcLogError = nullptr;  // no err logging by default

    explicit IsoTp(Standard defaultStandard)
    {
        static_assert(sizeof(BYTE) == 1, "BYTE must be well... a byte?");
        setDefaulStandard(defaultStandard);
    }
    IsoTp(Standard defaultStandard, BYTE padbyte)
        : IsoTp(defaultStandard)
    {
        setPaddingByte(padbyte);
    }

    IsoTp() = default;
    IsoTp(IsoTp&&) = default;
    IsoTp &operator=(IsoTp&&) = default;

    IsoTp(const IsoTp&) = delete;
    IsoTp &operator=(const IsoTp&) = delete;

    virtual ~IsoTp() = default;

    // Sets default CAN standard (Standard::Default in other methods will be auto-promoted).
    void setDefaulStandard(Standard standard) noexcept;
    // Retreives the default CAN standard.
    Standard getDefaultStandard() const noexcept;
    // Disables padding for ISO-TP messages
    void setNoPadding() noexcept;
    // Sets the padding byte value for ISO-TP messages
    void setPaddingByte(BYTE pad) noexcept;

    // Checks if padding is enabled
    bool IsPaddingEnabled() const noexcept;

    // Checks if IsoTp state machine is currently in receiving state
    bool IsReceiveActive() const;
    // Checks if IsoTp state machine is currently in transmitting state
    bool IsTransmitActive() const;

    // Clears the internal ISO-TP receive state
    void ClearIsotpReceiveState() noexcept;
    // Clears the internal ISO-TP transmit state
    void ClearIsotpTransmitState() noexcept;

    // Gets the maximum "Single-Frame" size for a given Standard. By default uses Standard passed to constructor.
    size_t MaxSingleFrameSize(Standard standard = Standard::Default) const noexcept;

    // Determines the ISO-TP frame type from received data.
    FrameType GetRxFrameType(const FrameData &msgRx) const noexcept;
    
    // Determines the neccessary frame type for a to-be-transmitted frame base on provided data size.
    FrameType GetTxFrameType(size_t size, Standard standard = Standard::Default) const noexcept;

    // Transmits a ISO-TP "Single-Frame", wihout changing the internal state
    MsgStateResult TransmitSingleFrame(ICanBusTx* bus, CANID txid, const FrameData &data) const;
    
    // Transmits the "First-Frame" of an ISO-TP multi-frame message. Remaining bytes will be automatically sent
    // after next received "Flow-Control" is passed to "IsoTp::ProcessIsotpResponse".
    MsgStateResult TransmitFirstFrame(ICanBusTx* bus, CANID txid, const std::vector<BYTE> &fulldata);

    // Transmits data using appropriate method, selected base on vector size and default CAN::Standard
    //  (see "IsoTp::TransmitSingleFrame" and "IsoTp::TransmitFirstFrame").
    MsgStateResult TransmitMessage(ICanBusTx* bus, CANID txid, const std::vector<BYTE> &data);

    // Processes an incoming ISO-TP response. Should be called on each received frame.
    MsgStateResult ProcessIsotpResponse(ICanBusTx* bus, CANID txid, CANID rxid, const FrameData &rxFrame);
    
    // Finalizes the collected ISO-TP response data and stores it in a buffer.
    // Should be called after  IsoTp::MsgStateResult::FINALIZE_RX is returned by "IsoTp::ProcessIsotpResponse".
    // Automatically resets receive state by calling "IsoTp::ClearIsotpReceiveState".
    bool FinalizeIsotpResponse(std::vector<BYTE>* buffer);
    
    // Finalizes the collected ISO-TP response data and stores it in a fixed-size buffer setting *outSize parameter.
    // Should be called after  IsoTp::MsgStateResult::FINALIZE_RX is returned by "IsoTp::ProcessIsotpResponse".
    // Automatically resets receive state by calling "IsoTp::ClearIsotpReceiveState".
    bool FinalizeIsotpResponse(std::array<BYTE, __CANISOTP_MAX_MSG_DATA_SIZE>* buffer, size_t* outSize);

 private:
    // Processes an incoming ISO-TP "Single-Frame" response.
    MsgStateResult ProcessSingleFrameResponse(ICanBusTx* bus, CANID txid, CANID rxid, const FrameData& rxFrame);
    // Processes an incoming ISO-TP "First-Frame" response.
    MsgStateResult ProcessFirstFrameResponse(ICanBusTx* bus, CANID txid, CANID rxid, const FrameData& rxFrame);
    // Processes an incoming ISO-TP "Consecutive-Frame" response.
    MsgStateResult ProcessConsecutiveFrameResponse(ICanBusTx* bus, CANID txid, CANID rxid, const FrameData& rxFrame);
    // Processes an incoming ISO-TP "Flow-Control" response.
    MsgStateResult ProcessFlowControlResponse(ICanBusTx* bus, CANID txid, CANID rxid, const FrameData& rxFrame);

    // Extracts parameters of ISO-TP "Flow-Control" frame.
    // Returns 'true' if passed frame was valid FC, 'false' otherwise.
    bool ExtractFlowControlFrameData(const FrameData &frame, FlowControlData<std::chrono::microseconds>* outFCData) const;
    
    // Extracts index of ISO-TP "Consecutive-Frame".
    // Returns 'true' if passed frame was valid CF, 'false' otherwise.
    bool ExtractIsotpConsecutiveFrameIndex(const FrameData &frame, int* outIndex) const;
    
    // Extracts details of ISO-TP "Single-Frame" or "First-Frame".
    // Returns 'true' if passed frame was valid SF/FF, 'false' otherwise.
    bool ExtractIsotpFirstOrSingleFrameSize(const FrameData &frame, uint32_t* outFullSize) const;

    // Creates new "Flow-Control" frame with provided parameters.
    FrameData NewFlowControlFrame(const FlowControlData<BYTE> &fcData) const;
    
    // Generates next "Consecutive-Frame" for transmission base on current IsoTp state.
    // Returns 'true' if current state was valid ("First-Frame" was sent).
    bool MakeNextConsecutiveFrame(FrameData* frame) noexcept;

    // Gets the number of remaining "Consecutive-Frames" to be transmitted base on current IsoTp state.
    int GetNumOfRemainingTxConsecutiveFrames() const noexcept;
    
    // Sends "Flow-Control" request with provided parameters using "ICanBusTx::Transmit" method.
    MsgStateResult TransmitFlowControl(ICanBusTx* bus, CANID txid, const FlowControlData<BYTE> &fc) const;

    // If "IsoTp::m_funcLogError" is set, logs an error message with optional arguments.
    template <typename... Args>
    void LogErrorInfo(const char* const rmsg, Args... args) const;
    // If "IsoTp::m_funcLogError" is set, logs an error message without additional arguments.
    void LogErrorInfo(const char* const rmsg) const;
};
}  // namespace CAN
