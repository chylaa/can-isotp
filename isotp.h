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
 public:
    // Logging function to be called on error condition (IsoTpState::ISOTP_ERROR). Disabled by default.
    std::function<void(const char *)> m_funcLogError = nullptr; 

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
    IsoTp& operator=(IsoTp&&) = default;

    IsoTp(const IsoTp&) = delete;
    IsoTp& operator=(const IsoTp&) = delete;

    ~IsoTp() = default;

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
    bool IsReceiveActive() const noexcept;
    // Checks if IsoTp state machine is currently in transmitting state
    bool IsTransmitActive() const noexcept;

    // Clears the internal ISO-TP receive state
    void ClearIsotpReceiveState() noexcept;
    // Clears the internal ISO-TP transmit state
    void ClearIsotpTransmitState() noexcept;

    // Transmits data from continuous payload buffer using Single/First frame (base on size and default CAN::Standard).
    // Returns IsoTpState::ISOTP_FINALIZE_TX if SF was transmitted, IsoTpState::ISOTP_CONTINUE if FF was transmitted or
    // IsoTpState::ISOTP_ERROR on any error (e.g. invalid size or "ICanBusTx::Transmit" fail).
    IsoTpState TransmitMessage(ICanBusTx* bus, CANID txid, const BYTE* payload, size_t size);
    // Transmits data from payload vector using Single/First frame (base on size and default CAN::Standard).
    // Returns IsoTpState::ISOTP_FINALIZE_TX if SF was transmitted, IsoTpState::ISOTP_CONTINUE if FF was transmitted or
    // IsoTpState::ISOTP_ERROR on any error (e.g. invalid size or "ICanBusTx::Transmit" fail).
    IsoTpState TransmitMessage(ICanBusTx* bus, CANID txid, const std::vector<BYTE>& payload);
    // Transmits payload data using Single/First frame (base on size and default CAN::Standard).
    // Returns IsoTpState::ISOTP_FINALIZE_TX if SF was transmitted, IsoTpState::ISOTP_CONTINUE if FF was transmitted or
    // IsoTpState::ISOTP_ERROR on any error (e.g. invalid size or "ICanBusTx::Transmit" fail).
    IsoTpState TransmitMessage(ICanBusTx* bus, CANID txid, const FrameData& payload);

    // Processes an incoming ISO-TP response. Should be called on each received frame.
    IsoTpState ProcessIsotpResponse(ICanBusTx* bus, CANID txid, CANID rxid, const FrameData& payload);
    
    // Finalizes the collected ISO-TP response data and stores it in a buffer.
    // Should be called after IsoTpState::FINALIZE_RX is returned by "IsoTp::ProcessIsotpResponse".
    // Automatically resets receive state by calling "IsoTp::ClearIsotpReceiveState".
    // In case of any invalid state buffer will be empty.
    void FinalizeIsotpResponse(std::vector<BYTE>* buffer);
    
    // Finalizes the collected ISO-TP response data and stores it in a fixed-size buffer setting *outSize parameter.
    // Should be called after IsoTpState::FINALIZE_RX is returned by "IsoTp::ProcessIsotpResponse".
    // Automatically resets receive state by calling "IsoTp::ClearIsotpReceiveState".
    // In case of any invalid state size will be 0.
    void FinalizeIsotpResponse(std::array<BYTE, __CANISOTP_MAX_MSG_DATA_SIZE>* buffer, size_t* outSize);

 private:
     enum FrameType : uint8_t;
     enum FlowControl : uint8_t;
     template <typename ST> struct FlowControlData;

     std::array<BYTE, __CANISOTP_MAX_MSG_DATA_SIZE> _messageBufferRxTx = {};
     size_t _messageBufferSize = 0;

     int _rxCompleteFrameExpectedSize = -1;
     int _rxConsecutiveFrameIndex = -1;
     int _txConsecutiveFrameIndex = -1;

     Standard _defaultCanStandard = Standard::CBFF;  // CAN 2.0A by default
     int _paddingByte = -1;                          // no padding by default

    // Gets the maximum "Single-Frame" size for a given Standard. By default uses Standard passed to constructor.
    size_t MaxSingleFrameSize(Standard standard = Standard::Default) const noexcept;

    // Determines the ISO-TP frame type from received data.
    FrameType GetRxFrameType(const FrameData& msgRx) const noexcept;
    // Determines the neccessary frame type for a to-be-transmitted frame base on provided data size.
    FrameType GetTxFrameType(size_t size, Standard standard = Standard::Default) const noexcept;

    // Transmits a ISO-TP "Single-Frame", wihout changing the internal state
    IsoTpState TransmitSingleFrame(ICanBusTx* bus, CANID txid, const FrameData& payload, Standard standard) const;
    // Transmits the "First-Frame" of an ISO-TP multi-frame message using default CAN::Standard. 
    // Remaining bytes will be automatically sent after next received "Flow-Control" is passed to "IsoTp::ProcessIsotpResponse".
    IsoTpState TransmitFirstFrame(ICanBusTx* bus, CANID txid, const BYTE* payload, size_t size);

    // Processes an incoming ISO-TP "Single-Frame" response.
    IsoTpState ProcessSingleFrameResponse(ICanBusTx* bus, CANID txid, CANID rxid, const FrameData& rxFrame);
    // Processes an incoming ISO-TP "First-Frame" response.
    IsoTpState ProcessFirstFrameResponse(ICanBusTx* bus, CANID txid, CANID rxid, const FrameData& rxFrame);
    // Processes an incoming ISO-TP "Consecutive-Frame" response.
    IsoTpState ProcessConsecutiveFrameResponse(ICanBusTx* bus, CANID txid, CANID rxid, const FrameData& rxFrame);
    // Processes an incoming ISO-TP "Flow-Control" response.
    IsoTpState ProcessFlowControlResponse(ICanBusTx* bus, CANID txid, CANID rxid, const FrameData& rxFrame);

    // Extracts parameters of ISO-TP "Flow-Control" frame.
    // Returns 'true' if passed frame was valid FC, 'false' otherwise.
    bool ExtractFlowControlFrameData(const FrameData& frame, FlowControlData<std::chrono::microseconds>* outFCData) const;
    
    // Extracts index of ISO-TP "Consecutive-Frame".
    // Returns 'true' if passed frame was valid CF, 'false' otherwise.
    bool ExtractIsotpConsecutiveFrameIndex(const FrameData& frame, int* outIndex) const;
    
    // Extracts details of ISO-TP "Single-Frame" or "First-Frame".
    // Returns 'true' if passed frame was valid SF/FF, 'false' otherwise.
    bool ExtractIsotpFirstOrSingleFrameSize(const FrameData& frame, size_t* outFullSize) const;

    // Creates new "Flow-Control" frame with provided parameters.
    FrameData NewFlowControlFrame(const FlowControlData<BYTE>& fcData) const;
    
    // Generates next "Consecutive-Frame" for transmission base on current IsoTp state.
    // Returns 'true' if current state was valid ("First-Frame" was sent).
    bool MakeNextConsecutiveFrame(FrameData* frame) noexcept;

    // Gets the number of remaining "Consecutive-Frames" to be transmitted base on current IsoTp state.
    int GetNumOfRemainingTxConsecutiveFrames() const noexcept;
    
    // Sends "Flow-Control" request with provided parameters using "ICanBusTx::Transmit" method.
    IsoTpState TransmitFlowControl(ICanBusTx* bus, CANID txid, const FlowControlData<BYTE>& fc) const;

    // If "IsoTp::m_funcLogError" is set, logs an error message with optional arguments.
    template <typename... Args>
    void LogErrorInfo(const char* const rmsg, Args... args) const;
    // If "IsoTp::m_funcLogError" is set, logs an error message without additional arguments.
    void LogErrorInfo(const char* const rmsg) const;
};
}  // namespace CAN
