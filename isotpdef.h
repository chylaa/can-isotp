/* Copyright (c) 2024 Michał Chyla
 * You may use, distribute and modify this code
 * under the terms of the MIT license.
 */

#pragma once

#include <array>
#include <cstdint>

namespace CAN {

#define __CAN_MAX_FRAME_SIZE  ((size_t)8)
#define __CANISOTP_MAX_MSG_DATA_SIZE  ((size_t)4095)

// Defines representation of byte in CAN frame
typedef uint8_t BYTE;
// Defines type of CAN identifier. 32-bit value recommended for compatibility with extended frame format.
typedef uint32_t CANID;

enum class Standard : uint8_t
{
    // Indicates usege of specific default/configured format.
    Default = 0x00,
    // CAN base frame format (11bit identifier) CAN 2.0A.
    CBFF = 0x01,
    // CAN extended frame format (29bit identifier) CAN 2.0B.
    CEFF = 0x02 | CBFF,
};
enum class IsoTpState : uint8_t 
{
    // Continue processing frames.
    ISOTP_CONTINUE,
    // Collect received message, finalize state.
    ISOTP_FINALIZE_RX,
    // Message was transmitted in this step.
    ISOTP_FINALIZE_TX,
    // Flow Control frame "Abort" was received.
    ISOTP_FLOW_ABORT,
    // Error condition occured in IsoTp state machine.
    ISOTP_ERROR,
};

struct FrameData
{
    uint8_t size;
    std::array<BYTE, __CAN_MAX_FRAME_SIZE> data;
};

class ICanBusTx 
{
 public:
    virtual bool Transmit(CAN::CANID txid, const CAN::FrameData& data, CAN::Standard standard) = 0;
 protected:
    virtual ~ICanBusTx() = default;
};
}  // namespace CAN