/* Copyright (c) 2024 Michał Chyla
 * You may use, distribute and modify this code
 * under the terms of the MIT license.
 */

#pragma once

#include <array>
#include <cstdint>

namespace CAN {

#define __CAN_MAX_FRAME_SIZE  ((size_t)8)

// Value to pass as second byte of IsoTp FlowControl frame to signalize to transmitter,
// that remaining ConsecutiveFrames should be sent without flow control or delay.
#define __CANISOTP_FC_REQ_ALL_REMAINING ((CAN::BYTE)0)

#define __CANISOTP_FC_FRAME_SIZE  ((size_t)3)
#define __CANISOTP_ID11_SF_MAX_DATA_SIZE  ((size_t)7)
#define __CANISOTP_ID29_SF_MAX_DATA_SIZE  ((size_t)6)
#define __CANISOTP_FF_DATA_SIZE  ((size_t)6)
#define __CANISOTP_CF_MAX_DATA_SIZE  ((size_t)7)
#define __CANISOTP_MAX_CF_INDEX  ((uint8_t)15)
#define __CANISOTP_MAX_MSG_DATA_SIZE  ((size_t)4095)
/* FUTURE SUPPORT ?
#define __CAN_FD_MAX_FRAME_SIZE  ((size_t)64)
#define __CANISOTP_FD_FF_DATA_SIZE  ((size_t)58)
#define __CANISOTP_FD_CF_MAX_DATA_SIZE  ((size_t)63)
*/

// Defines representation of byte in CAN frame
typedef std::uint8_t BYTE;
// Defines type of CAN identifier. 32-bit value recommended for compatibility with extended frame format.
typedef std::uint32_t CANID;

enum Standard : BYTE
{
    /* Indicates usege of implementation specific default/configured format. */
    Default = 0x00,
    /* CANN CC base frame format (11bit identifier) CAN 2.0A. */
    CBFF = 0x01,
    /* CANN CC extended frame format (29bit identifier) CAN 2.0B. Compatibile with "CBFF".*/
    CEFF = 0x02 | CBFF,
    // NOT SUPPROTED // CFDBFF = 0x04 | CBFF, // 5
    // NOT SUPPROTED // CFDEFF = CFDBFF | CEFF, // 7
};

struct FrameData 
{
    std::uint8_t size;
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