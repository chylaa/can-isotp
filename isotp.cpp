/* Copyright (c) 2024 Michał Chyla
 * You may use, distribute and modify this code
 * under the terms of the MIT license.
 */

#include "isotp.h"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <sstream>
#include <thread>

#include "./isotpdef.h"

 // Value to pass as second byte of IsoTp FlowControl frame to signalize to transmitter,
 // that remaining ConsecutiveFrames should be sent without flow control or delay.
#define __CANISOTP_FC_REQ_ALL_REMAINING ((CAN::BYTE)0)

#define __CANISOTP_FC_FRAME_SIZE  ((size_t)3)
#define __CANISOTP_ID11_SF_MAX_DATA_SIZE  ((size_t)7)
#define __CANISOTP_ID29_SF_MAX_DATA_SIZE  ((size_t)6)
#define __CANISOTP_FF_DATA_SIZE  ((size_t)6)
#define __CANISOTP_CF_MAX_DATA_SIZE  ((size_t)7)
#define __CANISOTP_MAX_CF_INDEX  ((uint8_t)15)

using CAN::BYTE;
using CAN::FrameData;
using CAN::IsoTp;
using CAN::IsoTpState;
using CAN::Standard;
using std::chrono::microseconds;

enum IsoTp::FrameType : uint8_t
{
    SINGLE = 0,
    FIRST = 1,
    CONSECUTIVE = 2,
    FLOW = 3,
};
enum IsoTp::FlowControl : uint8_t
{
    CONTINUE = 0,
    WAIT = 1,
    ABORT = 2,
};
template <typename ST>
struct IsoTp::FlowControlData
{
    FlowControl type;
    uint8_t blockSize;
    ST separationTime;
};

void IsoTp::setDefaulStandard(Standard standard) noexcept
{
    if (standard != Standard::Default)
    {
        _defaultCanStandard = standard;
    }
}
Standard IsoTp::getDefaultStandard() const noexcept
{
    return _defaultCanStandard;
}
void IsoTp::setNoPadding() noexcept
{
    _paddingByte = -1;
}
void IsoTp::setPaddingByte(BYTE pad) noexcept
{
    _paddingByte = pad;
}

inline bool IsoTp::IsPaddingEnabled() const noexcept
{
    return _paddingByte >= 0;
}

inline bool IsoTp::IsReceiveActive() const noexcept
{
    return (_rxCompleteFrameExpectedSize != -1);
}
inline bool IsoTp::IsTransmitActive() const noexcept
{
    return (_txConsecutiveFrameIndex != -1);
}

void IsoTp::ClearIsotpReceiveState() noexcept
{
    _messageBufferSize = 0;
    _rxCompleteFrameExpectedSize = -1;
    _rxConsecutiveFrameIndex = -1;
}
void IsoTp::ClearIsotpTransmitState() noexcept
{
    _messageBufferSize = 0;
    _txConsecutiveFrameIndex = -1;
}

IsoTpState CAN::IsoTp::TransmitMessage(ICanBusTx* bus, CANID txid, const BYTE* payload, size_t size)
{
    auto type = GetTxFrameType(size, _defaultCanStandard);

    if (type == FrameType::SINGLE)
    {
        FrameData txFrame{};
        txFrame.size = static_cast<uint8_t>(size);
        std::memcpy(txFrame.data.data(), payload, size);
        return TransmitSingleFrame(bus, txid, txFrame, _defaultCanStandard);
    }
    else if (type == FrameType::FIRST)
    {
        return TransmitFirstFrame(bus, txid, payload, size);
    }
    else
    {
        LogErrorInfo("Cannot send: Unrecognized frame type ", (int)type, '\n');
        return IsoTpState::ISOTP_ERROR;
    }
}

IsoTpState IsoTp::TransmitMessage(ICanBusTx* bus, CANID txid, const std::vector<BYTE>& payload)
{
    return TransmitMessage(bus, txid, payload.data(), payload.size());
}

IsoTpState CAN::IsoTp::TransmitMessage(ICanBusTx* bus, CANID txid, const FrameData& payload)
{
    return TransmitMessage(bus, txid, payload.data.data(), payload.size);
}

IsoTpState IsoTp::ProcessIsotpResponse(ICanBusTx* bus, CANID txid, CANID rxid, const FrameData& rxFrame)
{
    if (rxFrame.size <= 1)  // not a ISOTP frame - assume some status frame; continue
        return IsoTpState::ISOTP_CONTINUE;

    auto ftype = GetRxFrameType(rxFrame);
    IsoTpState result = IsoTpState::ISOTP_ERROR;

    switch (ftype)
    {
    case FrameType::SINGLE:      // RX
        result = ProcessSingleFrameResponse(bus, txid, rxid, rxFrame);
        break;

    case FrameType::FIRST:        // RX
        result = ProcessFirstFrameResponse(bus, txid, rxid, rxFrame);
        break;

    case FrameType::CONSECUTIVE:  // RX
        result = ProcessConsecutiveFrameResponse(bus, txid, rxid, rxFrame);
        break;

    case FrameType::FLOW:         // TX
        result = ProcessFlowControlResponse(bus, txid, rxid, rxFrame);
        break;

    default:
        LogErrorInfo("FrameType=", (int)ftype, " is not ISO-TP frame type!\n");
        break;
    }
    return result;
}

void IsoTp::FinalizeIsotpResponse(std::vector<BYTE>* buffer)
{
    // not checking for any nulls, this kind of bug should fail fast
    if (_rxCompleteFrameExpectedSize > 0)  // sanity check because expected size could be zeroed by user ("ClearIsotpReceiveState()")
    {
        buffer->clear();
        buffer->reserve(_rxCompleteFrameExpectedSize);
        for (size_t i = 0; i < _rxCompleteFrameExpectedSize; ++i)
        {
            buffer->emplace_back(_messageBufferRxTx[i]);
        }
    }
    ClearIsotpReceiveState();
}

void IsoTp::FinalizeIsotpResponse(std::array<BYTE, __CANISOTP_MAX_MSG_DATA_SIZE>* buffer, size_t* outSize)
{
    *outSize = 0; // not checking for any nulls, this kind of bug should fail fast
    if (_rxCompleteFrameExpectedSize > 0)  // sanity check because expected size could be zeroed by user ("ClearIsotpReceiveState()")
    {
        assert(_rxCompleteFrameExpectedSize <= _messageBufferSize);
        *outSize = _rxCompleteFrameExpectedSize;
        for (size_t i = 0; i < _rxCompleteFrameExpectedSize; ++i)
        {
            (*buffer)[i] = _messageBufferRxTx[i];
        }
    }
    ClearIsotpReceiveState();
}

#pragma region Privates

size_t IsoTp::MaxSingleFrameSize(Standard standard) const noexcept
{
    if (standard == Standard::Default)
        standard = _defaultCanStandard;  // promote to default standard
    if (standard >= Standard::CEFF)
        return __CANISOTP_ID29_SF_MAX_DATA_SIZE;  // CAN 2.0B (29bit id)
    else
        return __CANISOTP_ID11_SF_MAX_DATA_SIZE;  // assume CAN 2.0A (11bit id)
}

inline IsoTp::FrameType IsoTp::GetRxFrameType(const FrameData& msgRx) const noexcept
{
    return static_cast<FrameType>((msgRx.data[0] & 0b11110000) >> 4);
}
inline IsoTp::FrameType IsoTp::GetTxFrameType(size_t size, Standard standard) const noexcept
{
    return size > MaxSingleFrameSize(standard) ? FrameType::FIRST : FrameType::SINGLE;
}

IsoTpState IsoTp::TransmitFirstFrame(ICanBusTx* bus, CANID txid, const BYTE* fulldata, size_t size)
{
    if (size < __CANISOTP_FF_DATA_SIZE || __CANISOTP_MAX_MSG_DATA_SIZE < size)
    {
        LogErrorInfo("Invalid data size for isotp First-Frame: ", size, '\n');
        return IsoTpState::ISOTP_ERROR;
    }
    if (IsReceiveActive())
    {
        LogErrorInfo("Invalid state: isotp RX active; finalize receive!\n");
        return IsoTpState::ISOTP_ERROR;
    }
    ClearIsotpTransmitState();  // sanity call
    std::memcpy(_messageBufferRxTx.data(), fulldata, size);
    _messageBufferSize = size;
    _txConsecutiveFrameIndex = 0;

    FrameData ff{};
    ff.size = __CAN_MAX_FRAME_SIZE;  // first frame never padded
    ff.data[0] = (BYTE)((BYTE)(FrameType::FIRST << 4) | ((size >> 8) & 0xFF));
    ff.data[1] = (BYTE)(size & 0xFF);

    std::memcpy(&ff.data[2], _messageBufferRxTx.data(), __CANISOTP_FF_DATA_SIZE);

    if (bus->Transmit(txid, ff, _defaultCanStandard))
    {
        return IsoTpState::ISOTP_CONTINUE;
    }
    LogErrorInfo("ICanBusTx::Transmit failed to send FF to ", txid);
    ClearIsotpTransmitState();
    return IsoTpState::ISOTP_ERROR;
}

IsoTpState IsoTp::TransmitSingleFrame(ICanBusTx* bus, CANID txid, const FrameData &payload, Standard standard) const
{
    if (standard == Standard::Default)
    {
        standard = _defaultCanStandard;
    }
    const auto max = MaxSingleFrameSize(standard);
    if (payload.size > max)
    {
        LogErrorInfo("'SendSingleFame' method supprots only DLC <= ", max, " (passed ", (int)payload.size, " bytes).\n");
        return IsoTpState::ISOTP_ERROR;
    }
    FrameData sf{};
    sf.size = payload.size;
    if (sf.size) // we'll allow Transmit implementation to decide what to do with zero-length frames 
    {
        // In fact FrameType::Single << 4 is unnecessary, always 0, assuming compiler will optimize it out anyway
        sf.data[0] = (((BYTE)FrameType::SINGLE) << 4) | sf.size;
        std::memcpy(&(sf.data[1]), payload.data.data(), sf.size);
        sf.size += 1;
        if (IsPaddingEnabled())
        {
            auto beginIt = std::begin(sf.data) + sf.size;
            std::fill(beginIt, std::end(sf.data), static_cast<BYTE>(_paddingByte));
            sf.size = __CAN_MAX_FRAME_SIZE;
        }
    }
    if (bus->Transmit(txid, sf, standard))
    {
        return IsoTpState::ISOTP_FINALIZE_TX;
    }
    LogErrorInfo("ICanBusTx::Transmit failed to send SF to ", txid);
    return IsoTpState::ISOTP_ERROR;
}

IsoTpState IsoTp::ProcessSingleFrameResponse(ICanBusTx* bus, CANID txid, CANID rxid, const FrameData& rxFrame)
{
    IsoTpState result;
    size_t outFullSize;
    
    ClearIsotpReceiveState(); // sanity call
    if (ExtractIsotpFirstOrSingleFrameSize(rxFrame, &outFullSize))
    {
        const size_t dataoffset = 1;
        const auto& rxData = rxFrame.data;
        _rxCompleteFrameExpectedSize = outFullSize;
        std::memcpy(_messageBufferRxTx.data(), &rxData[dataoffset], outFullSize);
        _messageBufferSize = outFullSize;
        result = IsoTpState::ISOTP_FINALIZE_RX;
    }
    else
    {
        LogErrorInfo("Failed to extract single frame data!\n");
        result = IsoTpState::ISOTP_ERROR;
    }
    return result;
}

IsoTpState IsoTp::ProcessFirstFrameResponse(ICanBusTx* bus, CANID txid, CANID rxid, const FrameData& rxFrame)
{
    IsoTpState result;
    size_t outFullSize;

    ClearIsotpReceiveState();
    if (ExtractIsotpFirstOrSingleFrameSize(rxFrame, &outFullSize))
    {
        const size_t dataoffset = 2;
        const size_t datasize = rxFrame.size - dataoffset;
        const auto& rxData = rxFrame.data;

        _rxCompleteFrameExpectedSize = outFullSize;
        _rxConsecutiveFrameIndex = 0;
        std::memcpy(_messageBufferRxTx.data(), &rxData[dataoffset], datasize);
        _messageBufferSize = datasize;
        // Note: Implemnetation always requests all remaining frames wihout delay:
        // TODO: Introduce a way to select if 0 or "totalNumberOfRemaining" should be send?
        result = TransmitFlowControl(bus, txid, { FlowControl::CONTINUE, __CANISOTP_FC_REQ_ALL_REMAINING, 0 });
    }
    else
    {
        TransmitFlowControl(bus, txid, { FlowControl::ABORT, 0, 0 });
        LogErrorInfo("Failed to extract first frame data!\n");
        result = IsoTpState::ISOTP_ERROR;
    }
    return result;
}

IsoTpState IsoTp::ProcessConsecutiveFrameResponse(ICanBusTx* bus, CANID txid, CANID rxid, const FrameData& rxFrame)
{
    if (_rxConsecutiveFrameIndex < 0 || _rxCompleteFrameExpectedSize < 0)
    {
        LogErrorInfo("Invalid State: no First-Frame before this Consecutive-Frame!\n");
        return IsoTpState::ISOTP_ERROR;
    }

    ++_rxConsecutiveFrameIndex;  // expected CF index
    _rxConsecutiveFrameIndex %= (__CANISOTP_MAX_CF_INDEX + 1);

    IsoTpState result;
    int outIndexCF = -1;
    const auto& rxData = rxFrame.data;

    if (ExtractIsotpConsecutiveFrameIndex(rxFrame, &outIndexCF))
    {
        const size_t dataoffset = 1;
        const size_t datasize = rxFrame.size - dataoffset;

        auto dest = &(_messageBufferRxTx.data()[_messageBufferSize]);
        std::memcpy(dest, &rxData[dataoffset], datasize);
        _messageBufferSize += datasize;

        if (outIndexCF != _rxConsecutiveFrameIndex) 
        {
            LogErrorInfo("Unexpected Consecutive Frame index! Got ", outIndexCF, " expected ", _rxConsecutiveFrameIndex);
            TransmitFlowControl(bus, txid, { FlowControl::ABORT, 0, 0 });
            ClearIsotpReceiveState();
            result = IsoTpState::ISOTP_ERROR;
        }
        else if ((_messageBufferSize >= _rxCompleteFrameExpectedSize)) 
        {
            result = IsoTpState::ISOTP_FINALIZE_RX;
        }
        else 
        {
            result = IsoTpState::ISOTP_CONTINUE;
        }
    }
    else
    {
        LogErrorInfo("Failed to extract first frame data!\n");
        result = IsoTpState::ISOTP_ERROR;
    }
    return result;
}

IsoTpState IsoTp::ProcessFlowControlResponse(ICanBusTx* bus, CANID txid, CANID rxid, const FrameData& rxFrame)
{
    FlowControlData<microseconds> fcData;
    IsoTpState result = IsoTpState::ISOTP_ERROR;
    if (ExtractFlowControlFrameData(rxFrame, &fcData))
    {
        if (false == IsTransmitActive())
        {
            LogErrorInfo("Unexpected flow control frame: ISOTP transmit uninitialized!\n");
            return IsoTpState::ISOTP_ERROR;
        }
        if (fcData.type == FlowControl::WAIT)
        {
            return IsoTpState::ISOTP_CONTINUE;;
        }
        if (fcData.type == FlowControl::ABORT)
        {
            ClearIsotpReceiveState();
            return IsoTpState::ISOTP_FLOW_ABORT;
        }
        int numOfRemainingTxCF = GetNumOfRemainingTxConsecutiveFrames();
        if (numOfRemainingTxCF < 0)
        {
            LogErrorInfo("Unexpected isotp state: ", numOfRemainingTxCF, " frames to send!\n");
            return IsoTpState::ISOTP_ERROR;
        }

        int iblockSize = static_cast<int>(fcData.blockSize);
        if (iblockSize == 0)
        {
            iblockSize = numOfRemainingTxCF;
        }
        int numToSendBeforeNextCF = std::min(numOfRemainingTxCF, iblockSize);
        for (int i = 0; i < numToSendBeforeNextCF; i++)
        {
            FrameData nextCF;
            if (MakeNextConsecutiveFrame(&nextCF))
            {
                if (false == bus->Transmit(txid, nextCF, _defaultCanStandard))
                {
                    LogErrorInfo("ICanTxBus::Transmit failed on CF no. ", i);
                    result = IsoTpState::ISOTP_ERROR;
                    break;
                }
                // if requested introduce SOME delay
                if (fcData.separationTime.count() > 0)
                {   // not precise but good enough, hardly used anyway.
                    // ST indicates a minimum delay anyways (we'll take longer)
                    std::this_thread::sleep_for(fcData.separationTime);
                }
                result = IsoTpState::ISOTP_CONTINUE;
            }
            else
            {
                TransmitFlowControl(bus, txid, { FlowControl::ABORT, 0, 0 });
                LogErrorInfo("Failed to create next consecutive frame!\n");
                result = IsoTpState::ISOTP_ERROR;
                break;
            }
        }

        if (GetNumOfRemainingTxConsecutiveFrames() == 0)
        {
            ClearIsotpTransmitState();
            result = IsoTpState::ISOTP_FINALIZE_TX;
        }
        else if (result == IsoTpState::ISOTP_ERROR)
        {
            ClearIsotpTransmitState();
        }
    }
    else
    {
        LogErrorInfo("Failed to extract flow control frame data!\n");
        result = IsoTpState::ISOTP_ERROR;
    }
    return result;
}


bool IsoTp::ExtractFlowControlFrameData(const FrameData& frame, FlowControlData<microseconds>* outFCData) const
{
    if (frame.size < __CANISOTP_FC_FRAME_SIZE)
    {
        LogErrorInfo("Not Flow Control frame! DLC = ", frame.size, '\n');
        return false;
    }
    outFCData->type = static_cast<FlowControl>(frame.data[0] & 0b1111);
    outFCData->blockSize = frame.data[1];
    
    if (outFCData->blockSize == __CANISOTP_FC_REQ_ALL_REMAINING)
        // ST in miliseconds, between 0x01 - 0x7F, ignore msb
        outFCData->separationTime = std::chrono::milliseconds(frame.data[2] & 0x7F);
    else
        // ST in microseconds, between 0xF1 (100) - 0xF9 (900)
        outFCData->separationTime = std::chrono::microseconds((100 * ((frame.data[2] & 0x0F) % 0x0A)));
    
    return true;
}

bool IsoTp::ExtractIsotpConsecutiveFrameIndex(const FrameData& frame, int* outIndex) const
{
    if (GetRxFrameType(frame) == FrameType::CONSECUTIVE)
    {
        *outIndex = (frame.data[0] & 0b1111);
        return true;
    }
    *outIndex = -1;
    return false;
}
bool IsoTp::ExtractIsotpFirstOrSingleFrameSize(const FrameData& frame, size_t* outFullSize) const
{
    auto& fdata = frame.data;
    if ((fdata[0] == 0) || (fdata[0] == 0x10 && fdata[1] == 0x00)) // is SF_FD or FF_FD?
    {
        LogErrorInfo("Receive failed: CAN FD not supported!\n");
        return false;
    }
    if (frame.size < 2)
    {
        LogErrorInfo("Receive failed: unexpected frame lenght!\n");
        return false;
    }
    const auto ftype = GetRxFrameType(frame);
    if (ftype == FrameType::SINGLE)
    {
        *outFullSize = static_cast<size_t>(frame.data[0] & 0b1111);
        return true;
    }
    if (ftype != FrameType::FIRST)
    {
        LogErrorInfo("Received message is not expected ISOTP First-Frame!\n");
        return false;
    }
    *outFullSize = static_cast<size_t>(((fdata[0] & 0b1111) << 8) | fdata[1]);
    if (*outFullSize < 8)
    {
        LogErrorInfo("First-Frame error: declared full size less than minimum 8 bytes!\n");
        return false;
    }
    return true;
}

FrameData IsoTp::NewFlowControlFrame(const FlowControlData<BYTE>& fcData) const
{
    FrameData cf{};
    const size_t cfsize = __CANISOTP_FC_FRAME_SIZE;
    cf.data[0] = (BYTE)(0x30 | fcData.type);
    cf.data[1] = (BYTE)(fcData.blockSize);
    cf.data[2] = fcData.separationTime;

    if (IsPaddingEnabled())
    {
        const auto pad = static_cast<BYTE>(_paddingByte);
        std::fill(std::begin(cf.data) + cfsize, std::end(cf.data), pad);
        cf.size = std::size(cf.data);
    }
    else
    {
        cf.size = cfsize;
    }
    return cf;
}

int IsoTp::GetNumOfRemainingTxConsecutiveFrames() const noexcept
{
    long long remainingBytes = (_messageBufferSize - __CANISOTP_FF_DATA_SIZE);
    remainingBytes -= (_txConsecutiveFrameIndex * __CANISOTP_CF_MAX_DATA_SIZE);
    if (remainingBytes <= 0)
        return 0;

    int numOfConsevutiveFrames = (remainingBytes / __CANISOTP_CF_MAX_DATA_SIZE);
    numOfConsevutiveFrames += ((remainingBytes % __CANISOTP_CF_MAX_DATA_SIZE == 0) ? 0 : 1);
    return numOfConsevutiveFrames;
}

IsoTpState IsoTp::TransmitFlowControl(ICanBusTx* bus, CANID txid, const FlowControlData<BYTE>& fc) const
{
#ifdef _DEBUG
    if (fc.blockSize == __CANISOTP_FC_REQ_ALL_REMAINING && (fc.separationTime > 0x7F))
        throw std::exception("If blockSize == 0, ST must be in range of: 0 <= ST <= 127\n");
#endif
    if (bus->Transmit(txid, NewFlowControlFrame(fc), _defaultCanStandard))
        return IsoTpState::ISOTP_CONTINUE;

    LogErrorInfo("ICanBusTx::Transmit failed while transmitting FC frame ", fc.type, " to ", txid);
    return IsoTpState::ISOTP_ERROR;
}

bool IsoTp::MakeNextConsecutiveFrame(FrameData *frame) noexcept
{
    int i = __CANISOTP_FF_DATA_SIZE + (_txConsecutiveFrameIndex * __CANISOTP_CF_MAX_DATA_SIZE);
    if (i > _messageBufferSize)
    {
        frame->size = 0;
        LogErrorInfo("Calulated CF frame byte index ", i, " > ", _messageBufferSize, " (msg buffer size)\n");
        return false;
    }
    const int index = ++_txConsecutiveFrameIndex;
    const auto datasize = std::min(_messageBufferSize - i, __CANISOTP_CF_MAX_DATA_SIZE);

    frame->data[0] = (BYTE)((FrameType::CONSECUTIVE << 4) | (index % (__CANISOTP_MAX_CF_INDEX + 1)));
    std::memcpy(&(frame->data[1]), &_messageBufferRxTx[i], datasize);

    const auto frameSize = (1 + datasize);
    if (IsPaddingEnabled() && datasize < __CANISOTP_CF_MAX_DATA_SIZE)
    {
        std::fill(std::begin(frame->data) + frameSize, std::end(frame->data), static_cast<BYTE>(_paddingByte));
        frame->size = __CAN_MAX_FRAME_SIZE;
    }
    else
    {
        frame->size = static_cast<BYTE>(frameSize);
    }
    return true;
}

void IsoTp::LogErrorInfo(const char* const rmsg) const
{
    if (m_funcLogError)
    {
        m_funcLogError("[ISOTP ERROR]: ");
        m_funcLogError(rmsg);
    }
}

#if ((defined(_MSVC_LANG) && _MSVC_LANG >= 201703L) || __cplusplus >= 201703L) // C++17
template <typename... Args>
void IsoTp::LogErrorInfo(const char *const rmsg, Args... args) const
{
    if (m_funcLogError)
    {
        std::ostringstream s;
        s << "[ISOTP ERROR]: " << rmsg;
        ((s << args), ...);  // Fold expression (since C++17)
        m_funcLogError(s.str().c_str());
    }
}
#else

void static AppendToStream(std::ostringstream&) {}

template <typename T, typename... Args>
void static AppendToStream(std::ostringstream& s, T value, Args... args)
{
    s << value;
    AppendToStream(s, args...);
}

template <typename... Args>
void IsoTp::LogErrorInfo(const char* const rmsg, Args... args) const 
{
    if (m_funcLogError) 
    {
        std::ostringstream s;
        s << "[ISOTP ERROR]: " << rmsg;
        AppendToStream(s, args...);
        m_funcLogError(s.str().c_str());
    }
}

#endif
#pragma endregion
