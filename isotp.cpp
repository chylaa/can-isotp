/* Copyright (c) 2024 Michał Chyla
 * You may use, distribute and modify this code
 * under the terms of the MIT license.
 */

#include "isotp.h"

#include <algorithm>
#include <cstring>
#include <sstream>
#include <thread>

#include "./isotpdef.h"

using CAN::IsoTp;
using CAN::Standard;
using CAN::FrameData;
using CAN::BYTE;
using std::chrono::microseconds;

#pragma region Padding static prototypes
static size_t GetSizeWithoutPaddingByte(CAN::FrameData frame, int padding);
static size_t GetSizeWithoutPaddingByte(const std::vector<BYTE> &data, int padding);
static size_t GetSizeWithoutPaddingByte(const BYTE* begin, size_t datasize, int padding);
#pragma endregion

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

inline bool IsoTp::IsReceiveActive() const
{
    return (_rxCompleteFrameExpectedSize != -1);
}
inline bool IsoTp::IsTransmitActive() const
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

size_t IsoTp::MaxSingleFrameSize(Standard standard) const noexcept
{
    if (standard == Standard::Default)
        standard = _defaultCanStandard;  // promote to default standard
    if (standard >= Standard::CEFF)
        return __CANISOTP_ID29_SF_MAX_DATA_SIZE;  // CAN 2.0B (29bit id)
    else
        return __CANISOTP_ID11_SF_MAX_DATA_SIZE;  // assume CAN 2.0A (11bit id)
}

inline IsoTp::FrameType IsoTp::GetRxFrameType(const FrameData &msgRx) const noexcept
{
    return static_cast<FrameType>((msgRx.data[0] & 0b11110000) >> 4);
}
inline IsoTp::FrameType IsoTp::GetTxFrameType(size_t size, Standard standard) const noexcept
{
    return size > MaxSingleFrameSize(standard) ? FrameType::FIRST : FrameType::SINGLE;
}

IsoTp::MsgStateResult IsoTp::TransmitFirstFrame(ICanBusTx* bus, CANID txid, const std::vector<BYTE> &fulldata)
{
    auto fulldatasize = fulldata.size();
    if (fulldatasize < __CANISOTP_FF_DATA_SIZE || __CANISOTP_MAX_MSG_DATA_SIZE < fulldatasize)
    {
        LogErrorInfo("Invalid data size for isotp First-Frame: ", fulldatasize, '\n');
        return MsgStateResult::ERROR;
    }
    if (IsReceiveActive())
    {
        LogErrorInfo("Invalid state: isotp RX active; finalize receive!\n");
        return MsgStateResult::ERROR;
    }
    ClearIsotpTransmitState();  // sanity call
    std::memcpy(_messageBufferRxTx.data(), fulldata.data(), fulldatasize);
    _messageBufferSize = fulldatasize;
    _txConsecutiveFrameIndex = 0;

    FrameData ff{};
    ff.size = __CAN_MAX_FRAME_SIZE;  // first frame never padded
    ff.data[0] = (BYTE)((FrameType::FIRST << 4) | (BYTE)((fulldatasize >> 8) & 0xFF));
    ff.data[1] = (BYTE)(fulldatasize & 0xFF);

    std::memcpy(&ff.data[2], _messageBufferRxTx.data(), __CANISOTP_FF_DATA_SIZE);

    if (bus->Transmit(txid, ff))
    {
        return MsgStateResult::CONTINUE;
    }
    LogErrorInfo("ICanBusTx::Transmit failed to send FF to ", txid);
    ClearIsotpTransmitState();
    return MsgStateResult::ERROR;
}

IsoTp::MsgStateResult IsoTp::TransmitSingleFrame(ICanBusTx* bus, CANID txid, const FrameData &data) const
{
    const auto max = MaxSingleFrameSize(_defaultCanStandard);
    if (data.size > max)
    {
        LogErrorInfo("'SendSingleFame' method supprots only DLC <= ", max, " (passed ", (int)data.size, " bytes).\n");
        return MsgStateResult::ERROR;
    }
    FrameData sf{};
    const BYTE datasize = data.size;
    // In fact FrameType::Single << 4 is unnecessary, always 0, assuming compiler will optimize it out anyway
    sf.data[0] = (((BYTE)FrameType::SINGLE) << 4) | datasize;
    std::memcpy(&(sf.data[1]), data.data.data(), datasize);

    if (IsPaddingEnabled())
    {
        std::fill(std::begin(sf.data) + 1, std::end(sf.data), static_cast<BYTE>(_paddingByte));
        sf.size = __CAN_MAX_FRAME_SIZE;
    }
    else
    {
        sf.size = (1 + datasize);
    }
    if (bus->Transmit(txid, sf))
    {
        return MsgStateResult::FINALIZE_TX;
    }
    LogErrorInfo("ICanBusTx::Transmit failed to send SF to ", txid);
    return MsgStateResult::ERROR;
}

IsoTp::MsgStateResult IsoTp::TransmitMessage(ICanBusTx* bus, CANID txid, const std::vector<BYTE> &msgdata)
{
    auto datasize = msgdata.size();
    auto type = GetTxFrameType(datasize, _defaultCanStandard);

    if (type == FrameType::SINGLE)
    {
        FrameData txFrame{};
        txFrame.size = static_cast<uint8_t>(datasize);
        std::memcpy(txFrame.data.data(), msgdata.data(), datasize);
        return TransmitSingleFrame(bus, txid, txFrame);
    }
    else if (type == FrameType::FIRST)
    {
        return TransmitFirstFrame(bus, txid, msgdata);
    }
    else
    {
        LogErrorInfo("Cannot send: Unrecognized frame type ", (int)type, '\n');
        return MsgStateResult::ERROR;
    }
}

IsoTp::MsgStateResult IsoTp::ProcessIsotpResponse(ICanBusTx* bus, CANID txid, CANID rxid, const FrameData &rxFrame)
{
    if (rxFrame.size <= 1)  // not a ISOTP frame - assume some status frame; continue
        return MsgStateResult::CONTINUE;

    auto ftype = GetRxFrameType(rxFrame);
    MsgStateResult result = MsgStateResult::ERROR;

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

bool IsoTp::FinalizeIsotpResponse(std::vector<BYTE>* buffer)
{
    bool sizeOk = true;
    if (buffer)
    {
        buffer->clear();
        size_t sizeNoPadding = GetSizeWithoutPaddingByte(_messageBufferRxTx.data(), _messageBufferSize, _paddingByte);
        for (size_t i = 0; i < sizeNoPadding; ++i)
        {
            buffer->push_back(_messageBufferRxTx[i]);
        }
        sizeOk = (_rxCompleteFrameExpectedSize == sizeNoPadding);
    }
    ClearIsotpReceiveState();
    return sizeOk;
}

bool IsoTp::FinalizeIsotpResponse(std::array<BYTE, __CANISOTP_MAX_MSG_DATA_SIZE>* buffer, size_t* outSize)
{
    bool sizeOk = true;
    *outSize = 0;
    if (buffer)
    {
        size_t sizeval = GetSizeWithoutPaddingByte(_messageBufferRxTx.data(), _messageBufferSize, _paddingByte);
        *outSize = sizeval;
        for (size_t i = 0; i < sizeval; ++i)
        {
            (*buffer)[i] = _messageBufferRxTx[i];
        }
        sizeOk = (_rxCompleteFrameExpectedSize == sizeval);
    }
    ClearIsotpReceiveState();
    return sizeOk;
}

#pragma region Privates


IsoTp::MsgStateResult IsoTp::ProcessSingleFrameResponse(ICanBusTx* bus, CANID txid, CANID rxid, const FrameData& rxFrame)
{
    MsgStateResult result;
    uint32_t outFullSize;
    
    ClearIsotpReceiveState(); // sanity call
    if (ExtractIsotpFirstOrSingleFrameSize(rxFrame, &outFullSize))
    {
        const size_t dataoffset = 1;
        const auto& rxData = rxFrame.data;
        _rxCompleteFrameExpectedSize = outFullSize;
        std::memcpy(_messageBufferRxTx.data(), &rxData[dataoffset], outFullSize);
        _messageBufferSize = outFullSize;
        result = MsgStateResult::FINALIZE_RX;
    }
    else
    {
        LogErrorInfo("Failed to extract single frame data!\n");
        result = MsgStateResult::ERROR;
    }
    return result;
}

IsoTp::MsgStateResult IsoTp::ProcessFirstFrameResponse(ICanBusTx* bus, CANID txid, CANID rxid, const FrameData& rxFrame)
{
    MsgStateResult result;
    uint32_t outFullSize;

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
        result = TransmitFlowControl(bus, txid, { FlowControl::CONTINUE, __CANISOTP_FC_REQ_ALL_REMAINING, 0 });
    }
    else
    {
        TransmitFlowControl(bus, txid, { FlowControl::ABORT, 0, 0 });
        LogErrorInfo("Failed to extract first frame data!\n");
        result = MsgStateResult::ERROR;
    }
    return result;
}

IsoTp::MsgStateResult IsoTp::ProcessConsecutiveFrameResponse(ICanBusTx* bus, CANID txid, CANID rxid, const FrameData& rxFrame)
{
    if (_rxConsecutiveFrameIndex < 0 || _rxCompleteFrameExpectedSize < 0)
    {
        LogErrorInfo("Invalid State: no First-Frame before this Consecutive-Frame!\n");
        return MsgStateResult::ERROR;
    }

    ++_rxConsecutiveFrameIndex;  // expected CF index
    _rxConsecutiveFrameIndex %= (__CANISOTP_MAX_CF_INDEX + 1);

    MsgStateResult result;
    int outIndexCF = -1;
    const auto& rxData = rxFrame.data;

    if (ExtractIsotpConsecutiveFrameIndex(rxFrame, &outIndexCF))
    {
        const size_t dataoffset = 1;
        const size_t datasize = rxFrame.size - dataoffset;

        auto dest = &(_messageBufferRxTx.data()[_messageBufferSize]);
        std::memcpy(dest, &rxData[dataoffset], datasize);
        _messageBufferSize += datasize;

        if ((outIndexCF != _rxConsecutiveFrameIndex) || (_messageBufferSize >= _rxCompleteFrameExpectedSize))
            result = MsgStateResult::FINALIZE_RX;
        else
            result = MsgStateResult::CONTINUE;
    }
    else
    {
        LogErrorInfo("Failed to extract first frame data!\n");
        result = MsgStateResult::ERROR;
    }
    return result;
}

IsoTp::MsgStateResult IsoTp::ProcessFlowControlResponse(ICanBusTx* bus, CANID txid, CANID rxid, const FrameData& rxFrame)
{
    FlowControlData<microseconds> fcData;
    MsgStateResult result = MsgStateResult::ERROR;
    if (ExtractFlowControlFrameData(rxFrame, &fcData))
    {
        if (false == IsTransmitActive())
        {
            LogErrorInfo("Unexpected flow control frame: ISOTP transmit uninitialized!\n");
            return MsgStateResult::ERROR;
        }
        if (fcData.type == FlowControl::WAIT)
        {
            return MsgStateResult::CONTINUE;;
        }
        if (fcData.type == FlowControl::ABORT)
        {
            ClearIsotpReceiveState();
            return MsgStateResult::FLOW_ABORT;
        }
        int numOfRemainingTxCF = GetNumOfRemainingTxConsecutiveFrames();
        if (numOfRemainingTxCF < 0)
        {
            LogErrorInfo("Unexpected isotp state: ", numOfRemainingTxCF, " frames to send!\n");
            return MsgStateResult::ERROR;
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
                if (false == bus->Transmit(txid, nextCF))
                {
                    LogErrorInfo("ICanTxBus::Transmit failed on CF no. ", i);
                    result = MsgStateResult::ERROR;
                    break;
                }
                // if requested introduce SOME delay
                if (fcData.separationTime.count() > 0)
                {   // not precise but good enough, hardly used anyway.
                    // ST indicates a minimum delay anyways (we'll take longer)
                    std::this_thread::sleep_for(fcData.separationTime);
                }
                result = MsgStateResult::CONTINUE;
            }
            else
            {
                TransmitFlowControl(bus, txid, { FlowControl::ABORT, 0, 0 });
                LogErrorInfo("Failed to create next consecutive frame!\n");
                result = MsgStateResult::ERROR;
                break;
            }
        }

        if (GetNumOfRemainingTxConsecutiveFrames() == 0)
        {
            ClearIsotpTransmitState();
            result = MsgStateResult::FINALIZE_TX;
        }
        else if (result == MsgStateResult::ERROR)
        {
            ClearIsotpTransmitState();
        }
    }
    else
    {
        LogErrorInfo("Failed to extract flow control frame data!\n");
        result = MsgStateResult::ERROR;
    }
    return result;
}


bool IsoTp::ExtractFlowControlFrameData(const FrameData& frame, FlowControlData<microseconds>* outFCData) const
{
    size_t size = GetSizeWithoutPaddingByte(frame, _paddingByte);
    if (size != __CANISOTP_FC_FRAME_SIZE)
    {
        LogErrorInfo("Not Flow Control frame! DLC = ", size, '\n');
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
bool IsoTp::ExtractIsotpFirstOrSingleFrameSize(const FrameData& frame, uint32_t* outFullSize) const
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
        *outFullSize = static_cast<uint16_t>(frame.data[0] & 0b1111);
        return true;
    }
    if (ftype != FrameType::FIRST)
    {
        LogErrorInfo("Received message is not expected ISOTP First-Frame!\n");
        return false;
    }
    *outFullSize = static_cast<uint16_t>(((fdata[0] & 0b1111) << 8) | fdata[1]);
    if (*outFullSize < 8)
    {
        LogErrorInfo("First-Frame error: declared full size less than minimum 8 bytes!\n");
        return false;
    }
    return true;
}

FrameData IsoTp::NewFlowControlFrame(const FlowControlData<BYTE> &fcData) const
{
    FrameData cf{};
    const size_t cfsize = __CANISOTP_FC_FRAME_SIZE;
    cf.data[0] = static_cast<BYTE>(0x30 | fcData.type);
    cf.data[1] = static_cast<BYTE>(fcData.blockSize);
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

    int numOfConsevutiveFrames = static_cast<int>(remainingBytes / __CANISOTP_CF_MAX_DATA_SIZE);
    numOfConsevutiveFrames += ((remainingBytes % __CANISOTP_CF_MAX_DATA_SIZE == 0) ? 0 : 1);
    return numOfConsevutiveFrames;
}

IsoTp::MsgStateResult IsoTp::TransmitFlowControl(ICanBusTx* bus, CANID txid, const FlowControlData<BYTE> &fc) const
{
#ifdef _DEBUG
    if (fc.blockSize == __CANISOTP_FC_REQ_ALL_REMAINING && (fc.separationTime > 0x7F))
        throw std::exception("If blockSize == 0, ST must be in range of: 0 <= ST <= 127\n");
#endif
    if (bus->Transmit(txid, NewFlowControlFrame(fc)))
        return MsgStateResult::CONTINUE;

    LogErrorInfo("ICanBusTx::Transmit failed while transmitting FC frame ", fc.type, " to ", txid);
    return MsgStateResult::ERROR;
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

    frame->data[0] = ((BYTE)(((BYTE)FrameType::CONSECUTIVE << 4) | (index % (__CANISOTP_MAX_CF_INDEX + 1))));
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

#pragma region Padding statics
static size_t GetSizeWithoutPaddingByte(CAN::FrameData frame, int padding)
{
    return GetSizeWithoutPaddingByte(frame.data.data(), (size_t)frame.size, padding);
}
static size_t GetSizeWithoutPaddingByte(const std::vector<BYTE> &data, int padding)
{
    return GetSizeWithoutPaddingByte(data.data(), data.size(), padding);
}
static size_t GetSizeWithoutPaddingByte(const BYTE *begin, size_t datasize, int padding)
{
    if (padding < 0)
        return datasize;

    const auto padbyte = static_cast<BYTE>(padding);
    auto i = static_cast<int64_t>(datasize - 1);
    while (i >= 0 && begin[i] == padbyte)
    {
        --datasize;
        --i;
    }
    return datasize;
}
#pragma endregion

