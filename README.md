### CAN ISO-TP (ISO 15765-2) Support Library C++14/17/20...

Implementation of CAN ISO-TP state machine for transmitting/receiving data using **Single** and **First**/**Consecutive** frames with optional padding support.

- User must provide a way to communicate at the frame level with CAN hardware.
- The library supports only standard CAN frames with 8-byte payload (both 11/29-bit IDs) and ISO-TP 4095-byte payload (not compatible with CAN FD).
- By default on receiving First-Frame, IsoTp state machine always requests all remaining frames wihout delay (**`Flow Control`: `Block size = 0`, `ST = 0`**).
- Suited mostly for applications under some OS environment.

---

## API version 1.0.x

Note: using [SemVer](https://semver.org/) standard in commit tags.  

### Usage

Implement `ICanBusTx` interface in class responsible for CAN communication e.g.
```cpp
class MyCanBus : public CAN::ICanBusTx 
{
    // ...
    // Neccessary declaration for CAN::IsoTp
    bool Transmit(CAN::CANID txid, const CAN::FrameData& txdata, CAN::Standard standard) override
    {
        // CAN TX logic. Should handle potential (txdata.size == 0) case.
        return true;  // on tx success 
    }
    // "RX" function can be whatever as long as its results 
    //  are converted to CAN::FrameData before calling IsoTp::ProcessIsotpResponse
};
```

Create `CAN::IsoTp` object with suitable parameters e.g.
```cpp
constexpr CAN::BYTE PAD = 0xAA; // some implementations require specific padding to avoid Bit Stuffing 
constexpr CAN::Standard standard = CAN::Standard::CEFF; // e.g. CAN 2.0 extended 29-bit identifier

auto isotp = std::make_unique<CAN::IsoTp>(standard, PAD);
```

Optionally change your data representations in `isotpdef.h`
```cpp
// Defines representation of byte in CAN frame
typedef std::uint8_t BYTE;
// Defines type of CAN identifier. 32-bit value recommended for compatibility with extended frame format.
typedef std::uint32_t CANID;
```

Optionally set error logging function member e.g.
```cpp
isotp->m_funcLogError = [](const char* err) { std::clog << err; };
``` 
---
### API

##### RX

Process received frames in receive loop or something like "CAN data received" callback e.g.
```cpp
// ...
// std::array<CAN::BYTE, __CANISOTP_MAX_MSG_DATA_SIZE> m_responseBuffer;
std::vector<CAN::BYTE> m_responseBuffer; 
std::unique_ptr<MyCanBus> bus;
std::unique_ptr<CAN::IsoTp> m_isotp;

// Processing should be called for each received CAN frame, to ensure correct ISO-TP state 
void BusEvents::rx_callback(const MyFrame& canframe)
{
    CAN::FrameData data = _user_defined_conversion(canframe); 
    auto [txid, rxid] = _user_defined_canid_things(); // or separate CAN::IsoTp wrapper for each CANID pair

    auto result = m_isotp->ProcessIsotpResponse(bus, txid, rxid, data);
    switch (result)
    {
    case CAN::IsoTpState::ISOTP_CONTINUE:
        break;

    case CAN::IsoTpState::ISOTP_FINALIZE_TX:
        std::cout << "Info state - basically CONTINUE\n";
        std::cout << "Flow-Control was received from device, whole multi-frame message sent.\n";
        assert(false == isotp->IsTransmitActive()); // IsoTp class TX state automatically cleared
        break;

    case CAN::IsoTpState::ISOTP_FINALIZE_RX:
        // Copy collected payload bytes to provied buffer
        isotp->FinalizeIsotpResponse(&m_responseBuffer);
        assert(false == isotp->IsReceiveActive());  // IsoTp class RX state automatically cleared
        std::cout << "Received " << responseBuffer.size() << " bytes.\n";
        break;

    case CAN::IsoTpState::ISOTP_FLOW_ABORT:
        assert(false == isotp->IsReceiveActive()); // IsoTp class RX state automatically cleared
        break;

    case CAN::IsoTpState::ISOTP_ERROR:
        std::cout << "ISOTP error! Check err messages.\n";
        break;

    default:
        std::cout << "Invalid state!\n";
        break;
    }
}
//...
```
##### TX

Transmit messages using one of the following:
```cpp
// Copies payload to internal TX buffer and sends "Single/First-Frame" depending on data size.
// Any remaining bytes will be automatically sent as "Consecutive-Frames" 
// - after next received "Flow Control" is passed to 'IsoTp::ProcessIsotpResponse(...)'.
// If neccessary, CAN::Standard must be set via 'IsoTp::setDefaulStandard(...)'.
// Returns one of the following: IsoTpState::ISOTP_FINALIZE_TX/ISOTP_CONTINUE/ISOTP_ERROR.

IsoTpState TransmitMessage(ICanBusTx* bus, CANID txid, const BYTE* payload, size_t size);
IsoTpState TransmitMessage(ICanBusTx* bus, CANID txid, const std::vector<BYTE>& payload);
IsoTpState TransmitMessage(ICanBusTx* bus, CANID txid, const FrameData& payload);
```

---

### Build dependencies

GNU compilers for Win32 platform needs [pthreads](https://en.wikipedia.org/wiki/Pthreads) support (see `std::this_thread::slepp_for` in `isotp.cpp`).
Use appropriate version (out of the box since `GCC 13`?) or check out [mingw-std-threads](https://github.com/meganz/mingw-std-threads). 

When using modern Visual Studio C/C++ Windows packet, build-in `MSVC` should provide it by default.  

---

MC2024
