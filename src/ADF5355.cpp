#include "ADF5355.h"

// Macro utilities for math (integer arithmetic for division and clamping)
#define DIV_ROUND_CLOSEST(x, y) (((x) + ((y)/2)) / (y))
#define DIV_ROUND_UP(x, y) (((x) + (y) - 1) / (y))
#define CLAMP(val, minVal, maxVal) (((val) < (minVal)) ? (minVal) : (((val) > (maxVal)) ? (maxVal) : (val)))

// Constants from ADF5355 specifications
static const uint64_t ADF5355_MODULUS1 = 16777216ULL;       // 2^24, primary modulus
static const uint32_t ADF5355_MAX_MODULUS2 = 16384U;        // 2^14, max secondary modulus for ADF5355
static const uint32_t ADF5356_MAX_MODULUS2 = 268435456U;    // 2^28, max secondary modulus for ADF5356/ADF4356
static const uint32_t ADF5355_MIN_INT_89_PRESCALER = 75U;   // threshold for prescaler selection
static const uint32_t ADF5355_MAX_FREQ_PFD = 75000000U;     // max PFD frequency = 75 MHz&#8203;:contentReference[oaicite:5]{index=5}

// Register addresses (simply 0-13)
inline uint8_t regAddr(uint8_t regIndex) { return regIndex; }

// Bitfield macros for register construction (shifting values into correct bit positions):
// (These mirror definitions from the Analog Devices driver)
inline uint32_t REG0_INT(uint32_t x)    { return ((x & 0xFFFF) << 4); }
inline uint32_t REG0_PRESCALER(bool x)  { return (uint32_t)x << 20; }
inline uint32_t REG0_AUTOCAL(bool x)    { return (uint32_t)x << 21; }

inline uint32_t REG1_FRACT(uint32_t x)  { return (x & 0xFFFFFF) << 4; }

inline uint32_t REG2_MOD2(uint32_t x)   { return (x & 0x3FFF) << 4; }
inline uint32_t REG2_FRAC2(uint32_t x)  { return (x & 0x3FFF) << 18; }

inline uint32_t REG3_PHASE(uint32_t x)  { return (x & 0xFFFFFF) << 4; }
// (Phase adjust, resync, exact SD load are fixed or unused in this implementation)

inline uint32_t REG4_COUNTER_RESET(bool x)   { return (uint32_t)x << 4; }
inline uint32_t REG4_CP_THREESTATE(bool x)   { return (uint32_t)x << 5; }
inline uint32_t REG4_POWER_DOWN(bool x)      { return (uint32_t)x << 6; }
inline uint32_t REG4_PD_POLARITY_POS(bool x) { return (uint32_t)x << 7; }
inline uint32_t REG4_MUX_LOGIC(bool x)       { return (uint32_t)x << 8; }
inline uint32_t REG4_REFIN_DIFF(bool x)      { return (uint32_t)x << 9; }
inline uint32_t REG4_CP_CURRENT(uint32_t x)  { return (x & 0xF) << 10; }
inline uint32_t REG4_DOUBLE_BUFF(bool x)     { return (uint32_t)x << 14; }
inline uint32_t REG4_R_CNT(uint16_t x)       { return (x & 0x3FF) << 15; }
inline uint32_t REG4_RDIV2(bool x)           { return (uint32_t)x << 25; }
inline uint32_t REG4_REF_DBL(bool x)         { return (uint32_t)x << 26; }
inline uint32_t REG4_MUXOUT(uint8_t x)       { return (x & 0x7) << 27; }

inline uint32_t REG5_DEFAULT()              { return 0x00800025; } // Hardcoded default for Reg5

// For ADF4355/4356 (which have adjustable output B power):
inline uint32_t REG6_OUTB_PWR(uint8_t x)    { return (x & 0x3) << 7; }  
inline uint32_t REG6_OUTB_EN(bool x)        { return (uint32_t)x << 9; }  // For ADF4355/4356
inline uint32_t REG6_OUTA_PWR(uint8_t x)    { return (x & 0x3) << 4; }
inline uint32_t REG6_OUTA_EN(bool x)        { return (uint32_t)x << 6; }
inline uint32_t REG6_RF_DIV_SEL(uint8_t x)  { return (x & 0x7) << 21; }
inline uint32_t REG6_MUTE_TILL_LOCK(bool x) { return (uint32_t)x << 11; }
inline uint32_t REG6_CP_BLEED(uint8_t x)    { return (x & 0xFF) << 13; }
inline uint32_t REG6_FEEDBACK_FUND(bool x)  { return (uint32_t)x << 24; }
inline uint32_t REG6_NEG_BLEED_EN(bool x)   { return (uint32_t)x << 29; }
inline uint32_t REG6_GATED_BLEED_EN(bool x) { return (uint32_t)x << 30; }
inline uint32_t REG6_BLEED_POL(bool x)      { return (uint32_t)x << 31; }
// For ADF5355/5356, RF_OUTB_EN bit is at position 10:
inline uint32_t REG6_OUTB_EN_5355(bool x)   { return (uint32_t)x << 10; }
// Default static bits for Reg6 (ADF5355): sets feedback select to fundamental, etc.
inline uint32_t REG6_DEFAULT_5355()        { return 0x14000006; }
inline uint32_t REG6_DEFAULT_4356()        { return 0x15600006; } // Default for ADF4356/ADF5356
// (These defaults come from device datasheets and ensure reserved bits are set correctly)

inline uint32_t REG7_DEFAULT_5355()        { return 0x10000007; }
inline uint32_t REG7_DEFAULT_5356()        { return 0x04000007; }

inline uint32_t REG8_DEFAULT_5355()        { return 0x102D0428; }
inline uint32_t REG8_DEFAULT_5356()        { return 0x15596568; }

inline uint32_t REG9_TIMEOUT(uint16_t x)   { return (x & 0x3FF) << 14; }
inline uint32_t REG9_SYNTH_LOCK_TIMEOUT(uint8_t x){ return (x & 0x1F) << 4; }
inline uint32_t REG9_ALC_TIMEOUT(uint8_t x){ return (x & 0x1F) << 9; }
// (REG9 VCO_BAND_DIV is computed in code)

inline uint32_t REG10_ADC_EN(bool x)       { return (uint32_t)x << 4; }
inline uint32_t REG10_ADC_CONV_EN(bool x)  { return (uint32_t)x << 5; }
inline uint32_t REG10_ADC_CLK_DIV(uint8_t x){ return (x & 0xFF) << 6; }
inline uint32_t REG10_DEFAULT()           { return 0x00C0000A; }

inline uint32_t REG11_DEFAULT_5355()      { return 0x0061300B; }
inline uint32_t REG11_DEFAULT_5356()      { return 0x0061200B; }

inline uint32_t REG12_PHASE_RESYNC_DIV(uint32_t x, bool is5356){
    // Reg12: 16-bit resync divider for 5355, 20-bit for 5356
    return is5356 ? ((x & 0xFFFFF) << 12) : ((x & 0xFFFF) << 16);
}
inline uint32_t REG12_DEFAULT_5355()      { return 0x0000041C; }
inline uint32_t REG12_DEFAULT_5356()      { return 0x000005FC; }

inline uint32_t REG13_MOD2_MSB(uint32_t x) { return (x & 0x3FFF) << 4; }
inline uint32_t REG13_FRAC2_MSB(uint32_t x){ return (x & 0x3FFF) << 18; }

// Constructor
ADF5355::ADF5355() : _deviceId(ADF5355_DEVICE), _csPin(10), _initialized(false),
                     _refFreq(0), _refDiv2En(false), _refDoublerEn(false),
                     _minVcoFreq(0), _maxVcoFreq(0), _minOutFreq(0), _maxOutFreq(0),
                     _fpfd(0), _rCounter(1), _allRegsSynced(false), _outBselFund(false),
                     _cpCurrent_uA(5000), _cpNegBleedEn(false), _cpGatedBleedEn(false),
                     _cpBleedPolEn(false), _muteTillLockEn(false), _phaseDetPol(0),
                     _refDiff(false), _muxOutSel(ADF5355_MUX_DIGITAL_LD)
{
    // Initialize output enables/powers and frequencies to defaults
    _freqReq[0] = _freqReq[1] = 0;
    _outEnable[0] = true;
    _outEnable[1] = false;
    _outPower[0] = 3;
    _outPower[1] = 3;
    // Clear register mirror
    for(uint8_t i = 0; i < _REG_COUNT; ++i) {
        _regs[i] = 0;
    }
}

// Begin / initialize the device
int32_t ADF5355::begin(uint8_t csPin, ADF5355_DeviceID deviceId,
                       uint64_t refFreq, uint64_t initialFreq, uint8_t channel,
                       bool clkinDiv2, bool refDoubler, uint32_t cpCurrent_uA,
                       bool cpNegBleed, bool cpGatedBleed, bool cpBleedPol,
                       ADF5355_MuxOut muxOutSel, bool muteUntilLock,
                       bool outAEnable, bool outBEnable, uint8_t outAPower, uint8_t outBPower)
{
    _deviceId = deviceId;
    _csPin = csPin;
    _refFreq = (uint32_t)refFreq;    // (Reference frequency fits in 32 bits up to 600 MHz)
    _refDiv2En = clkinDiv2;
    _refDoublerEn = refDoubler;
    _cpCurrent_uA = cpCurrent_uA;
    _cpNegBleedEn = cpNegBleed;
    _cpGatedBleedEn = cpGatedBleed;
    _cpBleedPolEn = cpBleedPol;
    _muxOutSel = muxOutSel;
    _muteTillLockEn = muteUntilLock;
    _outEnable[0] = outAEnable;
    _outEnable[1] = outBEnable;
    _outPower[0] = CLAMP(outAPower, 0U, 3U);
    _outPower[1] = CLAMP(outBPower, 0U, 3U);
    _phaseDetPol = 0;              // use positive phase detector polarity unless requested negative
    _refDiff = false;              // assume single-ended reference by default
    _outBselFund = false;          // used only for ADF4356 device

    // Determine device frequency limits
    switch(_deviceId) {
        case ADF5355_DEVICE:
        case ADF5356_DEVICE:
        case ADF4356_DEVICE:
            _minVcoFreq = 3400000000ULL;  // 3.4 GHz
            _maxVcoFreq = 6800000000ULL;  // 6.8 GHz
            _minOutFreq = (_deviceId == ADF5355_DEVICE || _deviceId == ADF5356_DEVICE)
                            ? (_minVcoFreq / 64ULL)  // with max divider 64
                            : (_minVcoFreq / 72ULL);  // ADF4356 has 72 max divider (if any difference)
            _maxOutFreq = (_deviceId == ADF5355_DEVICE || _deviceId == ADF5356_DEVICE) 
                            ? _maxVcoFreq 
                            : _maxVcoFreq;
            break;
        case ADF4355_DEVICE:
            _minVcoFreq = 3400000000ULL;
            _maxVcoFreq = 6800000000ULL;
            _minOutFreq = _minVcoFreq / 64ULL;
            _maxOutFreq = _maxVcoFreq;
            break;
        case ADF4355_2_DEVICE:
            _minVcoFreq = 3400000000ULL;
            _maxVcoFreq = 6800000000ULL;
            _minOutFreq = _minVcoFreq / 64ULL;
            _maxOutFreq = _maxVcoFreq;
            break;
        case ADF4355_3_DEVICE:
            _minVcoFreq = 3300000000ULL;
            _maxVcoFreq = 6600000000ULL;
            _minOutFreq = _minVcoFreq / 64ULL;
            _maxOutFreq = _maxVcoFreq;
            break;
    }
    // For ADF4356/ADF5356, the device has an additional mode for output B fundamental output:
    if(_deviceId == ADF4356_DEVICE) {
        // ADF4356 allows output B to be fundamental as an option
        _outBselFund = !outBEnable ? false : true;  // if outB is enabled on ADF4356, assume fundamental mode by default
    }

    // Configure SPI
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);          // CS high (inactive)
    SPI.begin();                         // initialize SPI
    // Set SPI settings: use a safe default clock (e.g. 8 MHz for Uno, up to 10 MHz)
    // You can modify SPISettings if a different speed is desired.
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    SPI.endTransaction();

    // Calculate initial R counter and PFD (frequency to phase detector)
    _rCounter = 0;
    do {
        _rCounter++;
        // Effective reference into PFD = refFreq * (doubler?2:1) / (R * (div2?2:1))
        _fpfd = (uint32_t)(_refFreq * (_refDoublerEn ? 2ULL : 1ULL) / (_rCounter * (_refDiv2En ? 2ULL : 1ULL)));
    } while(_fpfd > ADF5355_MAX_FREQ_PFD && _rCounter < 1024);
    if(_fpfd > ADF5355_MAX_FREQ_PFD) {
        // Could not reduce PFD below max – invalid reference configuration
        return ADF5355_ERROR_INVALID;
    }

    // Prepare static register values (that do not depend on output frequency)
    // REG4 – Reference and charge pump setup
    uint32_t cpIdx = DIV_ROUND_CLOSEST((int32_t)_cpCurrent_uA - 315, 315U);
    cpIdx = CLAMP(cpIdx, 0U, 15U);
    _regs[4] = REG4_COUNTER_RESET(0) | REG4_CP_THREESTATE(0) | REG4_POWER_DOWN(0)
             | REG4_PD_POLARITY_POS(true)                    // positive phase detector polarity
             | REG4_MUX_LOGIC(false)                        // MUXOUT logic level (false = open-drain for 5V compat if needed)
             | REG4_REFIN_DIFF(_refDiff) 
             | REG4_CP_CURRENT(cpIdx)
             | REG4_DOUBLE_BUFF(true)
             | REG4_R_CNT(_rCounter - 1)                    // R counter (10-bit, program R-1)
             | REG4_RDIV2(_refDiv2En)
             | REG4_REF_DBL(_refDoublerEn)
             | REG4_MUXOUT((uint8_t)_muxOutSel);
    // REG5 – constant default
    _regs[5] = REG5_DEFAULT();
    // REG7 – lock detect and other features
    if(_deviceId == ADF5356_DEVICE) {
        _regs[7] = (REG6_BLEED_POL(false) | REG7_DEFAULT_5356());  // for ADF5356, ensure bleed polarity bit default
    } else {
        _regs[7] = REG7_DEFAULT_5355();
    }
    // REG8 – VCO calibration and noise settings (defaults vary by device)
    _regs[8] = (_deviceId == ADF5356_DEVICE || _deviceId == ADF4356_DEVICE) ? REG8_DEFAULT_5356() : REG8_DEFAULT_5355();
    // REG9 – timeout settings (auto-calibration, lock detect)
    uint32_t timeout = DIV_ROUND_UP(_fpfd, 600000U);  // ~ = ceil(fpfd / (20k * 30)) as per ADI driver&#8203;:contentReference[oaicite:6]{index=6}
    timeout = CLAMP(timeout, 1U, 1023U);
    uint8_t synth_lock_to = (uint8_t)DIV_ROUND_UP(_fpfd * 2U, 100000U * timeout);
    synth_lock_to = CLAMP(synth_lock_to, 0U, 31U);
    uint8_t alc_wait_to = (uint8_t)DIV_ROUND_UP(_fpfd * 5U, 100000U * timeout);
    alc_wait_to = CLAMP(alc_wait_to, 0U, 31U);
    uint8_t vco_band_div = (uint8_t)DIV_ROUND_UP(_fpfd,
                         (_deviceId == ADF4356_DEVICE || _deviceId == ADF5356_DEVICE) ? 1600000U : 2400000U);
    vco_band_div = CLAMP(vco_band_div, 1U, 255U);
    _regs[9] = REG9_TIMEOUT((uint16_t)timeout) 
             | REG9_SYNTH_LOCK_TIMEOUT(synth_lock_to)
             | REG9_ALC_TIMEOUT(alc_wait_to) 
             | ((uint32_t)vco_band_div << 24);
    // REG10 – ADC clock and ADC enable bits
    uint8_t adcClkDiv = CLAMP((uint8_t)DIV_ROUND_UP(_fpfd / 100000U - 2U, 4U), 1U, 255U);
    _regs[10] = REG10_ADC_EN(true) | REG10_ADC_CONV_EN(true) | REG10_ADC_CLK_DIV(adcClkDiv) | REG10_DEFAULT();
    // REG11 – default
    _regs[11] = (_deviceId == ADF5356_DEVICE || _deviceId == ADF4356_DEVICE) ? REG11_DEFAULT_5356() : REG11_DEFAULT_5355();
    // REG12 – default + set phase resync divider = 1
    _regs[12] = (_deviceId == ADF5356_DEVICE || _deviceId == ADF4356_DEVICE)
              ? (REG12_PHASE_RESYNC_DIV(1, true) | REG12_DEFAULT_5356())
              : (REG12_PHASE_RESYNC_DIV(1, false) | REG12_DEFAULT_5355());
    // (REG13 will be set in setFrequency if needed for ADF5356/4356)

    _allRegsSynced = false;
    _initialized = true;

    // Set initial frequency if provided (non-zero)
    int32_t result = ADF5355_ERROR_OK;
    if(initialFreq > 0ULL) {
        result = setFrequency(initialFreq, channel);
    } else {
        // Even if no frequency set, perform a full register write (to load defaults)
        result = updateRegisters(true);
    }
    return result;
}

// Compute PLL integer and fractional values for a given VCO frequency and PFD
void ADF5355::computePLL(uint64_t vcoFreq, uint32_t pfdFreq, uint32_t maxMod2,
                         uint32_t &intVal, uint32_t &frac1, uint32_t &frac2, uint32_t &mod2)
{
    // Compute integer part and initial remainder
    uint64_t integerPart = vcoFreq / pfdFreq;
    uint64_t remainder = vcoFreq % pfdFreq;
    if(integerPart < 1ULL) integerPart = 1ULL;  // ensure at least 1 (should always be >=1 for these devices)

    // Compute primary fractional (fract1) and secondary numerator (fract2)
    // Using 64-bit math to maintain precision: fract1 = floor((remainder * 2^24) / pfd)
    // and fract2 = (remainder * 2^24) mod pfd.
    uint64_t tmp = remainder * ADF5355_MODULUS1;
    uint64_t frac1_64 = tmp / pfdFreq;
    uint64_t rem2 = tmp % pfdFreq;
    uint32_t frac1_val = (uint32_t)frac1_64;   // 24-bit value (assuming it fits in 24 bits)
    uint32_t fract2_val = (uint32_t)rem2;
    uint32_t mod2_val = pfdFreq;

    // Scale down mod2 and fract2 if mod2 exceeds device limits
    while(mod2_val > maxMod2) {
        mod2_val >>= 1;
        fract2_val >>= 1;
    }
    // Reduce fract2/mod2 fraction
    // Compute GCD to simplify fraction
    uint32_t a = fract2_val, b = mod2_val;
    while(b != 0) {
        uint32_t t = b;
        b = a % b;
        a = t;
    }
    uint32_t gcd = a;
    if(gcd != 0) {
        mod2_val /= gcd;
        fract2_val /= gcd;
    }
    // Return values
    intVal = (uint32_t)integerPart;
    frac1 = frac1_val;
    frac2 = fract2_val;
    mod2 = mod2_val;
}

// Set output frequency on a given channel
int32_t ADF5355::setFrequency(uint64_t frequency, uint8_t channel) {
    if(!_initialized) {
        return ADF5355_ERROR_INVALID;  // not initialized
    }
    if(channel > 1) {
        return ADF5355_ERROR_INVALID;
    }

    // Check requested frequency against device limits
    if(frequency < _minOutFreq || frequency > _maxOutFreq) {
        return ADF5355_ERROR_INVALID;
    }

    // Determine effective VCO frequency and RF divider setting for the channel
    uint64_t vcoFrequency = frequency;
    uint8_t rf_div_sel = 0;
    if(channel == 0) {  
        // Channel A (fundamental output with divider chain)
        // Use the RF divider to bring VCO frequency into its valid range [minVco, maxVco]
        while(vcoFrequency < _minVcoFreq && (rf_div_sel < 7)) {
            vcoFrequency <<= 1;  // multiply freq by 2
            rf_div_sel++;
        }
        if(vcoFrequency < _minVcoFreq || vcoFrequency > _maxVcoFreq) {
            return ADF5355_ERROR_INVALID; // cannot attain this frequency
        }
        // Configure register 6 for output A:
        _regs[6] = 0; // reset reg6 (we will rebuild it fully)
        // For ADF4355/4356 devices, include output B power and enable if applicable:
        if(_deviceId == ADF4355_DEVICE || _deviceId == ADF4355_2_DEVICE || _deviceId == ADF4355_3_DEVICE || _deviceId == ADF4356_DEVICE) {
            _regs[6] |= REG6_OUTB_PWR(_outPower[1]) | REG6_OUTB_EN(_outEnable[1]);
        }
        // Common reg6 fields for channel A:
        _regs[6] |= REG6_OUTA_PWR(_outPower[0]) 
                 | REG6_OUTA_EN(_outEnable[0])
                 | REG6_MUTE_TILL_LOCK(_muteTillLockEn)
                 | REG6_CP_BLEED(0)           // will compute bleed current next
                 | REG6_RF_DIV_SEL(rf_div_sel)
                 | REG6_FEEDBACK_FUND(true)   // always use fundamental feedback
                 | REG6_NEG_BLEED_EN(_cpNegBleedEn)
                 | REG6_GATED_BLEED_EN(_cpGatedBleedEn);
        if(_deviceId == ADF4356_DEVICE || _deviceId == ADF5356_DEVICE) {
            // ADF4356/5356 have a bleed polarity bit:
            _regs[6] |= REG6_BLEED_POL(_cpBleedPolEn);
        }
        // For ADF5355/5356, output B enable bit is at bit 10. If we are using channel A primarily,
        // we can disable RFoutB (to save power) by clearing that bit.
        if(_deviceId == ADF5355_DEVICE || _deviceId == ADF5356_DEVICE) {
            _regs[6] |= REG6_OUTB_EN_5355(_outEnable[1]);
        }
    } else {  
        // Channel B (for high-frequency output, e.g. ADF5355 uses doubler, ADF4356 uses second output fundamental)
        if(_deviceId == ADF5355_DEVICE || _deviceId == ADF5356_DEVICE) {
            // ADF5355/5356: RFoutB is the doubler output (freq must be >= ~6800 MHz)
            if(frequency < 6800000000ULL || frequency > 13600000000ULL) {
                return ADF5355_ERROR_INVALID;
            }
            // The VCO frequency is half the desired output (because of doubler on RFoutB)
            vcoFrequency = frequency >> 1;  
            rf_div_sel = 0;  // (doubler path doesn't use the output divider)
            if(vcoFrequency < _minVcoFreq || vcoFrequency > _maxVcoFreq) {
                return ADF5355_ERROR_INVALID;
            }
            // Reg6 configuration for channel B on ADF5355:
            _regs[6] = 0;
            _regs[6] |= REG6_OUTA_PWR(_outPower[0]) | REG6_OUTA_EN(_outEnable[0]);
            _regs[6] |= REG6_OUTB_EN_5355(true);  // ensure RFoutB is enabled for channel B output
            _regs[6] |= REG6_MUTE_TILL_LOCK(_muteTillLockEn)
                     | REG6_CP_BLEED(0)
                     | REG6_RF_DIV_SEL(0)
                     | REG6_FEEDBACK_FUND(true)
                     | REG6_NEG_BLEED_EN(_cpNegBleedEn)
                     | REG6_GATED_BLEED_EN(_cpGatedBleedEn);
        } else if(_deviceId == ADF4356_DEVICE) {
            // ADF4356: RFoutB can be used for fundamental output (no doubler, just an independent output).
            // In this case, we require outB in "fundamental mode" (outBselFund = true) and frequency in normal range:
            if(!_outBselFund || frequency < _minOutFreq || frequency > _maxOutFreq) {
                return ADF5355_ERROR_INVALID;
            }
            vcoFrequency = frequency;
            rf_div_sel = 0;
            // If frequency below VCO min, use divider (like channel A logic for 4356)
            while(vcoFrequency < _minVcoFreq && (rf_div_sel < 7)) {
                vcoFrequency <<= 1;
                rf_div_sel++;
            }
            if(vcoFrequency < _minVcoFreq || vcoFrequency > _maxVcoFreq) {
                return ADF5355_ERROR_INVALID;
            }
            _regs[6] = 0;
            // For ADF4356, output B power and enable (as fundamental output):
            _regs[6] |= REG6_OUTB_PWR(_outPower[1]) | REG6_OUTB_EN(_outEnable[1]);
            // Also include output A if enabled:
            _regs[6] |= REG6_OUTA_PWR(_outPower[0]) | REG6_OUTA_EN(_outEnable[0]);
            _regs[6] |= REG6_MUTE_TILL_LOCK(_muteTillLockEn)
                     | REG6_CP_BLEED(0)
                     | REG6_RF_DIV_SEL(rf_div_sel)
                     | REG6_FEEDBACK_FUND(true)
                     | REG6_NEG_BLEED_EN(_cpNegBleedEn)
                     | REG6_GATED_BLEED_EN(_cpGatedBleedEn)
                     | REG6_BLEED_POL(_cpBleedPolEn);
        } else {
            // Other devices (ADF4355 variants) do not have a distinct channel B usage beyond secondary output:
            // We treat channel 1 as enabling output B (with fundamental feed).
            if(frequency < _minOutFreq || frequency > _maxOutFreq) {
                return ADF5355_ERROR_INVALID;
            }
            vcoFrequency = frequency;
            rf_div_sel = 0;
            while(vcoFrequency < _minVcoFreq && (rf_div_sel < 7)) {
                vcoFrequency <<= 1;
                rf_div_sel++;
            }
            if(vcoFrequency < _minVcoFreq || vcoFrequency > _maxVcoFreq) {
                return ADF5355_ERROR_INVALID;
            }
            _regs[6] = 0;
            // Enable output B for ADF4355 family:
            _regs[6] |= REG6_OUTB_PWR(_outPower[1]) | REG6_OUTB_EN(true);
            // Also preserve A output settings:
            _regs[6] |= REG6_OUTA_PWR(_outPower[0]) | REG6_OUTA_EN(_outEnable[0]);
            _regs[6] |= REG6_MUTE_TILL_LOCK(_muteTillLockEn)
                     | REG6_CP_BLEED(0)
                     | REG6_RF_DIV_SEL(rf_div_sel)
                     | REG6_FEEDBACK_FUND(true)
                     | REG6_NEG_BLEED_EN(_cpNegBleedEn)
                     | REG6_GATED_BLEED_EN(_cpGatedBleedEn);
        }
    }

    // Compute PLL settings for the target VCO frequency
    uint32_t integer, frac1, frac2, mod2;
    uint32_t maxMod2 = (_deviceId == ADF4356_DEVICE || _deviceId == ADF5356_DEVICE) ? ADF5356_MAX_MODULUS2 : ADF5355_MAX_MODULUS2;
    computePLL(vcoFrequency, _fpfd, maxMod2, integer, frac1, frac2, mod2);

    // Determine prescaler: use 8/9 prescaler if integer >= 75 (per datasheet threshold)
    bool prescaler89 = (integer >= ADF5355_MIN_INT_89_PRESCALER);

    // Charge pump bleed current (only for fractional mode and lower PFD)
    bool applyNegBleed = _cpNegBleedEn;
    if(_fpfd > 100000000U || (frac1 == 0 && frac2 == 0)) {
        // If PFD > 100 MHz or integer mode (no fractional), disable negative bleed regardless
        applyNegBleed = false;
    }
    uint32_t cp_bleed_val = 0;
    if(_deviceId == ADF4356_DEVICE || _deviceId == ADF5356_DEVICE) {
        // For ADF4356/5356, bleed = (24 * (fpfd/1000) * cp_uA) / (61440 * 900)
        cp_bleed_val = (uint32_t)((24ULL * (_fpfd / 1000ULL) * _cpCurrent_uA) / (61440ULL * 900ULL));
    } else {
        // For ADF5355/4355, bleed = ceil(400 * cp_uA / (integer * 375))
        if(integer * 375U != 0) {
            cp_bleed_val = DIV_ROUND_UP(400U * _cpCurrent_uA, integer * 375U);
        } else {
            cp_bleed_val = 0;
        }
    }
    cp_bleed_val = CLAMP(cp_bleed_val, 1U, 255U);

    // Populate frequency-dependent registers:
    _regs[0] = REG0_INT(integer) | REG0_PRESCALER(prescaler89) | REG0_AUTOCAL(true);
    _regs[1] = REG1_FRACT(frac1);
    _regs[2] = REG2_MOD2(mod2) | REG2_FRAC2(frac2);
    // If using ADF4356/5356, set MSBs of frac2 and mod2 in Reg13:
    if(_deviceId == ADF4356_DEVICE || _deviceId == ADF5356_DEVICE) {
        _regs[13] = REG13_MOD2_MSB(mod2 >> 14) | REG13_FRAC2_MSB(frac2 >> 14) | 0x0000000D;
    }
    // Update Reg6 with final bleed current and ensure default reserved bits:
    if(_deviceId == ADF4356_DEVICE || _deviceId == ADF5356_DEVICE) {
        _regs[6] |= REG6_CP_BLEED(cp_bleed_val) | REG6_DEFAULT_4356();
    } else {
        _regs[6] |= REG6_CP_BLEED(cp_bleed_val) | REG6_DEFAULT_5355();
    }

    // Store requested frequency for reference
    _freqReq[channel] = frequency;
    // Perform register update (partial or full depending on sync state)
    return updateRegisters(!_allRegsSynced);
}

// Write a 32-bit register value via SPI (handles the latch enable toggling)
void ADF5355::writeRegister(uint8_t regAddr, uint32_t regValue) {
    // Combine 28-bit data and 4-bit address:
    uint32_t outWord = regValue | regAddr;
    // SPI transaction: Chip select low -> send 4 bytes -> latch (CS high)
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    digitalWrite(_csPin, LOW);
    // Send 32-bit word, MSB first
    SPI.transfer((uint8_t)(outWord >> 24));
    SPI.transfer((uint8_t)(outWord >> 16));
    SPI.transfer((uint8_t)(outWord >> 8));
    SPI.transfer((uint8_t) outWord);
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
    // The ADF5355 needs a minimum LE high pulse; digitalWrite already introduces a small delay.
    delayMicroseconds(1); // ensure >10 ns LE pulse (1 µs is plenty)
}

// Internal function to write registers to device
int32_t ADF5355::updateRegisters(bool fullSync) {
    // Determine highest register used by this device (ADF5355 uses R0–R12, ADF5356 uses R0–R13)
    uint8_t maxReg = (_deviceId == ADF4356_DEVICE || _deviceId == ADF5356_DEVICE) ? 13 : 12;
    if(fullSync || !_allRegsSynced) {
        // Write all registers (from highest down to 1; R0 will be written last after autocal delay)
        for(int8_t reg = maxReg; reg >= 1; --reg) {
            writeRegister(regAddr(reg), _regs[reg]);
        }
        _allRegsSynced = true;
    } else {
        // Optimized update – only write registers that changed for frequency update:
        if(_deviceId == ADF4356_DEVICE || _deviceId == ADF5356_DEVICE) {
            writeRegister(13, _regs[13]);  // MSBs of fraction for 4356/5356
        }
        writeRegister(10, _regs[10]);      // ADC config (may include timeout adjustments)
        writeRegister(6, _regs[6]);       // RF output config (power, mute, div, bleed, etc.)
        // Toggle counter reset bit in Reg4 during frequency update:
        writeRegister(4, _regs[4] | REG4_COUNTER_RESET(true));
        writeRegister(2, _regs[2]);
        writeRegister(1, _regs[1]);
        // Write Reg0 with autocal disabled (bit21=0) to load new N value without starting calibration
        writeRegister(0, _regs[0] & ~REG0_AUTOCAL(true));
        // Re-write Reg4 with counter reset disabled (back to normal)
        writeRegister(4, _regs[4]);
    }
    // Delay > 16 cycles of ADC_CLK (ensures VCO band calibration timing)
    delayMicroseconds( (16 * 1000000UL) / _fpfd + 10 );  // conservative delay: 16 * (1/FPFD) in microseconds
    // Finally, write Reg0 with AUTOCAL = 1 to trigger calibration of the new frequency
    writeRegister(0, _regs[0]);
    return ADF5355_ERROR_OK;
}

// Get the current frequency of a channel (returns last set value)
uint64_t ADF5355::getFrequency(uint8_t channel) const {
    if(channel > 1) return 0;
    return _freqReq[channel];
}

// Round to the nearest achievable frequency (for simplicity, we assume any provided freq is achievable in this driver)
int32_t ADF5355::roundFrequency(uint64_t desiredFreq, uint64_t &roundedFreq) const {
    // In this implementation, we do not adjust the frequency (the ADF5355 has fine resolution due to fractional-N).
    roundedFreq = desiredFreq;
    return ADF5355_ERROR_OK;
}

// End communication and shut down outputs (optional)
int32_t ADF5355::end() {
    // Optionally, power down the device by writing the power-down bit:
    _regs[4] |= REG4_POWER_DOWN(true);
    writeRegister(4, _regs[4]);
    // Disable both RF outputs:
    _regs[6] &= ~REG6_OUTA_EN(true);
    if(_deviceId == ADF5355_DEVICE || _deviceId == ADF5356_DEVICE) {
        _regs[6] &= ~REG6_OUTB_EN_5355(true);
    } else {
        _regs[6] &= ~REG6_OUTB_EN(true);
    }
    writeRegister(6, _regs[6]);
    // Release SPI (not strictly necessary, but good practice)
    // (Chip remains in power-down until next begin())
    _initialized = false;
    return ADF5355_ERROR_OK;
}
