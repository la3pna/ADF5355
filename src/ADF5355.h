#ifndef ADF5355_H
#define ADF5355_H

#include <Arduino.h>
#include <SPI.h>

/*
 * ADF5355 Arduino Library
 * Generated on 2025-04-22
 * BSD 3-Clause License, see LICENSE file
 */

enum ADF5355_DeviceID {
    ADF5355_DEVICE = 0,
    ADF4355_DEVICE,
    ADF4355_2_DEVICE,
    ADF4355_3_DEVICE,
    ADF4356_DEVICE,
    ADF5356_DEVICE
};

enum ADF5355_MuxOut {
    ADF5355_MUX_TRISTATE,
    ADF5355_MUX_DVDD,
    ADF5355_MUX_GND,
    ADF5355_MUX_R_DIV_OUT,
    ADF5355_MUX_N_DIV_OUT,
    ADF5355_MUX_ANALOG_LD,
    ADF5355_MUX_DIGITAL_LD
};

const int32_t ADF5355_ERROR_OK      = 0;
const int32_t ADF5355_ERROR_INVALID = -1;
const int32_t ADF5355_ERROR_SPI     = -2;

class ADF5355 {
public:
    ADF5355();

    int32_t begin(uint8_t csPin,
                  ADF5355_DeviceID deviceId = ADF5355_DEVICE,
                  uint64_t refFreq = 100000000ULL,
                  uint64_t initialFreq = 0ULL,
                  uint8_t channel = 0,
                  bool clkinDiv2 = false,
                  bool refDoubler = false,
                  uint32_t cpCurrent_uA = 5000,
                  bool cpNegBleed = false,
                  bool cpGatedBleed = false,
                  bool cpBleedPol = false,
                  ADF5355_MuxOut muxOutSel = ADF5355_MUX_DIGITAL_LD,
                  bool muteUntilLock = false,
                  bool outAEnable = true,
                  bool outBEnable = false,
                  uint8_t outAPower = 3,
                  uint8_t outBPower = 3);

    int32_t setFrequency(uint64_t frequency, uint8_t channel = 0);
    uint64_t getFrequency(uint8_t channel = 0) const;
    int32_t roundFrequency(uint64_t desiredFreq, uint64_t &roundedFreq) const;
    int32_t end();

private:
    void writeRegister(uint8_t regAddr, uint32_t regValue);
    void computePLL(uint64_t vcoFreq, uint32_t pfdFreq, uint32_t maxMod2,
                    uint32_t &intVal, uint32_t &frac1, uint32_t &frac2, uint32_t &mod2);
    int32_t updateRegisters(bool fullSync);

    // Internal state
    static const uint8_t _REG_COUNT = 14;
    uint32_t _regs[_REG_COUNT];

    ADF5355_DeviceID _deviceId;
    uint8_t _csPin;
    bool _initialized;

    // Reference & PFD
    uint32_t _refFreq;
    bool _refDiv2En;
    bool _refDoublerEn;
    uint32_t _fpfd;
    uint16_t _rCounter;

    // Frequency limits
    uint64_t _minVcoFreq;
    uint64_t _maxVcoFreq;
    uint64_t _minOutFreq;
    uint64_t _maxOutFreq;

    // Channel state
    uint64_t _freqReq[2];
    bool _outEnable[2];
    uint8_t _outPower[2];
    bool _outBselFund;

    // Charge pump & bleed
    uint32_t _cpCurrent_uA;
    bool _cpNegBleedEn;
    bool _cpGatedBleedEn;
    bool _cpBleedPolEn;

    bool _muteTillLockEn;
    uint8_t _phaseDetPol;
    bool _refDiff;
    ADF5355_MuxOut _muxOutSel;

    bool _allRegsSynced;
};

#endif
