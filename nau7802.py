from enum import Enum
import ctypes


# Register Map
class ScaleRegisters(Enum):
    PU_CTRL = 0x00
    CTRL1 = 0x01
    CTRL2 = 0x02
    OCAL1_B2 = 0x03
    OCAL1_B1 = 0x04
    OCAL1_B0 = 0x05
    GCAL1_B3 = 0x06
    GCAL1_B2 = 0x07
    GCAL1_B1 = 0x08
    GCAL1_B0 = 0x09
    OCAL2_B2 = 0x0A
    OCAL2_B1 = 0x0B
    OCAL2_B0 = 0x0C
    GCAL2_B3 = 0x0D
    GCAL2_B2 = 0x0E
    GCAL2_B1 = 0x0F
    GCAL2_B0 = 0x10
    I2C_CONTROL = 0x11
    ADCO_B2 = 0x12
    ADCO_B1 = 0x13
    ADCO_B0 = 0x14
    ADC = 0x15  # //Shared ADC and OTP 32:24
    OTP_B1 = 0x15  # OTP 23:16 or 7:0?
    OTP_B0 = 0x16  # OTP 15:8
    PGA = 0x1B
    PGA_PWR = 0x1C
    DEVICE_REV = 0x1F


# Bits within the PU_CTRL register
class PUCTRLBits(Enum):
    PU_CTRL_RR = 0
    PU_CTRL_PUD = 1
    PU_CTRL_PUA = 2
    PU_CTRL_PUR = 3
    PU_CTRL_CS = 4
    PU_CTRL_CR = 5
    PU_CTRL_OSCS = 6
    PU_CTRL_AVDDS = 7


# Bits within the CTRL1 register
class CTRL1Bits(Enum):
    CTRL1_GAIN = 2
    CTRL1_VLDO = 5
    CTRL1_DRDY_SEL = 6
    CTRL1_CRP = 7


# Bits within the CTRL2 register
class CTRL2Bits(Enum):
    CTRL2_CALMOD = 0
    CTRL2_CALS = 2
    CTRL2_CAL_ERROR = 3
    CTRL2_CRS = 4
    CTRL2_CHS = 7


# Bits within the PGA register
class PGABits(Enum):
    PGA_CHP_DIS = 0
    PGA_INV = 3
    PGA_BYPASS_EN = 4
    PGA_OUT_EN = 5
    PGA_LDOMODE = 6
    PGA_RD_OTP_SEL = 7


# Bits within the PGA PWR register
class PGAPWRBits(Enum):
    PGA_PWR_PGA_CURR = 0
    PGA_PWR_ADC_CURR = 2
    PGA_PWR_MSTR_BIAS_CURR = 4
    PGA_PWR_PGA_CAP_EN = 7


# Allowed Low drop out regulator voltages
class LDOValues(Enum):
    LDO_2V4 = 0b111
    LDO_2V7 = 0b110
    LDO_3V0 = 0b101
    LDO_3V3 = 0b100
    LDO_3V6 = 0b011
    LDO_3V9 = 0b010
    LDO_4V2 = 0b001
    LDO_4V5 = 0b000


# Allowed gains
class GainValues(Enum):
    GAIN_128 = 0b111
    GAIN_64 = 0b110
    GAIN_32 = 0b101
    GAIN_16 = 0b100
    GAIN_8 = 0b011
    GAIN_4 = 0b010
    GAIN_2 = 0b001
    GAIN_1 = 0b000


# Allowed samples per second
class SPSValues(Enum):
    SPS_320 = 0b111
    SPS_80 = 0b011
    SPS_40 = 0b010
    SPS_20 = 0b001
    SPS_10 = 0b000


# Select between channel values
class Channels(Enum):
    CHANNEL_1 = 0
    CHANNEL_2 = 1


# Calibration state
class CalStatus(Enum):
    CAL_SUCCESS = 0
    CAL_IN_PROGRESS = 1
    CAL_FAILURE = 2


class NAU7802:

    _i2c_port = None
    _DEVICE_ADDRESS = 0x2A  # Default unshifted 7-bit address of the NAU7802

    # y = mx + b
    # this is b
    _zero_offset = None  # ctypes.c_int32
    # this is m. User provides this number so that we can output y when requested
    _calibration_factor = None  # float

    def __init__(self):
        pass

    def begin(self) -> bool:
        pass

    def is_connected(self) -> bool:
        pass

    def available(self) -> bool:
        pass

    def get_reading(self) -> ctypes.c_int32:
        return ctypes.c_int32(0)

    def get_average(self, samples_to_take: ctypes.c_uint8) -> ctypes.c_int32:
        return ctypes.c_int32(0)

    def calculate_zero_offset(self, average_amount: ctypes.c_uint8 = 8):
        pass

    def set_calibration_factor(self, cal_factor):
        pass

    def get_callibration_factor(self):
        pass

    def get_weight(self, allowNegativeWeights: bool = False, samplesToTake: ctypes.c_uint8 = 8) \
            -> float:
        pass

    def set_gain(self, gain_value: ctypes.c_uint8) -> bool:
        return True

    def set_LDO(self, ldo_value: ctypes.c_uint8) -> bool:
        return True

    def setSampleRate(self):
        pass

    def setChannel(self):
        pass

    def calibrateAFE(self):
        pass

    def beginCalibrateAFE(self):
        pass

    def waitForCalibrateAFE(self):
        pass

    def calAFEStatus(self):
        pass

    def reset(self):
        pass

    def powerUp(self):
        pass

    def powerDown(self):
        pass

    def setIntPolarityHigh(self):
        pass

    def setIntPolarityLow(self):
        pass

    def getRevisionCode(self):
        pass

    def setBit(self):
        pass

    def clearBit(self):
        pass

    def getBit(self):
        pass

    def getRegister(self):
        pass

    def setRegister(self):
        pass
