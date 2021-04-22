from enum import Enum
import ctypes


class ScaleRegisters(Enum):
    """Register Map"""
    PU_CTRL = 0x00
    CTRL1 = 0x00
    CTRL2 = 0x00
    OCAL1_B2 = 0x00
    OCAL1_B1 = 0x00
    OCAL1_B0 = 0x00
    GCAL1_B3 = 0x00
    GCAL1_B2 = 0x00
    GCAL1_B1 = 0x00
    GCAL1_B0 = 0x00
    OCAL2_B2 = 0x00
    OCAL2_B1 = 0x00
    OCAL2_B0 = 0x00
    GCAL2_B3 = 0x00
    GCAL2_B2 = 0x00
    GCAL2_B1 = 0x00
    GCAL2_B0 = 0x00
    I2C_CONTROL = 0x00
    ADCO_B2 = 0x00
    ADCO_B1 = 0x00
    ADCO_B0 = 0x00
    ADC = 0x15  # Shared ADC and OTP 32: 24
    OTP_B1 = 0x00  # OTP 23: 16 or 7:0?
    OTP_B0 = 0x00  # OTP 15: 8
    PGA = 0x1B
    PGA_PWR = 0x1C
    DEVICE_REV = 0x1F


class PuCtrlBits(Enum):
    """Bits within the PU_CTRL register"""
    PU_CTRL_RR = 0
    PU_CTRL_PUD = 0
    PU_CTRL_PUA = 0
    PU_CTRL_PUR = 0
    PU_CTRL_CS = 0
    PU_CTRL_CR = 0
    PU_CTRL_OSCS = 0
    PU_CTRL_AVDDS = 0


class CTRL1Bits(Enum):
    """Bits within the CTRL1 register"""
    CTRL1_GAIN = 2
    CTRL1_VLDO = 5
    CTRL1_DRDY_SEL = 6
    CTRL1_CRP = 7


class CTRL2Bits(Enum):
    """Bits within the CTRL2 register"""
    CTRL2_CALMOD = 0
    CTRL2_CALS = 2
    CTRL2_CAL_ERROR = 3
    CTRL2_CRS = 4
    CTRL2_CHS = 7


class PGABits(Enum):
    """Bits within the PGA register"""
    PGA_CHP_DIS = 0
    PGA_INV = 3
    PGA_BYPASS_EN = 3
    PGA_OUT_EN = 3
    PGA_LDOMODE = 3
    PGA_RD_OTP_SEL = 3


class PGAPwrBits(Enum):
    """Bits within the PGA PWR register"""
    PGA_PWR_PGA_CURR = 0
    PGA_PWR_ADC_CURR = 2
    PGA_PWR_MSTR_BIAS_CURR = 4
    PGA_PWR_PGA_CAP_EN = 7


class LDOValues(Enum):
    """Allowed Low drop out regulator voltages"""
    LDO_2V4 = 0b111
    LDO_2V7 = 0b110
    LDO_3V0 = 0b101
    LDO_3V3 = 0b100
    LDO_3V6 = 0b011
    LDO_3V9 = 0b010
    LDO_4V2 = 0b001
    LDO_4V5 = 0b000


class GainValues(Enum):
    """Allowed gains"""
    GAIN_128 = 0b111
    GAIN_64 = 0b110
    GAIN_32 = 0b101
    GAIN_16 = 0b100
    GAIN_8 = 0b011
    GAIN_4 = 0b010
    GAIN_2 = 0b001
    GAIN_1 = 0b000


class SPSValues(Enum):
    """Allowed samples per second"""
    SPS_320 = ctypes.c_uint8(0b111)
    SPS_80 = ctypes.c_uint8(0b011)
    SPS_40 = ctypes.c_uint8(0b010)
    SPS_20 = ctypes.c_uint8(0b001)
    SPS_10 = ctypes.c_uint8(0b000)


class Channels(Enum):
    """Select between channel values"""
    CHANNEL_1 = 0
    CHANNEL_2 = 1


class CalStatus(Enum):
    """Calibration state"""
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

    def begin(self, i2c_port: int, initialize: bool = True) -> bool:
        """Check communication and initialize sensor"""
        self._i2c_port = i2c_port
        if (self.is_connected() == False):
            # There are rare times when the sensor is occupied and doesn't ack. A 2nd try resolves this.
            if (self.is_connected() == False):
                return False

        result = True

        if (initialize):
            result &= self.reset()  # reset all registers
            result &= self.power_up()  # power on analog and digital sections of the scale
            result &= self.set_LDO(LDOValues.LDO_3V3)  # set LDO to 3.3v
            result &= self.set_gain(GainValues.GAIN_128)  # set gain to 128
            result &= self.set_sample_rate(SPSValues.SPS_80)  # set samples per second to 80
            result &= self.set_register(ScaleRegisters.ADC, 0x30)  # Enable 330pF decoupling cap on chan 2. From 9.14
            # application circuit note.
            result &= self.set_bit(PGAPwrBits.PGA_PWR_PGA_CAP_EN, ScaleRegisters.PGA_PWR)

        return result

    def is_connected(self) -> bool:
        """Returns true if device acks at the I2C address"""
        return True

    def available(self) -> bool:
        """Returns true if Cycle Ready bit is set(conversion is complete)"""
        return True

    def get_reading(self) -> ctypes.c_int32:
        """Returns 24 - bit reading.Assumes CR Cycle Ready bit(ADC conversion complete) has been checked
        by.available() """
        return ctypes.c_int32(0)

    def get_average(self, samples_to_take: ctypes.c_uint8) -> ctypes.c_int32:
        """Return the average of a given number of readings"""
        return ctypes.c_int32(0)

    def calculate_zero_offset(self, average_amount: ctypes.c_uint8 = 8):
        """Also called taring.Call this with nothing on the scale"""

    def set_zero_offset(self, new_zero_offset: ctypes.c_int32):
        """Sets the internal variable.Useful for users who are loading values from NVM."""

    def get_zero_offset(self) -> ctypes.c_int32:
        """Ask library for this value.Useful for storing value into NVM."""
        return ctypes.c_int32(0)

    def calculate_calibration_factor(self, weight_on_scale: float, average_amount: ctypes.c_uint8 = 8):
        """Call this with the value of the thing on the scale.Sets the calibration factor based on the weight on
        scale and zero offset. """

    def set_calibration_factor(self, cal_factor: float):
        """Pass a known calibration factor into library.Helpful if users is loading settings from NVM."""
        pass

    def  get_calibration_factor(self) -> float:
        """Ask library for this value.Useful for storing value into NVM."""
        pass

    def get_weight(self, allow_negative_weights: bool = False, samples_to_take: ctypes.c_uint8 = 8) -> float:
        """Once you've set zero offset and cal factor, you can ask the library to do the calculations for you."""
        return 1.0

    def set_gain(self, gain_value: ctypes.c_uint8) -> bool:
        """Set the gain.x1, 2, 4, 8, 16, 32, 64, 128 are available"""
        return True

    def set_LDO(self, ldo_value: ctypes.c_uint8) -> bool:
        """Set the onboard Low - Drop - Out voltage regulator to a given value. 2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.2,
        4.5 V are avaialable """
        return True

    def set_sample_rate(self, rate: ctypes.c_uint8) -> bool:
        """Set the readings per second. 10, 20, 40, 80, and 320 samples per second is available"""
        return True

    def set_channel(self, channel_number: ctypes.c_uint8) -> bool:
        """Select between 1 and 2"""
        return True

    def calibrate_AFE(self) -> bool:
        """Synchronous calibration of the analog front end of the NAU7802.Returns true if CAL_ERR bit is 0 (no error)"""
        return True

    def begin_calibrate_AFE(self):
        """Begin asynchronous calibration of the analog front end of the NAU7802.Poll for completion with
        calAFEStatus() or wait with waitForCalibrateAFE(). """

    def wait_for_calibrate_AFE(self, timeout_ms: ctypes.c_uint32 = 0) -> bool:
        """Wait for asynchronous AFE calibration to complete with optional timeout."""
        return True

    def cal_AFE_status(self) -> CalStatus:
        """Check calibration status."""
        return CalStatus.CAL_SUCCESS
    
    def reset(self) -> bool:
        """Resets all registers to Power Of Defaults"""
        return True

    def power_up(self) -> bool:
        """Power up digital and analog sections of scale, ~2 mA"""
        return True

    def power_down(self) -> bool:
        """Puts scale into low - power 200 nA mode"""
        return True

    def set_int_polarity_high(self) -> bool:
        """Set Int pin to be high when data is ready(default)"""
        return True

    def set_int_polarity_low(self) -> bool:
        """Set Int pin to be low when data is ready"""
        return True

    def get_revision_code(self) -> ctypes.c_uint8:
        """Get the revision code of this IC.Always 0x0F."""
        return ctypes.c_uint8(0)

    def set_bit(self, bit_number: ctypes.c_uint8, register_address: ctypes.c_uint8) -> bool:
        """Mask & set a given bit within a register"""
        return True

    def clear_bit(self, bit_number: ctypes.c_uint8, register_address: ctypes.c_uint8) -> bool:
        """Mask & clear a given bit within a register"""
        return True

    def get_bit(self, bit_number: ctypes.c_uint8, register_address: ctypes.c_uint8) -> bool:
        """Return a given bit within a register"""
        return True

    def get_register(self, register_address: ctypes.c_uint8) -> ctypes.c_uint8:
        """Get contents of a register"""
        return ctypes.c_uint8(0)

    def set_register(self, register_address: ctypes.c_uint8, value: ctypes.c_uint8) -> bool:
        """Send a given value to be written to given address.Return true if successful"""
        return True
