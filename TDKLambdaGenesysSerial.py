# -*- coding: utf-8 -*-
#
# This file is part of the TDKLambdaGenesysSerial project
#
#
#
# Distributed under the terms of the MIT license.
# See LICENSE.txt for more info.

""" TDK Lambda Genesys Power Supply

Class to control the TDK Lambda Genesys Power supply line via serial communication.
"""

# PyTango imports
import tango
from tango import DebugIt
from tango.server import run
from tango.server import Device
from tango.server import attribute, command
from tango.server import device_property
from tango import AttrQuality, DispLevel, DevState
from tango import AttrWriteType, PipeWriteType
import enum
# Additional import
# PROTECTED REGION ID(TDKLambdaGenesysSerial.additionnal_import) ENABLED START #
import pyvisa
from threading import Timer, Lock
import time
import numpy as np
import sys
# PROTECTED REGION END #    //  TDKLambdaGenesysSerial.additionnal_import

__all__ = ["TDKLambdaGenesysSerial", "main"]


class Limit(enum.IntEnum):
    """Python enumerated type for Limit attribute."""
    OFF = 0
    CV = 1
    CC = 2


class RampMode(enum.IntEnum):
    OFF = 0
    UP = 1
    DOWN = 2
    UPDOWN = 3


class RepeatTimer(Timer):
    def run(self):
        while not self.finished.wait(self.interval):
            self.function(*self.args, **self.kwargs)


class TDKLambdaGenesysSerial(Device):
    """
    Class to control the TDK Lambda Genesys Power supply line via serial communication.

    **Properties:**

    - Device Property
        dev_adr
            - Serial line device address (for daisy chaining)
            - Type:'DevLong'
        visa_resource
            - pyvisa resource name for connection
            - Type:'DevString'
    """
    # PROTECTED REGION ID(TDKLambdaGenesysSerial.class_variable) ENABLED START #
    # PROTECTED REGION END #    //  TDKLambdaGenesysSerial.class_variable

    # -----------------
    # Device Properties
    # -----------------

    dev_adr = device_property(
        dtype='DevLong',
        default_value=1
    )

    visa_resource = device_property(
        dtype='DevString',
        default_value="ASRL/dev/ttyUSB0::INSTR"
    )

    baud_rate = device_property(
        doc="Baud rate - only used for serial connection.",
        dtype=int,
        default_value=9600,
    )

    # ----------
    # Attributes
    # ----------

    current = attribute(
        dtype='DevDouble',
        unit="A",
        min_value=0,
    )

    current_limit = attribute(
        dtype='DevDouble',
        access=AttrWriteType.READ_WRITE,
        label="current limit",
        unit="A",
        min_value=0,
    )

    limit = attribute(
        dtype=Limit,
        label="control limit",
        doc="Indicates whether output is limited by voltage or current limit (or disabled)",
    )

    output = attribute(
        dtype='DevBoolean',
        access=AttrWriteType.READ_WRITE,
        label="output enable",
    )

    voltage = attribute(
        dtype='DevDouble',
        unit="V",
        min_value=0,
    )

    voltage_limit = attribute(
        dtype='DevDouble',
        access=AttrWriteType.READ_WRITE,
        label="voltage limit",
        unit="V",
        min_value=0,
    )

    ramp_mode = attribute(
        dtype=RampMode,
        label="ramp mode",
        access=AttrWriteType.READ_WRITE,
        doc="When to use setpoint ramping",
    )

    ramp_rate = attribute(
        dtype='DevDouble',
        access=AttrWriteType.READ_WRITE,
        unit="A/s",
        min_value = 0,
    )

    # ---------------
    # General methods
    # ---------------

    def init_device(self):
        """Initialises the attributes and properties of the TDKLambdaGenesysSerial."""
        Device.init_device(self)
        # PROTECTED REGION ID(TDKLambdaGenesysSerial.init_device) ENABLED START #
        self._current = 0.0
        self._output = False
        self._voltage = 0.0
        self._is_ramping = False
        self._ramp_mode = RampMode.UP
        self._ramp_rate = 10
        self._ramp_t0 = 0
        self._ramp_i0 = 0
        self._ramp_setpoint = 0
        self._hw_setpoint = 0
        self._serial_lock = Lock()
        try:
            self.rm = pyvisa.ResourceManager("@py")
            self.inst = self.rm.open_resource(self.visa_resource)
            if "SRL" in self.visa_resource:
                self.inst.baud_rate = self.baud_rate
            self.inst.read_termination = '\r'
            self.inst.write_termination = '\r'
            self.inst.timeout = 500
            ans = self.inst.query(f"ADR {self.dev_adr}")
            idn = self.inst.query("IDN?")
            self.debug_stream(f"Connection ok: {idn}")
            self.set_state(DevState.ON)
            self._ramp_loop = RepeatTimer(0.3, self._ramp_task)
            self._ramp_loop.start()
        except Exception as ex:
            self.error_stream(str(ex))
            self.set_state(DevState.OFF)
            sys.exit(1)
        # PROTECTED REGION END #    //  TDKLambdaGenesysSerial.init_device

    def always_executed_hook(self):
        """Method always executed before any TANGO command is executed."""
        # PROTECTED REGION ID(TDKLambdaGenesysSerial.always_executed_hook) ENABLED START #
        if self._is_ramping:
            self.set_state(DevState.MOVING)
        elif self._output:
            self.set_state(DevState.RUNNING)
        else:
            self.set_state(DevState.ON)
        # PROTECTED REGION END #    //  TDKLambdaGenesysSerial.always_executed_hook

    def delete_device(self):
        """Hook to delete resources allocated in init_device.

        This method allows for any memory or other resources allocated in the
        init_device method to be released.  This method is called by the device
        destructor and by the device Init command.
        """
        # PROTECTED REGION ID(TDKLambdaGenesysSerial.delete_device) ENABLED START #
        self.inst.close()
        self.rm.close()
        # PROTECTED REGION END #    //  TDKLambdaGenesysSerial.delete_device
    # ------------------
    # Attributes methods
    # ------------------

    def read_current(self):
        # PROTECTED REGION ID(TDKLambdaGenesysSerial.current_read) ENABLED START #
        """Return the current attribute."""
        ans = self.query("MC?")
        return float(ans)
        # PROTECTED REGION END #    //  TDKLambdaGenesysSerial.current_read

    def read_current_limit(self):
        # PROTECTED REGION ID(TDKLambdaGenesysSerial.current_limit_read) ENABLED START #
        """Return the current_limit attribute."""
        ans = self.query("PC?")
        self._hw_setpoint = float(ans)
        return self._hw_setpoint
        # PROTECTED REGION END #    //  TDKLambdaGenesysSerial.current_limit_read

    def write_current_limit(self, value):
        """Set the current_limit attribute."""
        self._ramp_setpoint = value
        self._ramp_t0 = time.time()
        self._ramp_i0 = self._hw_setpoint

    def _write_current_limit(self, value):
        """Used to directly set current setpoint without aborting ramp"""
        self.query(f"PC {value}")

    def read_limit(self):
        # PROTECTED REGION ID(TDKLambdaGenesysSerial.limit_read) ENABLED START #
        """Return the limit attribute."""
        ans = self.query("MODE?")
        return Limit[ans]
        # PROTECTED REGION END #    //  TDKLambdaGenesysSerial.limit_read

    def read_output(self):
        # PROTECTED REGION ID(TDKLambdaGenesysSerial.output_read) ENABLED START #
        """Return the output attribute."""
        self._output = self.query("OUT?") == "ON"
        return self._output
        # PROTECTED REGION END #    //  TDKLambdaGenesysSerial.output_read

    def write_output(self, value):
        # PROTECTED REGION ID(TDKLambdaGenesysSerial.output_write) ENABLED START #
        """Set the output attribute."""
        v = "ON" if value else "OFF"
        self.query(f"OUT {v}")
        self.set_state(DevState.RUNNING if value else DevState.ON)

        # PROTECTED REGION END #    //  TDKLambdaGenesysSerial.output_write

    def read_voltage(self):
        # PROTECTED REGION ID(TDKLambdaGenesysSerial.voltage_read) ENABLED START #
        """Return the voltage attribute."""
        ans = self.query("MV?")
        return float(ans)
        # PROTECTED REGION END #    //  TDKLambdaGenesysSerial.voltage_read

    def read_voltage_limit(self):
        # PROTECTED REGION ID(TDKLambdaGenesysSerial.voltage_limit_read) ENABLED START #
        """Return the voltage_limit attribute."""
        ans = self.query("PV?")
        return float(ans)
        # PROTECTED REGION END #    //  TDKLambdaGenesysSerial.voltage_limit_read

    def write_voltage_limit(self, value):
        # PROTECTED REGION ID(TDKLambdaGenesysSerial.voltage_limit_write) ENABLED START #
        """Set the voltage_limit attribute."""
        self.query(f"PV {value}")
        # PROTECTED REGION END #    //  TDKLambdaGenesysSerial.voltage_limit_write

    def read_ramp_rate(self):
        return self._ramp_rate

    def write_ramp_rate(self, value):
        self._ramp_rate = value

    def read_ramp_mode(self):
        return self._ramp_mode

    def write_ramp_mode(self, value):
        self._ramp_mode = value
    # --------
    # Commands
    # --------


    @command(
        dtype_in='DevString',
        doc_in="cmd",
        dtype_out='DevString',
        doc_out="ans",
    )
    # @DebugIt()
    def query(self, argin):
        # PROTECTED REGION ID(TDKLambdaGenesysSerial.query) ENABLED START #
        """
        Send query to device and return reply

        :param argin: 'DevString'
        cmd

        :return:'DevString'
        ans
        """
        with self._serial_lock:
            ans = self.inst.query(argin)
        self.debug_stream(f"{argin} -> {ans}")
        return ans
        # PROTECTED REGION END #    //  TDKLambdaGenesysSerial.query

    def _ramp_task(self):
        """persistent task that updates current setpoints according to ramp settings."""
        if self._ramp_setpoint != self._hw_setpoint:
            up = self._ramp_setpoint > self._hw_setpoint
            do_ramp = (
                (self._ramp_mode == RampMode.UP and up)
                or (self._ramp_mode == RampMode.DOWN and (not up))
                or (self._ramp_mode == RampMode.UPDOWN)
            )
            if do_ramp:
                # time.sleep(0.4)
                self._is_ramping = True
                t = time.time() - self._ramp_t0
                tmax = abs(self._ramp_setpoint - self._ramp_i0) / self._ramp_rate
                sign = 1 if up else -1
                new_setp = self._ramp_i0 + sign * self._ramp_rate * t
                new_setp = self._ramp_setpoint if t >= tmax else new_setp
            else:
                self._is_ramping = False
                new_setp = self._ramp_setpoint
            self._write_current_limit(new_setp)
        else:
            self._is_ramping = False


# ----------
# Run server
# ----------


def main(args=None, **kwargs):
    """Main function of the TDKLambdaGenesysSerial module."""
    # PROTECTED REGION ID(TDKLambdaGenesysSerial.main) ENABLED START #
    return TDKLambdaGenesysSerial.run_server()
    # PROTECTED REGION END #    //  TDKLambdaGenesysSerial.main


if __name__ == '__main__':
    main()
