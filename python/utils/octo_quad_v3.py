# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import struct
import wpilib


class OctoQuadV3:
    """Driver for OctoQuad V3 multi-encoder I2C device (address: 0x30)."""

    class EncoderData:
        def __init__(self):
            self.positions = [0] * 8
            self.velocities = [0] * 8

    def __init__(self, port):
        self._i2c = wpilib.I2C(port, 0x30)
        self._encoder_directions = 0

        chip_id = self.getChipId()
        if chip_id != 0x51:
            print(f"Invalid chipid: {chip_id}")

    def readRegister(self, register, count):
        data = bytearray(count)
        if self._i2c.read(register, count, data):
            print(f"Error reading register: {register}")
            return None
        return data

    def writeRegister(self, register, data):
        new_data = bytearray(len(data) + 1)
        new_data[0] = register
        new_data[1:] = data
        if self._i2c.writeBulk(new_data):
            print(f"Error writing register: {register}")
            return False
        return True

    def getChipId(self):
        read_val = self.readRegister(0, 1)
        if read_val is not None:
            return read_val[0]
        return 0

    def resetAllPositions(self):
        return self.writeRegister(0x04, bytes([21, 0xFF]))

    def setDirection(self, channel, reversed):
        if reversed:
            self._encoder_directions |= (1 << channel)
        else:
            self._encoder_directions &= ~(1 << channel)
        return self.writeRegister(0x04, bytes([1, 0, self._encoder_directions]))

    def readAllDataWithoutLocalizer(self, to_fill):
        num_read = (8 * 4) + (8 * 2)
        data = bytearray(num_read)

        if self._i2c.read(0x1C, num_read, data):
            print("Failed to read data")
            return False

        offset = 0
        for i in range(8):
            to_fill.positions[i] = struct.unpack_from('<i', data, offset)[0]
            offset += 4

        for i in range(8):
            to_fill.velocities[i] = struct.unpack_from('<h', data, offset)[0]
            offset += 2

        return True
