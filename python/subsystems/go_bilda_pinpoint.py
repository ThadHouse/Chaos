# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import struct
import wpilib


class GoBildaPinpoint:
    """Driver for the GoBILDA Pinpoint Odometry Computer (I2C address: 0x31)."""

    _GOBILDA_SWINGARM_POD = 13.26291192  # ticks-per-mm for the goBILDA Swingarm Pod
    _GOBILDA_4_BAR_POD = 19.89436789    # ticks-per-mm for the goBILDA 4-Bar Pod

    DEFAULT_ADDRESS = 0x31

    class _Register:
        DEVICE_ID = 1
        DEVICE_VERSION = 2
        DEVICE_STATUS = 3
        DEVICE_CONTROL = 4
        LOOP_TIME = 5
        X_ENCODER_VALUE = 6
        Y_ENCODER_VALUE = 7
        X_POSITION = 8
        Y_POSITION = 9
        H_ORIENTATION = 10
        X_VELOCITY = 11
        Y_VELOCITY = 12
        H_VELOCITY = 13
        MM_PER_TICK = 14
        X_POD_OFFSET = 15
        Y_POD_OFFSET = 16
        YAW_SCALAR = 17
        BULK_READ = 18

    class DeviceStatus:
        NOT_READY = 0
        READY = 1
        CALIBRATING = 1 << 1
        FAULT_X_POD_NOT_DETECTED = 1 << 2
        FAULT_Y_POD_NOT_DETECTED = 1 << 3
        FAULT_NO_PODS_DETECTED = (1 << 2) | (1 << 3)
        FAULT_IMU_RUNAWAY = 1 << 4

    class EncoderDirection:
        FORWARD = "FORWARD"
        REVERSED = "REVERSED"

    class GoBildaOdometryPods:
        goBILDA_SWINGARM_POD = "goBILDA_SWINGARM_POD"
        goBILDA_4_BAR_POD = "goBILDA_4_BAR_POD"

    class ReadData:
        ONLY_UPDATE_HEADING = "ONLY_UPDATE_HEADING"

    def __init__(self, port):
        self._i2c = wpilib.I2C(port, self.DEFAULT_ADDRESS)

        self._device_status = 0
        self._loop_time = 0
        self._x_encoder_value = 0
        self._y_encoder_value = 0
        self._x_position = 0.0
        self._y_position = 0.0
        self._h_orientation = 0.0
        self._x_velocity = 0.0
        self._y_velocity = 0.0
        self._h_velocity = 0.0

    def _write_int(self, reg, value):
        """Writes an int to the i2c device."""
        data = bytearray(5)
        data[0] = reg
        struct.pack_into('<i', data, 1, value)
        if self._i2c.writeBulk(data):
            wpilib.DriverStation.reportError("Failed to write register", False)

    def _read_int(self, reg):
        """Reads an int from a register of the i2c device."""
        data = bytearray(4)
        if self._i2c.read(reg, 4, data):
            wpilib.DriverStation.reportError("Failed to read register", False)
            return 0
        return struct.unpack_from('<i', data, 0)[0]

    def _read_float(self, reg):
        """Reads a float from a register."""
        data = bytearray(4)
        if self._i2c.read(reg, 4, data):
            wpilib.DriverStation.reportError("Failed to read register", False)
            return 0.0
        return struct.unpack_from('<f', data, 0)[0]

    def _write_float(self, reg, value):
        """Writes a float to a register on the i2c device."""
        data = bytearray(5)
        data[0] = reg
        struct.pack_into('<f', data, 1, value)
        if self._i2c.writeBulk(data):
            wpilib.DriverStation.reportError("Failed to write register", False)

    def _lookup_status(self, s):
        """Looks up the DeviceStatus corresponding with an int value."""
        ds = self.DeviceStatus
        if (s & ds.CALIBRATING) != 0:
            return ds.CALIBRATING
        x_pod_detected = (s & ds.FAULT_X_POD_NOT_DETECTED) == 0
        y_pod_detected = (s & ds.FAULT_Y_POD_NOT_DETECTED) == 0

        if not x_pod_detected and not y_pod_detected:
            return ds.FAULT_NO_PODS_DETECTED
        if not x_pod_detected:
            return ds.FAULT_X_POD_NOT_DETECTED
        if not y_pod_detected:
            return ds.FAULT_Y_POD_NOT_DETECTED
        if (s & ds.FAULT_IMU_RUNAWAY) != 0:
            return ds.FAULT_IMU_RUNAWAY
        if (s & ds.READY) != 0:
            return ds.READY
        return ds.NOT_READY

    def update(self, data=None):
        """
        Call this once per loop to read new data from the Odometry Computer.
        Data will only update once this is called.

        :param data: Optional ReadData.ONLY_UPDATE_HEADING for a narrow read.
        """
        if data == self.ReadData.ONLY_UPDATE_HEADING:
            buf = bytearray(4)
            if self._i2c.read(self._Register.H_ORIENTATION, 4, buf):
                wpilib.DriverStation.reportError("Failed to read heading register", False)
                return
            self._h_orientation = struct.unpack_from('<f', buf, 0)[0]
            return

        buf = bytearray(40)
        if self._i2c.read(self._Register.BULK_READ, 40, buf):
            return

        offset = 0
        self._device_status = struct.unpack_from('<i', buf, offset)[0]; offset += 4
        self._loop_time = struct.unpack_from('<i', buf, offset)[0]; offset += 4
        self._x_encoder_value = struct.unpack_from('<i', buf, offset)[0]; offset += 4
        self._y_encoder_value = struct.unpack_from('<i', buf, offset)[0]; offset += 4
        self._x_position = struct.unpack_from('<f', buf, offset)[0]; offset += 4
        self._y_position = struct.unpack_from('<f', buf, offset)[0]; offset += 4
        self._h_orientation = struct.unpack_from('<f', buf, offset)[0]; offset += 4
        self._x_velocity = struct.unpack_from('<f', buf, offset)[0]; offset += 4
        self._y_velocity = struct.unpack_from('<f', buf, offset)[0]; offset += 4
        self._h_velocity = struct.unpack_from('<f', buf, offset)[0]; offset += 4

    def setOffsets(self, x_offset_mm, y_offset_mm):
        """
        Sets the odometry pod positions relative to the point the odometry
        computer tracks around.

        :param x_offset_mm: How sideways (mm) from tracking point the X (forward) pod is.
        :param y_offset_mm: How far forwards (mm) from tracking point the Y (strafe) pod is.
        """
        self._write_float(self._Register.X_POD_OFFSET, float(x_offset_mm))
        self._write_float(self._Register.Y_POD_OFFSET, float(y_offset_mm))

    def recalibrateIMU(self):
        """Recalibrates the Odometry Computer's internal IMU. Robot MUST be stationary."""
        self._write_int(self._Register.DEVICE_CONTROL, 1 << 0)

    def resetPosAndIMU(self):
        """
        Resets the current position to 0,0,0 and recalibrates the IMU.
        Robot MUST be stationary.
        """
        self._write_int(self._Register.DEVICE_CONTROL, 1 << 1)

    def setEncoderDirections(self, x_encoder, y_encoder):
        """
        Can reverse the direction of each encoder.

        :param x_encoder: EncoderDirection.FORWARD or REVERSED for X (forward) pod.
        :param y_encoder: EncoderDirection.FORWARD or REVERSED for Y (strafe) pod.
        """
        if x_encoder == self.EncoderDirection.FORWARD:
            self._write_int(self._Register.DEVICE_CONTROL, 1 << 5)
        if x_encoder == self.EncoderDirection.REVERSED:
            self._write_int(self._Register.DEVICE_CONTROL, 1 << 4)

        if y_encoder == self.EncoderDirection.FORWARD:
            self._write_int(self._Register.DEVICE_CONTROL, 1 << 3)
        if y_encoder == self.EncoderDirection.REVERSED:
            self._write_int(self._Register.DEVICE_CONTROL, 1 << 2)

    def setEncoderResolution(self, pods_or_ticks_per_mm):
        """
        Sets the encoder resolution.

        :param pods_or_ticks_per_mm: A GoBildaOdometryPods value or a float (ticks/mm).
        """
        if pods_or_ticks_per_mm == self.GoBildaOdometryPods.goBILDA_SWINGARM_POD:
            self._write_float(self._Register.MM_PER_TICK, self._GOBILDA_SWINGARM_POD)
        elif pods_or_ticks_per_mm == self.GoBildaOdometryPods.goBILDA_4_BAR_POD:
            self._write_float(self._Register.MM_PER_TICK, self._GOBILDA_4_BAR_POD)
        else:
            self._write_float(self._Register.MM_PER_TICK, float(pods_or_ticks_per_mm))

    def setYawScalar(self, yaw_offset):
        """
        Sets a scalar applied to the gyro's yaw value.

        :param yaw_offset: A scalar for the robot's heading.
        """
        self._write_float(self._Register.YAW_SCALAR, float(yaw_offset))

    def setPosition(self, pose):
        """
        Sends a position that the Pinpoint should use to track the robot relative to.

        :param pose: A Pose2d describing the robot's new position.
        :return: The pose that was set.
        """
        from wpimath.geometry import Pose2d
        self._write_float(self._Register.X_POSITION, float(pose.X() * 1000))   # convert m -> mm
        self._write_float(self._Register.Y_POSITION, float(pose.Y() * 1000))   # convert m -> mm
        self._write_float(self._Register.H_ORIENTATION, float(pose.rotation().radians()))
        return pose

    def getDeviceID(self):
        """Checks the deviceID of the Odometry Computer. Should return 1."""
        return self._read_int(self._Register.DEVICE_ID)

    def getDeviceVersion(self):
        """Returns the firmware version of the Odometry Computer."""
        return self._read_int(self._Register.DEVICE_VERSION)

    def getYawScalar(self):
        return self._read_float(self._Register.YAW_SCALAR)

    def getDeviceStatus(self):
        """Returns the device status, indicating any faults."""
        return self._lookup_status(self._device_status)

    def getLoopTime(self):
        """
        Returns the Odometry Computer's most recent loop time in microseconds.
        Normal values: 500 - 1100 µs.
        """
        return self._loop_time

    def getFrequency(self):
        """
        Returns the Odometry Computer's loop frequency in Hz.
        Normal values: 900 - 2000 Hz.
        """
        if self._loop_time != 0:
            return 1_000_000.0 / self._loop_time
        return 0.0

    def getEncoderX(self):
        """Returns the raw value of the X (forward) encoder in ticks."""
        return self._x_encoder_value

    def getEncoderY(self):
        """Returns the raw value of the Y (strafe) encoder in ticks."""
        return self._y_encoder_value

    def getPosX(self):
        """Returns the estimated X (forward) position of the robot in meters."""
        return self._x_position / 1000.0  # mm -> m

    def getPosY(self):
        """Returns the estimated Y (strafe) position of the robot in meters."""
        return self._y_position / 1000.0  # mm -> m

    def getHeading(self):
        """Returns the estimated heading of the robot as a Rotation2d."""
        from wpimath.geometry import Rotation2d
        return Rotation2d(self._h_orientation)

    def getVelX(self):
        """Returns the estimated X (forward) velocity of the robot in m/s."""
        return self._x_velocity / 1000.0  # mm/s -> m/s

    def getVelY(self):
        """Returns the estimated Y (strafe) velocity of the robot in m/s."""
        return self._y_velocity / 1000.0  # mm/s -> m/s

    def getHeadingVelocity(self):
        """Returns the estimated heading velocity of the robot in rad/s."""
        return self._h_velocity

    def getXOffset(self):
        """Returns the user-set offset for the X (forward) pod in meters."""
        return self._read_float(self._Register.X_POD_OFFSET) / 1000.0  # mm -> m

    def getYOffset(self):
        """Returns the user-set offset for the Y (strafe) pod in meters."""
        return self._read_float(self._Register.Y_POD_OFFSET) / 1000.0  # mm -> m

    def getPosition(self):
        """Returns a Pose2d containing the estimated position of the robot."""
        from wpimath.geometry import Pose2d
        return Pose2d(self.getPosX(), self.getPosY(), self.getHeading())
