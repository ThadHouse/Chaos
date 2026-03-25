package frc.utils;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import org.wpilib.hardware.bus.I2C;

public class OctoQuadV3 {
    private final I2C i2c;

    private int encoderDirections = 0;

    public static class EncoderData {
        public int[] positions = new int[8];
        public short[] velocities = new short[8];
    }

    public OctoQuadV3(I2C.Port port) {
        i2c = new I2C(port, 0x30);

        byte chipId = getChipId();

        if (chipId != 0x51) {
            System.out.println("Invalid chipid: " + chipId);
        }
    }

    public byte[] readRegister(int register, int count) {
        byte[] val = new byte[count];
        if (i2c.read(register, count, val)) {
            System.out.println("Error reading register:" + register);
            return null;
        }

        return val;
    }

    public boolean writeRegister(int register, byte[] data) {
        byte[] newData = new byte[data.length + 1];
        for (int i = 0; i < data.length; i++) {
            newData[i + 1] = data[i];
        }
        newData[0] = (byte) register;
        if (i2c.writeBulk(newData)) {
            System.out.println("Error writing register:" + register);
            return false;
        }
        return true;
    }

    public byte getChipId() {
        byte[] readVal = readRegister(0, 1);
        if (readVal != null) {
            return readVal[0];
        }
        return 0;
    }

    public boolean resetAllPositions() {
        return writeRegister(0x04, new byte[] { (byte) 21, (byte) 0xFF });
    }

    public boolean setDirection(int channel, boolean reversed) {
        if (reversed) {
            encoderDirections |= (1 << channel);
        } else {
            encoderDirections &= ~(1 << channel);
        }

        return writeRegister(0x04, new byte[] { (byte) 1, (byte) 0, (byte) encoderDirections });
    }

    public boolean readAllDataWithoutLocalizer(EncoderData toFill) {
        int numRead = (8 * 4) + (8 * 2);

        byte[] data = new byte[numRead];

        if (i2c.read(0x1C, numRead, data)) {
            System.out.println("Failed to read data");
            return false;
        }

        ByteBuffer buffer = ByteBuffer.wrap(data);
        buffer.order(ByteOrder.LITTLE_ENDIAN);

        for (int i = 0; i < 8; i++) {
            toFill.positions[i] = buffer.getInt();
        }

        for (int i = 0; i < 8; i++) {
            toFill.velocities[i] = buffer.getShort();
        }

        return true;
    }
}
