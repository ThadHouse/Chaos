package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.OctoQuad.EncoderData;

public class OctoQuadEncoders {
    private final OctoQuad m_octoQuad = new OctoQuad(Port.kOnboard);

    private final EncoderData m_encoderData = new EncoderData();

    public OctoQuadEncoders() {
        m_octoQuad.setDirection(0, true);
        m_octoQuad.setDirection(1, true);
    }

    public void periodic() {
        m_octoQuad.readAllData(m_encoderData);
    }

    public double getFrontLeftPosition() {
        return m_encoderData.positions[0] * DriveConstants.kEncoderDistancePerPulse;
    }

    public double getFrontRightPosition() {
        return m_encoderData.positions[1] * DriveConstants.kEncoderDistancePerPulse;
    }

    public double getRearLeftPosition() {
        return m_encoderData.positions[2] * DriveConstants.kEncoderDistancePerPulse;
    }

    public double getRearRightPosition() {
        return m_encoderData.positions[3] * DriveConstants.kEncoderDistancePerPulse;
    }

    public double getFrontLeftRate() {
        return m_encoderData.velocities[0] * DriveConstants.kEncoderDistancePerPulse;
    }

    public double getFrontRightRate() {
        return m_encoderData.velocities[1] * DriveConstants.kEncoderDistancePerPulse;
    }

    public double getRearLeftRate() {
        return m_encoderData.velocities[2] * DriveConstants.kEncoderDistancePerPulse;
    }

    public double getRearRightRate() {
        return m_encoderData.velocities[3] * DriveConstants.kEncoderDistancePerPulse;
    }

    public void resetPositions() {
        m_octoQuad.resetAllPositions();
    }
}
