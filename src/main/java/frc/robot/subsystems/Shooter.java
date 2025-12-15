package frc.robot.subsystems;

import org.wpilib.command3.Mechanism;
import org.wpilib.epilogue.Logged;
import org.wpilib.epilogue.NotLogged;
import org.wpilib.hardware.expansionhub.ExpansionHubMotor;
import org.wpilib.hardware.expansionhub.ExpansionHubServo;
import org.wpilib.units.measure.Current;

import frc.robot.Constants.ShooterConstants;

@Logged
public class Shooter extends Mechanism {

    @NotLogged
    private ExpansionHubMotor m_shooterMotor = new ExpansionHubMotor(1, ShooterConstants.kShooterMotorPort);

    @NotLogged
    private ExpansionHubServo m_leftFeederServo = new ExpansionHubServo(1, 0);

    @NotLogged
    private ExpansionHubServo m_rightFeederServo = new ExpansionHubServo(1, 2);

    /** Creates a new Shooter. */
    public Shooter() {
        m_shooterMotor.setReversed(true);
        m_shooterMotor.setDistancePerCount(ShooterConstants.kEncoderDistancePerPulse);
        m_shooterMotor.setEnabled(true);

        m_leftFeederServo.setContinousRotationMode(true);
        m_rightFeederServo.setContinousRotationMode(true);

        m_leftFeederServo.setReversed(true);

        m_leftFeederServo.setEnabled(true);
        m_rightFeederServo.setEnabled(true);

        var pidConstants = m_shooterMotor.getVelocityPidConstants();
        pidConstants.setPID(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
        pidConstants.setFF(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);
    }

    public double getShooterVelocity() {
        return m_shooterMotor.getEncoderVelocity();
    }

    public double getShooterPosition() {
        return m_shooterMotor.getEncoderPosition();
    }

    public Current getShooterCurrent() {
        return m_shooterMotor.getCurrent();
    }

    public boolean isHubConnected() {
        return m_shooterMotor.isHubConnected();
    }

    // @NotLogged
    // private Voltage m_lastVoltage = Volts.of(0);

    // private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
    //         new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(voltage -> {
    //             m_shooterMotor.setVoltage(voltage);
    //             m_lastVoltage = voltage;
    //         }, log -> {
    //             log.motor("shooter-wheel")
    //                     .voltage(m_lastVoltage)
    //                     .angularPosition(Rotations.of(getShooterPosition()))
    //                     .angularVelocity(RotationsPerSecond.of(getShooterVelocity()));
    //         }, this));

    private double m_lastSpeed = 0.0;

    public void setSpeed(double speed) {
        m_lastSpeed = speed;
        m_shooterMotor.setVelocitySetpoint(speed);
    }

    public void setFeed(boolean feed) {
        var error = getShooterVelocity() - m_lastSpeed;
        if (Math.abs(error) > 2) {
            feed = false;
        }

        if (feed) {
            m_leftFeederServo.set(1.0);
            m_rightFeederServo.set(1.0);
        } else {
            m_leftFeederServo.set(0);
            m_rightFeederServo.set(0);
        }
    }

    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return m_sysIdRoutine.quasistatic(direction);
    // }

    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return m_sysIdRoutine.dynamic(direction);
    // }
}
