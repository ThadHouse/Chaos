package frc.robot.subsystems;

import static org.wpilib.units.Units.Seconds;

import org.wpilib.command3.Command;
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

        this.setDefaultCommand(this.runRepeatedly(() -> {
            setSpeed(0);
            setFeed(false);
        }).withPriority(Command.LOWEST_PRIORITY).named("Default Shooter"));
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

    @NotLogged
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

    public Command getSpinCommand() {
        return this.runRepeatedly(() -> {
            setSpeed(40);
            setFeed(false);
        }).withPriority(Command.DEFAULT_PRIORITY).named("Spin Shooter");
    }

    public Command getSpinAndFeedCommand() {
        return this.runRepeatedly(() -> {
            setSpeed(40);
            setFeed(true);
        }).withPriority(Command.DEFAULT_PRIORITY + 1).named("Spin and Feed Shooter");
    }

    public Command shootTime(double time) {
        return this.run(c -> {
            setSpeed(40);
            setFeed(true);
            c.wait(Seconds.of(time));
            setSpeed(0);
            setFeed(false);
        }).withPriority(Command.DEFAULT_PRIORITY).named("Shoot " + time);
    }
}
