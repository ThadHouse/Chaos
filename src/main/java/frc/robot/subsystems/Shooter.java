package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utils.ExpansionHubMotor;
import frc.utils.ExpansionHubServo;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ShooterConstants;

@Logged
public class Shooter extends SubsystemBase {

    @NotLogged
    ExpansionHubMotor m_shooterMotor = new ExpansionHubMotor(1, ShooterConstants.kShooterMotorPort);

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

        m_shooterFeedback.setTolerance(3);
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
    private Voltage m_lastVoltage = Volts.of(0);

    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(voltage -> {
                m_shooterMotor.setVoltage(voltage);
                m_lastVoltage = voltage;
            }, log -> {
                log.motor("shooter-wheel")
                        .voltage(m_lastVoltage)
                        .angularPosition(Rotations.of(getShooterPosition()))
                        .angularVelocity(RotationsPerSecond.of(getShooterVelocity()));
            }, this));

    @NotLogged
    private final PIDController m_shooterFeedback = new PIDController(ShooterConstants.kP, ShooterConstants.kI,
            ShooterConstants.kD);

    @NotLogged
    private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(ShooterConstants.kS,
            ShooterConstants.kV, ShooterConstants.kA);

    public void setSpeed(double speed) {
        m_shooterMotor.setVoltage(Volts.of(m_shooterFeedback.calculate(getShooterVelocity(), speed)
                + m_shooterFeedforward.calculate(speed)));
    }

    public void setFeed(boolean feed) {

        if (feed && m_shooterFeedback.atSetpoint()) {
            m_leftFeederServo.set(1.0);
            m_rightFeederServo.set(1.0);
        } else {
            m_leftFeederServo.set(0);
            m_rightFeederServo.set(0);
        }
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}
