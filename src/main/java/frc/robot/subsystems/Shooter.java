package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utils.ExpansionHubServo;
import frc.utils.OctoQuadV3;
import frc.utils.OctoQuadV3.EncoderData;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.SparkMini;
import frc.robot.Constants.ShooterConstants;

@Logged
public class Shooter extends SubsystemBase {
    @NotLogged
    OctoQuadV3 m_octoQuad = new OctoQuadV3(ShooterConstants.kI2cPort);

    @NotLogged
    SparkMini m_shooterMotor = new SparkMini(ShooterConstants.kShooterMotorPort);

    @NotLogged
    private EncoderData m_encoderData = new EncoderData();

    @NotLogged
    private ExpansionHubServo m_leftFeederServo = new ExpansionHubServo(0, 0);

    @NotLogged
    private ExpansionHubServo m_rightFeederServo = new ExpansionHubServo(0, 2);

    /** Creates a new Shooter. */
    public Shooter() {
        m_shooterMotor.setInverted(true);

        m_octoQuad.resetAllPositions();

        m_leftFeederServo.setContinousRotationMode(true);
        m_rightFeederServo.setContinousRotationMode(true);

        m_leftFeederServo.setReversed(true);

        m_leftFeederServo.setEnabled(true);
        m_rightFeederServo.setEnabled(true);

        m_shooterFeedback.setTolerance(3);
    }

    @Override
    public void periodic() {
        if (!m_octoQuad.readAllDataWithoutLocalizer(m_encoderData)) {
            System.out.println("Error reading octoquad data");
        }
    }

    public double getShooterVelocity() {
        return m_encoderData.velocities[ShooterConstants.kEncoderPort] * ShooterConstants.kEncoderDistancePerPulse * ShooterConstants.kEncoderSampleRate * -1;
    }

    public double getShooterPosition() {
        return m_encoderData.positions[ShooterConstants.kEncoderPort] * ShooterConstants.kEncoderDistancePerPulse * -1;
    }

    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(m_shooterMotor::setVoltage, log -> {
                log.motor("shooter-wheel")
                        .voltage(Volts.of(m_shooterMotor.get() * RobotController.getBatteryVoltage()))
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
        m_shooterMotor.setVoltage(m_shooterFeedback.calculate(getShooterVelocity(), speed)
                    + m_shooterFeedforward.calculate(speed));
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
