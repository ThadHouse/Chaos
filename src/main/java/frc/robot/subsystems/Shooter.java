package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.utils.OctoQuadV3;
import frc.utils.OctoQuadV3.EncoderData;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.SparkMini;
import frc.robot.Constants.ShooterConstants;

@Logged
public class Shooter extends SubsystemBase {
    @NotLogged
    OctoQuadV3 m_octoQuad = new OctoQuadV3(ShooterConstants.kI2cPort);

    @NotLogged
    SparkMini m_shooterMotor = new SparkMini(ShooterConstants.kShooterMotorPort);

    private EncoderData m_encoderData = new EncoderData();

    /** Creates a new Shooter. */
    public Shooter() {
        m_shooterMotor.setInverted(true);

        m_octoQuad.resetAllPositions();
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

    private final PIDController m_shooterFeedback = new PIDController(ShooterConstants.kP, ShooterConstants.kI,
            ShooterConstants.kD);

    private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(ShooterConstants.kS,
            ShooterConstants.kV, ShooterConstants.kA);

    public Command runShooter(DoubleSupplier shooterSpeed) {
        return run(() -> {
            m_shooterMotor.setVoltage(m_shooterFeedback.calculate(getShooterVelocity(), shooterSpeed.getAsDouble())
                    + m_shooterFeedforward.calculate(shooterSpeed.getAsDouble()));

        })
        .finallyDo(() -> {
            m_shooterMotor.stopMotor();
        })
        .withName("runShooter");
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}
