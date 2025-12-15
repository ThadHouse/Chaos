// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystemNew;
import frc.robot.subsystems.Shooter;

import static org.wpilib.units.Units.Amps;

import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.epilogue.Logged;
import org.wpilib.epilogue.NotLogged;
import org.wpilib.hardware.discrete.AnalogInput;
import org.wpilib.math.util.MathUtil;
import org.wpilib.units.measure.Current;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
@Logged
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystemNew m_robotDrive = new DriveSubsystemNew();

    private final Shooter m_shooter = new Shooter();

    @NotLogged
    private final AnalogInput m_currentReading = new AnalogInput(5);

    @Logged
    public Current getRobotCurrent() {
        double voltage = m_currentReading.getVoltage();
        double current = (voltage / 3.3) * 50;
        return Amps.of(current);
    }

    @NotLogged
    // The driver's controller
    private final CommandGamepad m_driverController = new CommandGamepad(
            OIConstants.kDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        m_robotDrive.setDefaultCommand(m_robotDrive.runRepeatedly(() -> {
            m_robotDrive.driveJoysticks(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(),
                            OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(),
                            OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(),
                            OIConstants.kDriveDeadband),
                    true);
        }).withPriority(Command.LOWEST_PRIORITY).named("Drive Default"));

        // m_driverController
        // .a()
        // .and(m_driverController.start())
        // .whileTrue(m_robotDrive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // m_driverController
        // .b()
        // .and(m_driverController.start())
        // .whileTrue(m_robotDrive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // m_driverController
        // .x()
        // .and(m_driverController.start())
        // .whileTrue(m_robotDrive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // m_driverController
        // .y()
        // .and(m_driverController.start())
        // .whileTrue(m_robotDrive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        m_shooter.setDefaultCommand(m_shooter.runRepeatedly(() -> {
            m_shooter.setSpeed(m_driverController.getHID().getRightShoulderButton() ? 40 : 0);
            m_shooter.setFeed(m_driverController.getHID().getLeftShoulderButton() && m_driverController.getHID().getRightShoulderButton() ? true : false);
        }).withPriority(Command.LOWEST_PRIORITY).named("Default Shooter"));

        // m_driverController
        // .a()
        // .and(m_driverController.back())
        // .whileTrue(m_shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // m_driverController
        // .b()
        // .and(m_driverController.back())
        // .whileTrue(m_shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // m_driverController
        // .x()
        // .and(m_driverController.back())
        // .whileTrue(m_shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // m_driverController
        // .y()
        // .and(m_driverController.back())
        // .whileTrue(m_shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Command.noRequirements().executing((cr) -> {
            cr.park();
        }).named("Empty Autonomous Command");
    }
}
