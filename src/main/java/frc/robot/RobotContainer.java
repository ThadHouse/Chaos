// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystemNew;
import frc.robot.subsystems.Shooter;
import frc.utils.ExpansionHub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static edu.wpi.first.units.Units.Amps;

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
    private final ExpansionHub m_expansionHub = new ExpansionHub(0);

    @NotLogged
    private final AnalogInput m_currentReading = new AnalogInput(5);

    @Logged
    public boolean isExpansionHubConnected() {
        return m_expansionHub.isHubConnected();
    }

    @Logged
    public Current getRobotCurrent() {
        double voltage = m_currentReading.getVoltage();
        double current = (voltage / 3.3) * 50;
        return Amps.of(current);
    }

    @NotLogged
    // The driver's controller
    private final CommandXboxController m_driverController = new CommandXboxController(
            OIConstants.kDriverControllerPort);

    // @NotLogged
    // private final CommandGamepad m_driveGp = new
    // CommandGamepad(OIConstants.kDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        m_robotDrive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                                true),
                        m_robotDrive));

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
        m_shooter.setDefaultCommand(new RunCommand(() -> {
            m_shooter.setSpeed(m_driverController.getHID().getRightBumperButton() ? 40 : 0);
            m_shooter.setFeed(m_driverController.getHID().getLeftBumperButton() ? true : false);
        }, m_shooter));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
