// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.wpilib.units.Units.Amps;

import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.epilogue.Epilogue;
import org.wpilib.epilogue.Logged;
import org.wpilib.epilogue.NotLogged;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.hardware.discrete.AnalogInput;
import org.wpilib.system.DataLogManager;
import org.wpilib.units.measure.Current;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystemNew;
import frc.robot.subsystems.Shooter;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
@Logged
public class Robot extends OpModeRobot {
  // private Command m_autonomousCommand;

  // private final RobotContainer m_robotContainer;

  // The robot's subsystems
  @NotLogged
  private final DriveSubsystemNew m_robotDrive = new DriveSubsystemNew();

  @NotLogged
  private final Shooter m_shooter = new Shooter();

  @Logged
  public DriveSubsystemNew getDrive() {
    return m_robotDrive;
  }

  @Logged
  public Shooter getShooter() {
    return m_shooter;
  }

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

  @NotLogged
  public CommandGamepad getDriverGamepad() {
    return m_driverController;
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    DataLogManager.start("/home/systemcore/logs");
    DataLogManager.start();
  }

  public void robotPeriodic() {
    Scheduler.getDefault().run();
    long start = System.nanoTime();
    var config = Epilogue.getConfig();
    Epilogue.robotLogger.tryUpdate(config.backend.getNested(config.root), this, config.errorHandler);
    config.backend.log("Epilogue/Stats/Last Run", (System.nanoTime() - start) / 1e6);
  }

  @Override
  public void nonePeriodic() {
    robotPeriodic();
  }
}
