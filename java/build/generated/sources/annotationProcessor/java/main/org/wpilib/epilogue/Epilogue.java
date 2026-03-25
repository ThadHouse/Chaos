package org.wpilib.epilogue;

import static org.wpilib.units.Units.Seconds;

import org.wpilib.hardware.hal.HAL;

import frc.robot.RobotLogger;
import frc.robot.subsystems.DriveSubsystemNewLogger;
import frc.robot.subsystems.ShooterLogger;

public final class Epilogue {
  static {
    HAL.reportUsage("Epilogue", "");
  }

  private static final EpilogueConfiguration config = new EpilogueConfiguration();

  public static final DriveSubsystemNewLogger driveSubsystemNewLogger = new DriveSubsystemNewLogger();
  public static final RobotLogger robotLogger = new RobotLogger();
  public static final ShooterLogger shooterLogger = new ShooterLogger();

  public static void configure(java.util.function.Consumer<EpilogueConfiguration> configurator) {
    configurator.accept(config);
  }

  public static EpilogueConfiguration getConfig() {
    return config;
  }

  /**
   * Checks if data associated with a given importance level should be logged.
   */
  public static boolean shouldLog(Logged.Importance importance) {
    return importance.compareTo(config.minimumImportance) >= 0;
  }
}
