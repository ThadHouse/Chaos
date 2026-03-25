package frc.robot;

import org.wpilib.epilogue.Logged;
import org.wpilib.epilogue.Epilogue;
import org.wpilib.epilogue.logging.ClassSpecificLogger;
import org.wpilib.epilogue.logging.EpilogueBackend;

public class RobotLogger extends ClassSpecificLogger<Robot> {
  public RobotLogger() {
    super(Robot.class);
  }

  @Override
  public void update(EpilogueBackend backend, Robot object) {
    if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
      Epilogue.driveSubsystemNewLogger.tryUpdate(backend.getNested("getDrive"), object.getDrive(), Epilogue.getConfig().errorHandler);
      Epilogue.shooterLogger.tryUpdate(backend.getNested("getShooter"), object.getShooter(), Epilogue.getConfig().errorHandler);
      backend.log("getRobotCurrent", object.getRobotCurrent());
    }
  }
}
