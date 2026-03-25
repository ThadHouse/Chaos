package frc.robot.subsystems;

import org.wpilib.epilogue.Logged;
import org.wpilib.epilogue.Epilogue;
import org.wpilib.epilogue.logging.ClassSpecificLogger;
import org.wpilib.epilogue.logging.EpilogueBackend;

public class ShooterLogger extends ClassSpecificLogger<Shooter> {
  public ShooterLogger() {
    super(Shooter.class);
  }

  @Override
  public void update(EpilogueBackend backend, Shooter object) {
    if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
      backend.log("getShooterVelocity", object.getShooterVelocity());
      backend.log("getShooterPosition", object.getShooterPosition());
      backend.log("getShooterCurrent", object.getShooterCurrent());
      backend.log("isHubConnected", object.isHubConnected());
    }
  }
}
