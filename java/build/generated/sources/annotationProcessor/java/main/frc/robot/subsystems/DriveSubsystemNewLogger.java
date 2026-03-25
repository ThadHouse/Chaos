package frc.robot.subsystems;

import org.wpilib.epilogue.Logged;
import org.wpilib.epilogue.Epilogue;
import org.wpilib.epilogue.logging.ClassSpecificLogger;
import org.wpilib.epilogue.logging.EpilogueBackend;

public class DriveSubsystemNewLogger extends ClassSpecificLogger<DriveSubsystemNew> {
  public DriveSubsystemNewLogger() {
    super(DriveSubsystemNew.class);
  }

  @Override
  public void update(EpilogueBackend backend, DriveSubsystemNew object) {
    if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
      backend.log("getFrontLeftCurrent", object.getFrontLeftCurrent());
      backend.log("getFrontRightCurrent", object.getFrontRightCurrent());
      backend.log("getRearLeftCurrent", object.getRearLeftCurrent());
      backend.log("getRearRightCurrent", object.getRearRightCurrent());
      backend.log("isHubConnected", object.isHubConnected());
      backend.log("getPose", object.getPose(), org.wpilib.math.geometry.Pose2d.struct);
      backend.log("getCurrentWheelSpeeds", object.getCurrentWheelSpeeds(), org.wpilib.math.kinematics.MecanumDriveWheelVelocities.struct);
      backend.log("getCurrentWheelDistances", object.getCurrentWheelDistances(), org.wpilib.math.kinematics.MecanumDriveWheelPositions.struct);
      backend.log("getTurnRate", object.getTurnRate());
      backend.log("getHeading", object.getHeading(), org.wpilib.math.geometry.Rotation2d.struct);
    }
  }
}
