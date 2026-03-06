package frc.robot.autoopmodes;

import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;

import frc.robot.Robot;

@Autonomous
public class DoNothingAuto extends PeriodicOpMode {
    private final Robot m_robot;

    public DoNothingAuto(Robot robot) {
        m_robot = robot;
    }

    @Override
    public void disabledPeriodic() {
        m_robot.robotPeriodic();
    }

    @Override
    public void periodic() {
        m_robot.robotPeriodic();
    }

}
