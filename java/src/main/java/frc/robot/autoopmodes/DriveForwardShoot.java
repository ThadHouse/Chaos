package frc.robot.autoopmodes;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;

import frc.robot.Robot;

@Autonomous
public class DriveForwardShoot extends PeriodicOpMode {
    private final Robot m_robot;

    private final Command m_command;

    public DriveForwardShoot(Robot robot) {
        m_robot = robot;

        m_command = Command.sequence(
            m_robot.getDrive().driveForwardTime(2),
            m_robot.getShooter().shootTime(5)
        ).named("DriveForwardShoot");
    }

    @Override
    public void disabledPeriodic() {
        m_robot.robotPeriodic();
    }

    @Override
    public void start() {
        Scheduler.getDefault().schedule(m_command);
    }

    @Override
    public void periodic() {
        m_robot.robotPeriodic();
    }

    @Override
    public void end() {
        Scheduler.getDefault().cancel(m_command);
    }
}
