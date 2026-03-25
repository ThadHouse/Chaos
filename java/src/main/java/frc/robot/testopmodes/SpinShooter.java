package frc.robot.testopmodes;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.TestOpMode;

import frc.robot.Robot;

@TestOpMode
public class SpinShooter extends PeriodicOpMode {
    private final Robot m_robot;

    private final Command spinCommand;

    public SpinShooter(Robot robot) {
        m_robot = robot;
        spinCommand = m_robot.getShooter().getSpinCommand();
    }

    @Override
    public void disabledPeriodic() {
        m_robot.robotPeriodic();
    }

    @Override
    public void start() {
        Scheduler.getDefault().schedule(spinCommand);
    }

    @Override
    public void periodic() {
        m_robot.robotPeriodic();
    }

    @Override
    public void end() {
        Scheduler.getDefault().cancel(spinCommand);
    }

}
