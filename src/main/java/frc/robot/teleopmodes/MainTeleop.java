package frc.robot.teleopmodes;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

import frc.robot.Robot;

@Teleop
public class MainTeleop extends PeriodicOpMode {
    private final Robot m_robot;

    private final Command m_joystickDriveCommand;

    public MainTeleop(Robot robot) {
        m_robot = robot;
        var driverGamepad = robot.getDriverGamepad();
        m_joystickDriveCommand = m_robot.getDrive().getJoystickDriveCommand(driverGamepad.getHID());
    }

    @Override
    public void disabledPeriodic() {
        m_robot.robotPeriodic();
    }

    @Override
    public void start() {
        Scheduler.getDefault().schedule(m_joystickDriveCommand);

        var shooter = m_robot.getShooter();
        var driverGamepad = m_robot.getDriverGamepad();

        driverGamepad.rightBumper().and(driverGamepad.leftBumper()).whileTrue(shooter.getSpinAndFeedCommand());
        driverGamepad.rightBumper().and(driverGamepad.leftBumper().negate()).whileTrue(shooter.getSpinCommand());
    }

    @Override
    public void periodic() {
        m_robot.robotPeriodic();
    }

    @Override
    public void end() {
        Scheduler.getDefault().cancel(m_joystickDriveCommand);
    }

}
