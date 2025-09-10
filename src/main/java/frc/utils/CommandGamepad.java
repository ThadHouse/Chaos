package frc.utils;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandGamepad extends CommandGenericHID {



  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public CommandGamepad(int port) {
    super(port);
    //m_hid = new Gamepad(port);
  }


    /**
   * Constructs a Trigger instance around the A button's digital signal.
   *
   * @return a Trigger instance representing the A button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #a(EventLoop)
   */
  public Trigger a() {
    return a(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the A button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the A button's digital signal attached
   *     to the given loop.
   */
  public Trigger a(EventLoop loop) {
    return button(Gamepad.Button.kSouth.value, loop);
  }

  /**
   * Constructs a Trigger instance around the B button's digital signal.
   *
   * @return a Trigger instance representing the B button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #b(EventLoop)
   */
  public Trigger b() {
    return b(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the B button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the B button's digital signal attached
   *     to the given loop.
   */
  public Trigger b(EventLoop loop) {
    return button(Gamepad.Button.kEast.value, loop);
  }

  /**
   * Constructs a Trigger instance around the X button's digital signal.
   *
   * @return a Trigger instance representing the X button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #x(EventLoop)
   */
  public Trigger x() {
    return x(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the X button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the X button's digital signal attached
   *     to the given loop.
   */
  public Trigger x(EventLoop loop) {
    return button(Gamepad.Button.kWest.value, loop);
  }

  /**
   * Constructs a Trigger instance around the Y button's digital signal.
   *
   * @return a Trigger instance representing the Y button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #y(EventLoop)
   */
  public Trigger y() {
    return y(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the Y button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the Y button's digital signal attached
   *     to the given loop.
   */
  public Trigger y(EventLoop loop) {
    return button(Gamepad.Button.kNorth.value, loop);
  }

  /**
   * Constructs a Trigger instance around the left bumper button's digital signal.
   *
   * @return a Trigger instance representing the left bumper button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #leftBumper(EventLoop)
   */
  public Trigger leftBumper() {
    return leftBumper(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the left bumper button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the left bumper button's digital signal attached
   *     to the given loop.
   */
  public Trigger leftBumper(EventLoop loop) {
    return button(Gamepad.Button.kLeftShoulder.value, loop);
  }
}
