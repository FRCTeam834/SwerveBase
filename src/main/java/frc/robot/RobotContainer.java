/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// User files
import frc.robot.DriverProfiles.ProfilingManagement;
import frc.robot.commands.LetsRoll1Joystick;
import frc.robot.commands.LetsRoll2Joysticks;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.UltrasonicSensor;
import frc.robot.swerve.DriveTrain;
import frc.robot.Parameters;
import frc.robot.robotMode.ROBOT_STATE;

// WPI Libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Subsystems
  private final ProfilingManagement profilingManagement = new ProfilingManagement();
  private final NavX navX = new NavX();
  private final DriveTrain driveTrain = new DriveTrain();
  private final UltrasonicSensor ultrasonicSensor = new UltrasonicSensor();

  // Commands
  private final LetsRoll2Joysticks letsRoll2Joysticks = new LetsRoll2Joysticks();
  private final LetsRoll1Joystick letsRoll1Joystick = new LetsRoll1Joystick();

  // Define the joysticks (need to be public so commands can access axes)
  public static Joystick leftJoystick;
  public static Joystick rightJoystick;
  
  // Left Joystick button array
  public static JoystickButton leftJoystickButtons[];

  // Right Joystick button array
  public static JoystickButton rightJoystickButtons[];

  // The robot's state
  public static ROBOT_STATE robotState;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Try to assign the left joystick
    try {
      leftJoystick = new Joystick(0);

      // If we get here, then the left joystick was successful and we can try the right joystick
      try {
        rightJoystick = new Joystick(1);

        // Both joysticks are present
        robotState = ROBOT_STATE.TWO_JOYSTICKS;

      }
      catch (Exception e) {

        // We only have one joystick, the left
        robotState = ROBOT_STATE.ONE_JOYSTICK;
      }

    }
    catch (Exception e) {

      // No joysticks detected
      robotState = ROBOT_STATE.NO_JOYSTICKS;
    }


    // Setup the robot based on the state of it
    if (robotState == ROBOT_STATE.TWO_JOYSTICKS) {
      // Full setup

      // Left Joystick button assignment (buttons array starts at 0)
      for(int buttonIndex = 1; buttonIndex <= Parameters.joysticks.JOYSTICK_BUTTON_COUNT; buttonIndex++) {
        leftJoystickButtons[buttonIndex - 1] = new JoystickButton(leftJoystick, buttonIndex);
      }

      // Right Joystick button assignment (buttons array starts at 0)
      for(int buttonIndex = 1; buttonIndex <= Parameters.joysticks.JOYSTICK_BUTTON_COUNT; buttonIndex++) {
        rightJoystickButtons[buttonIndex - 1] = new JoystickButton(rightJoystick, buttonIndex);
      }

      // Command setup
      // Configure the command (on the second button of the joystick)
      leftJoystickButtons[1].whenPressed(letsRoll2Joysticks);
    }
    else if (robotState == ROBOT_STATE.ONE_JOYSTICK) {
      // Left Joystick button assignment (buttons array starts at 0)
      for(int buttonIndex = 1; buttonIndex <= Parameters.joysticks.JOYSTICK_BUTTON_COUNT; buttonIndex++) {
        leftJoystickButtons[buttonIndex - 1] = new JoystickButton(leftJoystick, buttonIndex);
      }

      // Command setup
      // Configure the command (on the second button of the joystick)
      leftJoystickButtons[1].whenPressed(letsRoll1Joystick);
    }
    else {
      // No joysticks (show mode)

      // Command setup

    }
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
