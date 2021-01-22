/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.DriverProfiles.ProfilingManagement;
import frc.robot.commands.LetsRoll;
import frc.robot.subsystems.NavX;
import frc.robot.swerve.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Subsystems
  //private final ProfilingManagement profilingManagement = new ProfilingManagement();
  //private final NavX navX = new NavX();
  //private final DriveTrain driveTrain = new DriveTrain();

  // Commands
  private final LetsRoll letsRoll = new LetsRoll();


  private final Joystick leftJoystick = new Joystick(0);
  private final Joystick rightJoystick = new Joystick(1);

  
  @SuppressWarnings("unused")
  private final JoystickButton
  // Left Joystick
  lJoystick1 = new JoystickButton(leftJoystick, 1), lJoystick2 = new JoystickButton(leftJoystick, 2),
  lJoystick3 = new JoystickButton(leftJoystick, 3), lJoystick4 = new JoystickButton(leftJoystick, 4),
  lJoystick5 = new JoystickButton(leftJoystick, 5), lJoystick6 = new JoystickButton(leftJoystick, 6),
  lJoystick7 = new JoystickButton(leftJoystick, 7), lJoystick8 = new JoystickButton(leftJoystick, 8),
  lJoystick9 = new JoystickButton(leftJoystick, 9), lJoystick10 = new JoystickButton(leftJoystick, 10),
  lJoystick11 = new JoystickButton(leftJoystick, 11),

  // Right Joystick
  rJoystick1 = new JoystickButton(rightJoystick, 1), rJoystick2 = new JoystickButton(rightJoystick, 2),
  rJoystick3 = new JoystickButton(rightJoystick, 3), rJoystick4 = new JoystickButton(rightJoystick, 4),
  rJoystick5 = new JoystickButton(rightJoystick, 5), rJoystick6 = new JoystickButton(rightJoystick, 6),
  rJoystick7 = new JoystickButton(rightJoystick, 7), rJoystick8 = new JoystickButton(rightJoystick, 8),
  rJoystick9 = new JoystickButton(rightJoystick, 9), rJoystick10 = new JoystickButton(rightJoystick, 10),
  rJoystick11 = new JoystickButton(rightJoystick, 11);


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
    rJoystick2.whenPressed(letsRoll);
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
