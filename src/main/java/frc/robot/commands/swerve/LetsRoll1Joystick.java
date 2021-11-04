/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * @author Christian Piper (@CAP1Sup)
 * @since 5/8/20
 */

package frc.robot.commands.swerve;

import java.lang.Math;

// Robot libraries
import frc.robot.Parameters;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.enums.ControlInputs;

// WPI libraries
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.GenericHID.Hand;


public class LetsRoll1Joystick extends CommandBase {

  public LetsRoll1Joystick() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Declare variables for the X and Y
    double leftX, leftY;

    // Get all of the current joystick inputs
    if (Parameters.driver.currentProfile.inputType == ControlInputs.JOYSTICKS) {
      leftX = RobotContainer.constrainJoystick(RobotContainer.leftJoystick.getX());
      leftY = RobotContainer.constrainJoystick(RobotContainer.leftJoystick.getY());
    }
    else {
      leftX = RobotContainer.constrainJoystick(RobotContainer.xbox.getX(Hand.kRight));
      leftY = RobotContainer.constrainJoystick(RobotContainer.xbox.getY(Hand.kRight));
    }

    // Declare here so the value is in scope
    double turning = 0;

    // Check the values of the buttons
    boolean counterClockwiseButton = false; //RobotContainer.leftJoystick.getRawButton(6);
    boolean clockwiseButton = false; //RobotContainer.leftJoystick.getRawButton(7);

    // Determine the correct amount of turning based on the button inputs
    if (!(clockwiseButton && counterClockwiseButton)) {
      if (counterClockwiseButton) {
        // Positive direction is pressed
        turning = 1;
      }
      else if (clockwiseButton) {
        // Negative direction is pressed
        turning = -1;
      }
    }

    // If any of the sticks are out of range, then we need to move. Otherwise, lock up the drivetrain (if specified)
    if (leftX != 0 || leftY != 0 || turning != 0) {

      // Move the drivetrain with the desired values
      Robot.driveTrain.drive((leftX * Parameters.driver.currentProfile.maxModVelocity), (leftY * Parameters.driver.currentProfile.maxModVelocity),
                        Math.toRadians(turning * Parameters.driver.currentProfile.maxSteerRate), Parameters.driver.currentProfile.fieldCentric);
    }
    else if (Parameters.driver.currentProfile.lockemUp) {
      Robot.driveTrain.lockemUp();
    }
    else {
      Robot.driveTrain.stopModules();
    }

    // Update driver profile if available
    Robot.profilingManagement.checkForUpdate();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}