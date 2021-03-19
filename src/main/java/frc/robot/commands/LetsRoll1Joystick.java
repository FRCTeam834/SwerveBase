/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.lang.Math;

// Robot libraries
import frc.robot.Parameters;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.swerve.DriveTrain;

// WPI libraries
import edu.wpi.first.wpilibj2.command.CommandBase;


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

    // Get all of the current joystick inputs
    double leftX = 0; // constrainJoystick(RobotContainer.leftJoystick.getX());
    double leftY = 0; // constrainJoystick(RobotContainer.leftJoystick.getY());

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
      Robot.driveTrain.drive((leftX * Parameters.driver.CURRENT_PROFILE.MAX_SPEED), (leftY * Parameters.driver.CURRENT_PROFILE.MAX_SPEED),
                        Math.toRadians(turning * Parameters.driver.CURRENT_PROFILE.MAX_STEER_SPEED), Parameters.driver.CURRENT_PROFILE.FIELD_CENTRIC);
    }
    else if (Parameters.driver.CURRENT_PROFILE.LOCKEM_UP) {
      Robot.driveTrain.lockemUp();
    }
    else {
      Robot.driveTrain.haltAllModules();
    }

    // Update driver profile if available
    Robot.profilingManagement.checkForUpdate();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.haltAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Return a constrained Joystick value
  private double constrainJoystick(double rawValue) {

    // If the value is out of tolerance, then zero it. Otherwise return the value of the joystick
    if (Math.abs(rawValue) < Parameters.driver.CURRENT_PROFILE.JOYSTICK_DEADZONE) {
      return 0;
    }
    else {
      return rawValue;
    }
  }
}