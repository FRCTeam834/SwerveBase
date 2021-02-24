/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

// Robot libraries
import frc.robot.Parameters;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.swerve.DriveTrain;

// WPI libraries
import edu.wpi.first.wpilibj2.command.CommandBase;


public class LetsRoll extends CommandBase {

  DriveTrain driveTrain;

  public LetsRoll() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
    driveTrain = Robot.driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Get all of the current joystick inputs
    double leftX =  constrainJoystick(RobotContainer.leftJoystick.getX());
    double leftY =  constrainJoystick(RobotContainer.leftJoystick.getY());
    double rightX = constrainJoystick(RobotContainer.rightJoystick.getX());
    
    // If any of the sticks are out of range, then we need to move. Otherwise, lock up the drivetrain (if specified)
    if (leftX != 0 || leftY != 0 || rightX != 0) {

      // Move the drivetrain with the desired values
      driveTrain.drive((leftX * Parameters.driver.CURRENT_DRIVER_PROFILE.MAX_SPEED), (leftY * Parameters.driver.CURRENT_DRIVER_PROFILE.MAX_SPEED),
                        Math.toRadians(rightX * Parameters.driver.CURRENT_DRIVER_PROFILE.MAX_TURN_SPEED), Parameters.driver.CURRENT_DRIVER_PROFILE.FIELD_CENTRIC);
    }
    else if (Parameters.driver.CURRENT_DRIVER_PROFILE.LOCKEM_UP) {
      driveTrain.lockemUp();
    }

    // Update driver profile if available
    Robot.profilingManagement.checkForUpdate();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.lockemUp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Return a constrained Joystick value
  private double constrainJoystick(double rawValue) {

    // If the value is out of tolerance, then zero it. Otherwise 
    if (rawValue < Parameters.driver.CURRENT_DRIVER_PROFILE.JOYSTICK_DEADZONE) {
      return 0;
    }
    else {
      return rawValue;
    }
  }
}
