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
import edu.wpi.first.wpilibj.GenericHID.Hand;

// WPI libraries
import edu.wpi.first.wpilibj2.command.CommandBase;


public class LetsRoll2Joysticks extends CommandBase {

  public LetsRoll2Joysticks() {
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
    double leftX =  RobotContainer.constrainJoystick(RobotContainer.leftJoystick.getX());
    double rightX = RobotContainer.constrainJoystick(RobotContainer.rightJoystick.getX());
    double rightY = RobotContainer.constrainJoystick(RobotContainer.rightJoystick.getY());
    //double leftX =  constrainJoystick(RobotContainer.xbox.getX(Hand.kLeft));
    //double rightX = constrainJoystick(RobotContainer.xbox.getX(Hand.kRight));
    //double rightY = constrainJoystick(RobotContainer.xbox.getY(Hand.kRight));

    // If any of the sticks are out of range, then we need to move. Otherwise, lock up the drivetrain (if specified) or just halt the modules
    if (leftX != 0 || rightX != 0 || rightY != 0) {

      // Move the drivetrain with the desired values (left right values are flipped from the logical way, thanks WPI)
      Robot.driveTrain.drive((rightY * Parameters.driver.CURRENT_PROFILE.MAX_SPEED), (rightX * Parameters.driver.CURRENT_PROFILE.MAX_SPEED),
                        Math.toRadians(leftX * Parameters.driver.CURRENT_PROFILE.MAX_STEER_SPEED), Parameters.driver.CURRENT_PROFILE.FIELD_CENTRIC);
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
}