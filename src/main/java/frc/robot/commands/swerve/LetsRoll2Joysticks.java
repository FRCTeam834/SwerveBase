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

// Imports
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Parameters;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.enums.ControlInputs;

public class LetsRoll2Joysticks extends CommandBase {

  // Stores robot driving type (between FOD and ROD)
  boolean fieldCentric = Parameters.driver.currentProfile.fieldCentric;

  public LetsRoll2Joysticks() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Clear the trigger pressed flag
    RobotContainer.leftJoystick.getTriggerPressed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Get if the left trigger has been pressed, inverting the driving mode
    if (RobotContainer.leftJoystick.getTriggerPressed()) {
      fieldCentric = !fieldCentric;
    }

    // Create variables for storing movement values
    double leftX, rightX, rightY;

    // Get all of the current joystick inputs
    if (Parameters.driver.currentProfile.inputType == ControlInputs.JOYSTICKS) {
      leftX = RobotContainer.constrainJoystick(RobotContainer.leftJoystick.getX());
      rightX = RobotContainer.constrainJoystick(RobotContainer.rightJoystick.getX());
      rightY = RobotContainer.constrainJoystick(RobotContainer.rightJoystick.getY());
    } else {
      leftX = RobotContainer.constrainJoystick(RobotContainer.xbox.getX(Hand.kLeft));
      rightX = RobotContainer.constrainJoystick(RobotContainer.xbox.getX(Hand.kRight));
      rightY = RobotContainer.constrainJoystick(RobotContainer.xbox.getY(Hand.kRight));
    }

    // If any of the sticks are out of range, then we need to move. Otherwise, lock up the
    // drivetrain (if specified) or just halt the modules
    if (leftX != 0 || rightX != 0 || rightY != 0) {

      // Move the drivetrain with the desired values (left right values are flipped from the logical
      // way, thanks WPI)
      Robot.driveTrain.drive(
          (rightY * Parameters.driver.currentProfile.maxModVelocity),
          (rightX * Parameters.driver.currentProfile.maxModVelocity),
          Math.toRadians(leftX * Parameters.driver.currentProfile.maxSteerRate),
          fieldCentric);
    } else if (Parameters.driver.currentProfile.lockemUp) {
      Robot.driveTrain.lockemUp();
    } else {
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
