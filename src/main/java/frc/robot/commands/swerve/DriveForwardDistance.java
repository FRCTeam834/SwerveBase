// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

// Parameters
import frc.robot.Parameters;

// Robot
import frc.robot.Robot;

// WPI libraries
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveForwardDistance extends CommandBase {
  /** Moves the robot to the desired position */

  // Main carrier variables 
  Pose2d desiredPose2d;
  double linearVel;

  // Move forward at the set linear velocity
  public DriveForwardDistance(double distance, double linearVelocity) {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);

    // Get the current position
    Pose2d currentPosition = Robot.driveTrain.getPose2D();
    double newX = currentPosition.getX() + (distance * currentPosition.getRotation().getCos());
    double newY = currentPosition.getY() + (distance * currentPosition.getRotation().getSin());
    this.desiredPose2d = new Pose2d(newX, newY, currentPosition.getRotation());
    this.linearVel = linearVelocity;
  }


  // Default to current driver profile for the linear speed
  public DriveForwardDistance(double distance) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);

    // Get the current position
    Pose2d currentPosition = Robot.driveTrain.getPose2D();
    double newX = currentPosition.getX() + (distance * currentPosition.getRotation().getCos());
    double newY = currentPosition.getY() + (distance * currentPosition.getRotation().getSin());
    this.desiredPose2d = new Pose2d(newX, newY, currentPosition.getRotation());
    this.linearVel = Parameters.driver.CURRENT_PROFILE.MAX_SPEED;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Set the drivetrain to run to the position
    Robot.driveTrain.trajectoryFollow(desiredPose2d, linearVel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // Check if the trajectory is complete
    return Robot.driveTrain.atTrajectoryReference();
  }
}