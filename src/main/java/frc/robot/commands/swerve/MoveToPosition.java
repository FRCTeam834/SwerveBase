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

public class MoveToPosition extends CommandBase {
  /** Moves the robot to the desired position */

  // Main defines
  Pose2d desiredPosition;
  double linearVel;

  public MoveToPosition(Pose2d desiredPose, double linearVelocity) {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
    this.desiredPosition = desiredPose;
    this.linearVel = linearVelocity;
  }

  // Default to 2 m/s for the linear speed
  public MoveToPosition(Pose2d desiredPose) {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
    this.desiredPosition = desiredPose;
    this.linearVel = Parameters.driver.CURRENT_PROFILE.MAX_SPEED;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Set the drivetrain to run to the position
    Robot.driveTrain.trajectoryFollow(desiredPosition, linearVel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // Check if the trajectory is complete
    return Robot.driveTrain.finishedMovement();
  }
}
