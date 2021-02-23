// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// WPI libraries
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;

import frc.robot.swerve.DriveTrain;

public class MoveToPosition extends CommandBase {
  /** Moves the robot to the desired position */

  DriveTrain driveTrain;
  Pose2d desiredPosition;
  double linearVel;

  public MoveToPosition(Pose2d desiredPose, double linearVelocity) {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
    driveTrain = Robot.driveTrain;
    desiredPosition = desiredPose;
    linearVel = linearVelocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Set the drivetrain to run to the position
    // literally the entire command lol
    driveTrain.trajectoryFollow(desiredPosition, linearVel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // Check if the trajectory is complete
    return driveTrain.atTrajectoryReference();
  }
}
