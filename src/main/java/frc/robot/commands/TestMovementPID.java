// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class TestMovementPID extends CommandBase {
  /** Creates a new TestPID. */
  public TestMovementPID() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Wait for half a second
    RobotContainer.timer.delay(.5);

    // Move each of the wheels to 0
    Robot.driveTrain.resetOdometry(new Pose2d(0, 0, new Rotation2d()));
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // The top button
    if (RobotContainer.leftJoystick.getRawButton(3)) {
      Robot.driveTrain.trajectoryFollow(new Pose2d(Robot.driveTrain.getXPosition() + 1, Robot.driveTrain.getYPosition(), new Rotation2d()), 1);
    }

    // The bottom button
    else if (RobotContainer.leftJoystick.getRawButton(2)) {
      Robot.driveTrain.trajectoryFollow(new Pose2d(Robot.driveTrain.getXPosition() - 1, Robot.driveTrain.getYPosition(), new Rotation2d()), 1);
    }

    // The left button
    else if (RobotContainer.leftJoystick.getRawButton(4)) {
      Robot.driveTrain.trajectoryFollow(new Pose2d(Robot.driveTrain.getXPosition(), Robot.driveTrain.getYPosition() - 1, new Rotation2d()), 1);
    }

    // The right button
    else if (RobotContainer.leftJoystick.getRawButton(5)) {
      Robot.driveTrain.trajectoryFollow(new Pose2d(Robot.driveTrain.getXPosition(), Robot.driveTrain.getYPosition() + 1 , new Rotation2d()), 1);
    }

    // The left throttle button (moves in clockwise direction)
    else if (RobotContainer.leftJoystick.getRawButton(8)) {
      Robot.driveTrain.trajectoryFollow(new Pose2d(Robot.driveTrain.getXPosition(), Robot.driveTrain.getYPosition(), Rotation2d.fromDegrees(Robot.driveTrain.getThetaPosition().getDegrees() - 90)), 1);
    }

    // The right throttle button (moves in the counterclockwise direction)
    else if (RobotContainer.leftJoystick.getRawButton(9)) {
      Robot.driveTrain.trajectoryFollow(new Pose2d(Robot.driveTrain.getXPosition(), Robot.driveTrain.getYPosition(), Rotation2d.fromDegrees(Robot.driveTrain.getThetaPosition().getDegrees() + 90)), 1);
    }

    // Publishes velocity and angle to network tables
    Robot.driveTrain.publishPerformanceData();
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.rightJoystick.getRawButton(1);
  }
}
