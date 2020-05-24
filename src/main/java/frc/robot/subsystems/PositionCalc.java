/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.automove;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import Parameters and internal libraries
import frc.robot.Parameters;

// Robot
import frc.robot.Robot;

public class PositionCalc extends SubsystemBase {

  public PositionCalc() {

  }

  // Gets the distance to an object
  public double calulateDistanceToObject(double actualHeight, double relativeHeight) {
    double distance = (actualHeight * Parameters.CAMERA_FOCAL_LENGTH) / relativeHeight;
    return distance;
  }

  // Calculates the field position relative to the goal
  public Pose2d calculateFieldPosition(double goalHeight) {
    double robotAngle = 90 - Robot.navX.getFusedHeading();
    double distanceToGoal = calulateDistanceToObject(Parameters.GOAL_HEIGHT, goalHeight);

    double x = distanceToGoal * Math.cos(robotAngle) * ((robotAngle >= 0) ? 1 : -1); // If robot is turned to the left looking at the goal, then the robot must be to the right of the goal
    double y = distanceToGoal * Math.sin(robotAngle);

    Pose2d currentPosition = new Pose2d(new Translation2d(x, y), Robot.navX.getFusedRotation2d());

    return currentPosition;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
