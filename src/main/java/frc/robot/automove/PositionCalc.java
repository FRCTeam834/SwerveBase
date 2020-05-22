/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.automove;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import constants and internal libraries
import frc.robot.Constants;
import frc.robot.automove.FieldCoordinates;

// Robot
import frc.robot.Robot;

public class PositionCalc extends SubsystemBase {

  public PositionCalc() {

  }

  public double calulateDistanceToObject(double actualHeight, double relativeHeight) {
    double distance = (actualHeight * Constants.CAMERA_FOCAL_LENGTH) / relativeHeight;
    return distance;
  }

  public FieldCoordinates calculateFieldPosition(double goalHeight) {
    double robotAngle = 0 - Robot.navX.getFusedHeading();
    double distanceToGoal = calulateDistanceToObject(Constants.GOAL_HEIGHT, goalHeight);

    double x = distanceToGoal * Math.cos(robotAngle);
    double y = distanceToGoal * Math.sin(robotAngle);

    FieldCoordinates currentPosition = new FieldCoordinates(x, y, Robot.navX.getFusedHeading());

    return currentPosition;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
