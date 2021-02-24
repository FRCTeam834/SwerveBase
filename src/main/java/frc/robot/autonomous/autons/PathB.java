// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.autons;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// Import the robot
import frc.robot.Robot;
import frc.robot.commands.MoveToPosition;
// Parameters class
import frc.robot.Parameters;

//This is the auton for the Infinite Recharge at Home Auton Challenge; Part B

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathB extends SequentialCommandGroup {
  /** Creates a new PathB. */
  public PathB() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    /*
    Starting position: D1
    *detect distance to nearest ball(either d5 or d6) if distance = 1.2m its red path if = 1.5m its blue path*


    */

    if(Robot.ultrasonicSensor.withinRange(1.2, 0.1)) {
      //move to (1.2, 0) at 2 m/s; spin intake;
      addCommands(new MoveToPosition(new Pose2d(1.2, 0, Rotation2d.fromDegrees(0)), 2));
    }
    else if(Robot.ultrasonicSensor.withinRange(1.5, 0.1)) {
      addCommands();
    }

  }
}