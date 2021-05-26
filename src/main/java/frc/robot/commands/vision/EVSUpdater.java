/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*
package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class EVSUpdater extends CommandBase {

  public EVSUpdater() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.EVSNetworkTables);
    addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.EVSNetworkTables.getVisionTable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //We just wanna run the vision, you know
    try {
      if (Robot.EVSNetworkTables.getGoalArray().get(0).get(1) > 360) {

        Robot.driveTrain.setDrive(-.08, .08);
        System.out.println("turn left");

      } else if (Robot.EVSNetworkTables.getGoalArray().get(0).get(1) < 290) {

        System.out.println("turn right");
        Robot.driveTrain.setDrive(.08, -.08);

      }

      else {

        Robot.driveTrain.stop();
      }
    } catch (Exception e) {

    }

  }

  // Called once the command ends or is interrupted.
//  @Override
//  public void end(boolean interrupted) {
//
//    //returns the amount of balls that are currently in the shot, and the values inside the first array.
//    //System.out.println("Vision ArrayList Size: " + RobotContainer.EVSNetworkTables.getArray().size());
//    try {
//    
//      System.out.println("Values in the Array: " + RobotContainer.EVSNetworkTables.getArray().get(1).get(0) + ", "
//          + RobotContainer.EVSNetworkTables.getArray().get(1).get(1) + ", " + RobotContainer.EVSNetworkTables.getArray().get(1).get(2)
//          + ", " + RobotContainer.EVSNetworkTables.getArray().get(1).get(3) + ", "
//          + RobotContainer.EVSNetworkTables.getArray().get(1).get(4));
//    
//    } catch (Exception e) {
//    
//      //Don't do anything.
//    }
//
    System.out.println("");
    System.out.println(Robot.EVSNetworkTables.getPowerCellArray());
    System.out.println(Robot.EVSNetworkTables.getGoalArray());
    System.out.println("");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
*/