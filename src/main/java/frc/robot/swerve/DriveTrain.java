/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Constants
import frc.robot.Constants;

// Vendor Libs


public class DriveTrain extends SubsystemBase {

  SwerveModule frontLeft;
  SwerveModule frontRight;
  SwerveModule backLeft;
  SwerveModule backRight;

  
  // Main drivetrain class
  public DriveTrain() {

    frontLeft = new SwerveModule(Constants.FRONT_LEFT_STEER_ID, Constants.FRONT_LEFT_DRIVE_ID);
    frontRight = new SwerveModule(Constants.FRONT_RIGHT_STEER_ID, Constants.FRONT_RIGHT_DRIVE_ID);
    backLeft = new SwerveModule(Constants.BACK_LEFT_STEER_ID, Constants.BACK_LEFT_DRIVE_ID);
    backRight = new SwerveModule(Constants.BACK_RIGHT_STEER_ID, Constants.BACK_RIGHT_DRIVE_ID);

  }

  public void goSomewhere(double direction, double drive_speed) {

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
