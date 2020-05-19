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

// Internal libraries
import frc.robot.swerve.SwerveModule;
import frc.robot.Robot;

// Java libraries
import java.util.Arrays;
import java.util.Collections;

// FRC/Vendor Libs
import edu.wpi.first.wpilibj.Joystick;


public class DriveTrain extends SubsystemBase {

  SwerveModule frontLeft;
  SwerveModule frontRight;
  SwerveModule backLeft;
  SwerveModule backRight;
  
  Joystick left = new Joystick(0);
  Joystick right = new Joystick(1);

  
  // Main drivetrain class
  public DriveTrain() {

    frontLeft = new SwerveModule(Constants.FRONT_LEFT_STEER_ID, Constants.FRONT_LEFT_DRIVE_ID, Constants.FL_PID_PARAM);
    frontRight = new SwerveModule(Constants.FRONT_RIGHT_STEER_ID, Constants.FRONT_RIGHT_DRIVE_ID, Constants.FR_PID_PARAM);
    backLeft = new SwerveModule(Constants.BACK_LEFT_STEER_ID, Constants.BACK_LEFT_DRIVE_ID, Constants.BL_PID_PARAM);
    backRight = new SwerveModule(Constants.BACK_RIGHT_STEER_ID, Constants.BACK_RIGHT_DRIVE_ID, Constants.BR_PID_PARAM);

  }

  public void robotCentricDrive(double x, double y, double omega) {
    // Note: Angles range from -180 to +180 degrees CW; zero is straight ahead

    // Calculate factors once, as they are used multiple times
    double A = x - (omega * (Constants.DRIVE_LENGTH * 2.0));
    double B = x + (omega * (Constants.DRIVE_LENGTH * 2.0));
    double C = y - (omega * (Constants.DRIVE_WIDTH * 2.0));
    double D = y + (omega * (Constants.DRIVE_WIDTH * 2.0));

    // Start calculating module positions
    // Speed calculations

    // Calculate all of the speeds
    double FLSpeed = Math.sqrt( speed(B, D) );
    double FRSpeed = Math.sqrt( speed(B, C) );
    double BLSpeed = Math.sqrt( speed(A, D) );
    double BRSpeed = Math.sqrt( speed(A, C) );

    // Get the highest speed. If it is greater than 1, divide all of the other values by it
    double highestSpeed = Collections.max(Arrays.asList(FLSpeed, FRSpeed, BLSpeed, BRSpeed, 1.0));;

    // Front Left
    frontLeft.setDriveSpeed(FLSpeed / highestSpeed);

    // Front Right
    frontRight.setDriveSpeed(FRSpeed / highestSpeed);

    // Back Left
    backLeft.setDriveSpeed(BLSpeed / highestSpeed);
    
    // Back Right
    backRight.setDriveSpeed(BRSpeed / highestSpeed);

    // Angle calculations
    frontLeft.setDriveAngle(  angle(B, D) );
    frontRight.setDriveAngle( angle(B, C) );
    backLeft.setDriveAngle(   angle(A, D) );
    backRight.setDriveAngle(  angle(A, C) );

  }

  public void fieldCentricDrive(double x, double y, double omega) {
    
    // Get the current angle of robot
    double robotAngle = Math.toRadians(0 - Robot.navX.getFusedHeading());

    // Do offset calculations. Use temp so that y is not changed while it is still important
    double temp = (y * Math.cos(robotAngle)) - (x * Math.sin(robotAngle));
    x = (y * Math.sin(robotAngle)) + (x * Math.cos(robotAngle));
    y = temp;

    // Drive the robot in the offset directions
    robotCentricDrive(x, y, omega);
  }

  // Sets the drive modules in an X pattern
  public void lockemUp() {
    
    // Make sure all the drive motors are shut off
    frontLeft.setDriveSpeed(0);
    frontRight.setDriveSpeed(0);
    backLeft.setDriveSpeed(0);
    backRight.setDriveSpeed(0);

    // Set the angles to be an X
    frontLeft.setDriveAngle(-45);
    frontRight.setDriveAngle(45);
    backLeft.setDriveAngle(45);
    backRight.setDriveAngle(-45);

  }

  // Actually moves the robot based on stick positions. Scaled by a multipiler. Make sure the multipler is 0 < x <= 1
  public void drive(double maxSpeed, boolean fieldCentric, boolean lockemUp) {
    double x = left.getX();
    double y = -left.getY();
    double omega = right.getX() * Constants.TURN_SCALE;

    // Add a small deadzone on the joysticks
    if (Math.abs(x) < Constants.JOYSTICK_DEADZONE)
      x = 0.0;
    if (Math.abs(y) < Constants.JOYSTICK_DEADZONE)
      y = 0.0;
    if (Math.abs(omega) < Constants.JOYSTICK_DEADZONE * Constants.TURN_SCALE)
      omega = 0.0;

    x = x * maxSpeed;
    y = x * maxSpeed;
    omega = omega * maxSpeed;

    if (lockemUp) {
      if (x == 0 && y == 0 && omega == 0) {
        this.lockemUp();
      }
      else {
        if (fieldCentric) {
          this.fieldCentricDrive(x, y, omega);
        }
        else {
          this.robotCentricDrive(x, y, omega);
        }
      }
    }
    else {
      if (fieldCentric) {
        this.fieldCentricDrive(x, y, omega);
      }
      else {
        this.robotCentricDrive(x, y, omega);
      }
    }
  }

	private double speed(double val1, double val2) {
		return Math.hypot(val1, val2);
	}

	private double angle(double val1, double val2) {
		return Math.toDegrees(Math.atan2(val1, val2));
	}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
