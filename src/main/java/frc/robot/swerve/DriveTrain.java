/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.swerve;


// WPI libraries
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

// Import constants
import frc.robot.Constants;

// Import robot
import frc.robot.Robot;

// Internal libraries
import frc.robot.swerve.SwerveModule;
import frc.robot.automove.FieldCoordinates;


public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new Drivetrain.
   */

  SwerveModule frontLeft;
  SwerveModule frontRight;
  SwerveModule backLeft;
  SwerveModule backRight;

  Translation2d FLLocation = new Translation2d(Constants.DRIVE_LENGTH / 2, Constants.DRIVE_WIDTH / 2);
  Translation2d FRLocation = new Translation2d(Constants.DRIVE_LENGTH / 2, -Constants.DRIVE_WIDTH / 2);
  Translation2d BLLocation = new Translation2d(-Constants.DRIVE_LENGTH / 2, Constants.DRIVE_WIDTH / 2);
  Translation2d BRLocation = new Translation2d(-Constants.DRIVE_LENGTH / 2, -Constants.DRIVE_WIDTH / 2);

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(FLLocation, FRLocation, BLLocation, BRLocation);


  public DriveTrain() {

    frontLeft = new SwerveModule(Constants.FRONT_LEFT_STEER_ID, Constants.FRONT_LEFT_DRIVE_ID, Constants.FL_T_PID_PARAM, Constants.FL_D_PID_PARAM);
    frontRight = new SwerveModule(Constants.FRONT_RIGHT_STEER_ID, Constants.FRONT_RIGHT_DRIVE_ID, Constants.FR_T_PID_PARAM, Constants.FR_D_PID_PARAM);
    backLeft = new SwerveModule(Constants.BACK_LEFT_STEER_ID, Constants.BACK_LEFT_DRIVE_ID, Constants.BL_T_PID_PARAM, Constants.BL_D_PID_PARAM);
    backRight = new SwerveModule(Constants.BACK_RIGHT_STEER_ID, Constants.BACK_RIGHT_DRIVE_ID, Constants.BR_T_PID_PARAM, Constants.BR_D_PID_PARAM);

  }
  
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getAngle());

  public Rotation2d getAngle() {
    // Adjusted angle
    return Rotation2d.fromDegrees(Robot.navX.getFusedHeading());
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = kinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rot, getAngle())
            : new ChassisSpeeds(xSpeed, ySpeed, rot)
    );
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.MAX_SPEED);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void lockemUp() {
    // Fix later
  }

  /**
   * Updates the field relative position of the robot.
   */
  public void updateOdometry() {
    odometry.update(
        getAngle(),
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState()
    );
  }
/*
  public void resetOdometry(FieldCoordinates currentPosition) {
    odometry.resetPose()

  }
*/
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
