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
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

// Import Parameters
import frc.robot.Parameters;

// Import robot
import frc.robot.Robot;


public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new Drivetrain.
   */

  // Create the modules
  SwerveModule frontLeft;
  SwerveModule frontRight;
  SwerveModule backLeft;
  SwerveModule backRight;

  // Define their position (relative to center of robot)
  Translation2d FLLocation = new Translation2d(Parameters.DRIVE_LENGTH / 2, Parameters.DRIVE_WIDTH / 2);
  Translation2d FRLocation = new Translation2d(Parameters.DRIVE_LENGTH / 2, -Parameters.DRIVE_WIDTH / 2);
  Translation2d BLLocation = new Translation2d(-Parameters.DRIVE_LENGTH / 2, Parameters.DRIVE_WIDTH / 2);
  Translation2d BRLocation = new Translation2d(-Parameters.DRIVE_LENGTH / 2, -Parameters.DRIVE_WIDTH / 2);

  // Create the drivetrain map
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(FLLocation, FRLocation, BLLocation, BRLocation);

  // Setup the drivetrain
  public DriveTrain() {

    // Create each swerve module instance
    frontLeft =  new SwerveModule(Parameters.FRONT_LEFT_STEER_ID,  Parameters.FRONT_LEFT_DRIVE_ID,  Parameters.FRONT_LEFT_CODER_ID,  Parameters.FL_T_PID_PARAM, Parameters.FL_D_PID_PARAM);
    frontRight = new SwerveModule(Parameters.FRONT_RIGHT_STEER_ID, Parameters.FRONT_RIGHT_DRIVE_ID, Parameters.FRONT_RIGHT_CODER_ID, Parameters.FR_T_PID_PARAM, Parameters.FR_D_PID_PARAM);
    backLeft =   new SwerveModule(Parameters.BACK_LEFT_STEER_ID,   Parameters.BACK_LEFT_DRIVE_ID,   Parameters.BACK_LEFT_CODER_ID,   Parameters.BL_T_PID_PARAM, Parameters.BL_D_PID_PARAM);
    backRight =  new SwerveModule(Parameters.BACK_RIGHT_STEER_ID,  Parameters.BACK_RIGHT_DRIVE_ID,  Parameters.BACK_RIGHT_CODER_ID,  Parameters.BR_T_PID_PARAM, Parameters.BR_D_PID_PARAM);

  }
  
  // Declare the odometry of the robot
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, Robot.navX.getFusedRotation2d());

  // Less complicated version, runs with the robot's actual center
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    // Define an accumulator for the states
    SwerveModuleState[] swerveModuleStates;

    // Set up the modules
    if (fieldRelative) {
      swerveModuleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Robot.navX.getFusedRotation2d()));
    }
    else {
      swerveModuleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
    }
    
    // Setup the max speed of each module
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Parameters.MAX_MODULE_SPEED);

    // Set each of the modules to their state
    frontLeft.setDesiredState( swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(  swerveModuleStates[2]);
    backRight.setDesiredState( swerveModuleStates[3]);

    // Update the robot's odometry
    updateOdometry();
    
  }
  
  // More complicated, runs with a custom center relative to robot
  public void driveRelativeCenter(double xSpeed, double ySpeed, double rot, boolean fieldRelative, Translation2d relativeCenter) {

    // Define an accumulator for the states
    SwerveModuleState[] swerveModuleStates;

    // Set up the modules
    if (fieldRelative) {
      swerveModuleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Robot.navX.getFusedRotation2d()), relativeCenter);
    }
    else {
      swerveModuleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot), relativeCenter);
    }
    
    // Setup the max speed of each module
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Parameters.MAX_MODULE_SPEED);

    // Set each of the modules to their optimized state
    frontLeft.setDesiredState( swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(  swerveModuleStates[2]);
    backRight.setDesiredState( swerveModuleStates[3]);

    // Update the robot's odometry
    updateOdometry();

  }

  // More complicated, runs with a custom center relative to field
  public void driveAbsoluteCenter(double xSpeed, double ySpeed, double rot, boolean fieldRelative, Translation2d absoluteCenter) {

    // Get the current position of the robot on the field
    Pose2d currentPose = odometry.getPoseMeters();

    // Create a pose of the field coordinate
    Pose2d fieldCenterPose = new Pose2d(absoluteCenter, new Rotation2d(0));

    // Convert the field coordinates to robot coordinates
    Translation2d centerOfRotation = fieldCenterPose.relativeTo(currentPose).getTranslation();

    // Define an accumulator for the states
    SwerveModuleState[] swerveModuleStates;

    // Set up the modules
    if (fieldRelative) {
      swerveModuleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Robot.navX.getFusedRotation2d()), centerOfRotation);
    }
    else {
      swerveModuleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot), centerOfRotation);
    }
    
    // Setup the max speed of each module
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Parameters.MAX_MODULE_SPEED);

    // Set each of the modules to their optimized state
    frontLeft.setDesiredState( swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(  swerveModuleStates[2]);
    backRight.setDesiredState( swerveModuleStates[3]);

    // Update the robot's odometry
    updateOdometry();

  }

  

  public void lockemUp() {
    // Makes an X pattern with the swerve base
    // Set the modules to 45 degree angles
    frontLeft.setAngle(-45);
    frontRight.setAngle(45);
    backLeft.setAngle(45);
    backRight.setAngle(-45);

    // Halt all the motors and hold them there
    frontLeft.setSpeed(0);
    frontRight.setSpeed(0);
    backRight.setSpeed(0);
    backLeft.setSpeed(0);

  }

  /**
   * Updates the field relative position of the robot.
   */
  public void updateOdometry() {
    odometry.update(
        Robot.navX.getFusedRotation2d(),
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState()
    );
  }

  public void resetOdometry(Pose2d currentPosition) {
    odometry.resetPosition(currentPosition, Robot.navX.getFusedRotation2d());
  }

  public void updateParameters() {

    // Update the PID parameters with the new driver profile values
    Parameters.FL_T_PID_PARAM.setPeakOutput(Parameters.CURRENT_DRIVER_PROFILE.MAX_SPEED);
    Parameters.FR_T_PID_PARAM.setPeakOutput(Parameters.CURRENT_DRIVER_PROFILE.MAX_SPEED);
    Parameters.BL_T_PID_PARAM.setPeakOutput(Parameters.CURRENT_DRIVER_PROFILE.MAX_SPEED);
    Parameters.BR_T_PID_PARAM.setPeakOutput(Parameters.CURRENT_DRIVER_PROFILE.MAX_SPEED);

    Parameters.FL_D_PID_PARAM.setPeakOutput(Parameters.CURRENT_DRIVER_PROFILE.MAX_SPEED);
    Parameters.FR_D_PID_PARAM.setPeakOutput(Parameters.CURRENT_DRIVER_PROFILE.MAX_SPEED);
    Parameters.BL_D_PID_PARAM.setPeakOutput(Parameters.CURRENT_DRIVER_PROFILE.MAX_SPEED);
    Parameters.BR_D_PID_PARAM.setPeakOutput(Parameters.CURRENT_DRIVER_PROFILE.MAX_SPEED);

    // Set steering parameters
    frontLeft.setSteerMParams(Parameters.FL_T_PID_PARAM,  Parameters.CURRENT_DRIVER_PROFILE.STEER_IDLE_MODE);
    frontRight.setSteerMParams(Parameters.FR_T_PID_PARAM, Parameters.CURRENT_DRIVER_PROFILE.STEER_IDLE_MODE);
    backLeft.setSteerMParams(Parameters.BL_T_PID_PARAM,   Parameters.CURRENT_DRIVER_PROFILE.STEER_IDLE_MODE);
    backRight.setSteerMParams(Parameters.BR_T_PID_PARAM,  Parameters.CURRENT_DRIVER_PROFILE.STEER_IDLE_MODE);

    // Set driving parameters
    frontLeft.setDriveMParams(Parameters.FL_D_PID_PARAM,  Parameters.CURRENT_DRIVER_PROFILE.DRIVE_IDLE_MODE);
    frontRight.setDriveMParams(Parameters.FR_D_PID_PARAM, Parameters.CURRENT_DRIVER_PROFILE.DRIVE_IDLE_MODE);
    backLeft.setDriveMParams(Parameters.BL_D_PID_PARAM,   Parameters.CURRENT_DRIVER_PROFILE.DRIVE_IDLE_MODE);
    backRight.setDriveMParams(Parameters.BR_D_PID_PARAM,  Parameters.CURRENT_DRIVER_PROFILE.DRIVE_IDLE_MODE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
