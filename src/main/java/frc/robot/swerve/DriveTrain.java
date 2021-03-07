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
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.estimator.SwerveDrivePoseEstimator;
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
  Translation2d FL_POS = new Translation2d(Parameters.driveTrain.dimensions.DRIVE_LENGTH / 2, Parameters.driveTrain.dimensions.DRIVE_WIDTH / 2);
  Translation2d FR_POS = new Translation2d(Parameters.driveTrain.dimensions.DRIVE_LENGTH / 2, -Parameters.driveTrain.dimensions.DRIVE_WIDTH / 2);
  Translation2d BL_POS = new Translation2d(-Parameters.driveTrain.dimensions.DRIVE_LENGTH / 2, Parameters.driveTrain.dimensions.DRIVE_WIDTH / 2);
  Translation2d BR_POS = new Translation2d(-Parameters.driveTrain.dimensions.DRIVE_LENGTH / 2, -Parameters.driveTrain.dimensions.DRIVE_WIDTH / 2);

  // Create the drivetrain map
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(FL_POS, FR_POS, BL_POS, BR_POS);

  // Pose estimator
  private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(Robot.navX.getFusedRotation2d(), Parameters.positions.STARTING_POS, kinematics, Parameters.driveTrain.movement.POSE_STD_DEV, Parameters.driveTrain.movement.ENCODER_GYRO_DEV, Parameters.driveTrain.movement.VISION_DEVIATION);

  // Holomonic drive controller
  private HolonomicDriveController driveController = new HolonomicDriveController(Parameters.driveTrain.movement.MOVEMENT_PID, Parameters.driveTrain.movement.MOVEMENT_PID, Parameters.driveTrain.movement.ROTATION_PID);

  // Setup the drivetrain
  public DriveTrain() {

    // Create each swerve module instance
    frontLeft  = new SwerveModule(Parameters.driveTrain.can.FL_STEER_ID,  Parameters.driveTrain.can.FL_DRIVE_ID,  Parameters.driveTrain.can.FL_CODER_ID,  Parameters.driveTrain.pid.FL_STEER_PID, Parameters.driveTrain.pid.FL_DRIVE_PID);
    frontRight = new SwerveModule(Parameters.driveTrain.can.FR_STEER_ID, Parameters.driveTrain.can.FR_DRIVE_ID, Parameters.driveTrain.can.FR_CODER_ID, Parameters.driveTrain.pid.FR_STEER_PID, Parameters.driveTrain.pid.FR_DRIVE_PID);
    backLeft   = new SwerveModule(Parameters.driveTrain.can.BL_STEER_ID,   Parameters.driveTrain.can.BL_DRIVE_ID,   Parameters.driveTrain.can.BL_CODER_ID,   Parameters.driveTrain.pid.BL_STEER_PID, Parameters.driveTrain.pid.BL_DRIVE_PID);
    backRight  = new SwerveModule(Parameters.driveTrain.can.BR_STEER_ID,  Parameters.driveTrain.can.BR_DRIVE_ID,  Parameters.driveTrain.can.BR_CODER_ID,  Parameters.driveTrain.pid.BR_STEER_PID, Parameters.driveTrain.pid.BR_DRIVE_PID);

    // Center the odometry of the robot
    resetOdometry(Parameters.positions.STARTING_POS);
  }


  // Less complicated version, runs with the robot's actual center
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    // Set up the modules
    if (fieldRelative) {
      setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Robot.navX.getFusedRotation2d()));
    }
    else {
      setModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
    }
  }


  // More complicated, runs with a custom center relative to robot
  public void driveRelativeCenter(double xSpeed, double ySpeed, double rot, boolean fieldRelative, Translation2d relativeCenter) {

    // Drive with selected mode
    if (fieldRelative) {
      setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Robot.navX.getFusedRotation2d()), relativeCenter);
    }
    else {
      setModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot), relativeCenter);
    }
  }


  // More complicated, runs with a custom center relative to field
  public void driveAbsoluteCenter(double xSpeed, double ySpeed, double rot, boolean fieldRelative, Translation2d absoluteCenter) {

    // Get the current position of the robot on the field
    Pose2d currentPose = poseEstimator.getEstimatedPosition();

    // Create a pose of the field coordinate
    Pose2d fieldCenterPose = new Pose2d(absoluteCenter, new Rotation2d(0));

    // Convert the field coordinates to robot coordinates
    Translation2d centerOfRotation = fieldCenterPose.relativeTo(currentPose).getTranslation();

    // Set up the modules
    if (fieldRelative) {
      setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Robot.navX.getFusedRotation2d()), centerOfRotation);
    }
    else {
      setModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot), centerOfRotation);
    }
  }


  // Sets all of the states of the modules and updates the odometry of the robot
  private void setModuleStates(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation) {

    // Get the module states
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds, centerOfRotation);

    // Setup the max speed of each module
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Parameters.driveTrain.maximums.MAX_MODULE_SPEED);

    // Set each of the modules to their optimized state
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);

    // Update the robot's odometry
    updateOdometry();
  }


  // Sets all of the states of the modules and updates the odometry of the robot
  private void setModuleStates(ChassisSpeeds chassisSpeeds) {

    // Get the module states
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

    // Setup the max speed of each module
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Parameters.driveTrain.maximums.MAX_MODULE_SPEED);

    // Set each of the modules to their optimized state
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);

    // Update the robot's odometry
    updateOdometry();
  }


  // Locks the modules of the robot to prevent movement
  public void lockemUp() {

    // Makes an X pattern with the swerve base
    // Set the modules to 45 degree angles
    frontLeft.setDesiredAngle(-45);
    frontRight.setDesiredAngle(45);
    backLeft.setDesiredAngle(45);
    backRight.setDesiredAngle(-45);

    // Halt all the motors and hold them there
    frontLeft.setDesiredSpeed(0);
    frontRight.setDesiredSpeed(0);
    backRight.setDesiredSpeed(0);
    backLeft.setDesiredSpeed(0);
  }


  // Updates the field relative position of the robot.
  public void updateOdometry() {
    poseEstimator.update(
      Robot.navX.getFusedRotation2d(),
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
    );
  }


  // Resets the odometry of the robot
  public void resetOdometry(Pose2d currentPosition) {
    poseEstimator.resetPosition(currentPosition, Robot.navX.getFusedRotation2d());
  }


  // Updates the pose estimator with a vision position
  public void visionPositionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }


  // Function to have the drivetrain follow a set trajectory
  public void trajectoryFollow(Pose2d desiredPosition, double linearVelocity) {

    // Calculate the speeds for the chassis
    ChassisSpeeds adjustedSpeeds = driveController.calculate(poseEstimator.getEstimatedPosition(), desiredPosition, linearVelocity, desiredPosition.getRotation());

    // Set the modules to move at those speeds
    setModuleStates(adjustedSpeeds);
  }


  // Check if the robot is at the reference point of the trajectory
  public boolean atTrajectoryReference() {

    // Return if the trajectory is complete
    return driveController.atReference();
  }


  // Updates all of the steering parameters, such as PID loops and driver settings
  public void updateParameters() {

    // Update the PID parameters with the new driver profile values
    Parameters.driveTrain.pid.FL_STEER_PID.setPeakOutput(Parameters.driver.CURRENT_PROFILE.MAX_SPEED);
    Parameters.driveTrain.pid.FR_STEER_PID.setPeakOutput(Parameters.driver.CURRENT_PROFILE.MAX_SPEED);
    Parameters.driveTrain.pid.BL_STEER_PID.setPeakOutput(Parameters.driver.CURRENT_PROFILE.MAX_SPEED);
    Parameters.driveTrain.pid.BR_STEER_PID.setPeakOutput(Parameters.driver.CURRENT_PROFILE.MAX_SPEED);

    Parameters.driveTrain.pid.FL_DRIVE_PID.setPeakOutput(Parameters.driver.CURRENT_PROFILE.MAX_SPEED);
    Parameters.driveTrain.pid.FR_DRIVE_PID.setPeakOutput(Parameters.driver.CURRENT_PROFILE.MAX_SPEED);
    Parameters.driveTrain.pid.BL_DRIVE_PID.setPeakOutput(Parameters.driver.CURRENT_PROFILE.MAX_SPEED);
    Parameters.driveTrain.pid.BR_DRIVE_PID.setPeakOutput(Parameters.driver.CURRENT_PROFILE.MAX_SPEED);

    // Set steering parameters
    frontLeft.setSteerMParams(Parameters.driveTrain.pid.FL_STEER_PID,  Parameters.driver.CURRENT_PROFILE.STEER_IDLE_MODE);
    frontRight.setSteerMParams(Parameters.driveTrain.pid.FR_STEER_PID, Parameters.driver.CURRENT_PROFILE.STEER_IDLE_MODE);
    backLeft.setSteerMParams(Parameters.driveTrain.pid.BL_STEER_PID,   Parameters.driver.CURRENT_PROFILE.STEER_IDLE_MODE);
    backRight.setSteerMParams(Parameters.driveTrain.pid.BR_STEER_PID,  Parameters.driver.CURRENT_PROFILE.STEER_IDLE_MODE);

    // Set driving parameters
    frontLeft.setDriveMParams(Parameters.driveTrain.pid.FL_DRIVE_PID,  Parameters.driver.CURRENT_PROFILE.DRIVE_IDLE_MODE);
    frontRight.setDriveMParams(Parameters.driveTrain.pid.FR_DRIVE_PID, Parameters.driver.CURRENT_PROFILE.DRIVE_IDLE_MODE);
    backLeft.setDriveMParams(Parameters.driveTrain.pid.BL_DRIVE_PID,   Parameters.driver.CURRENT_PROFILE.DRIVE_IDLE_MODE);
    backRight.setDriveMParams(Parameters.driveTrain.pid.BR_DRIVE_PID,  Parameters.driver.CURRENT_PROFILE.DRIVE_IDLE_MODE);
  }


  // Sets all of the modules to treat their current position as the zero position.
  public void zeroEncoders() {

    // Go through the CANCoders, setting each to zero
    frontLeft.setEncoderOffset(0);
    frontRight.setEncoderOffset(0);
    backLeft.setEncoderOffset(0);
    backRight.setEncoderOffset(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
