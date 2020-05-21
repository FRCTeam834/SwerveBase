/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.swerve;

// Constants
import frc.robot.Constants;

// Internal libraries
import frc.robot.swerve.PID_PARAMETERS;

// Vendor Libs
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;

// WPI Libraries
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;


public class SwerveModule {

  private WPI_TalonSRX steerMotor;
  private CANSparkMax driveMotor;
  private boolean driveMotorInverted = false;
  private CANPIDController driveMotorPID;
  

  public SwerveModule(int steerID, int driveID, PID_PARAMETERS T_PID_params, PID_PARAMETERS D_PID_params) {

    // Steering motor
    steerMotor = new WPI_TalonSRX(steerID);
    steerMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.Analog, 0, 10);
    steerMotor.config_kF(Constants.PID_IDX, T_PID_params.FF, Constants.PID_TIMEOUT);
    steerMotor.config_kP(Constants.PID_IDX, T_PID_params.P, Constants.PID_TIMEOUT);
    steerMotor.config_kI(Constants.PID_IDX, T_PID_params.I, Constants.PID_TIMEOUT);
    steerMotor.config_kD(Constants.PID_IDX, T_PID_params.D, Constants.PID_TIMEOUT);

    // Drive motor
    driveMotor = new CANSparkMax(driveID, CANSparkMax.MotorType.kBrushless);
    driveMotor.setOpenLoopRampRate(Constants.DRIVE_RAMP_RATE);
    driveMotor.setIdleMode(IdleMode.kBrake);

    driveMotorPID = new CANPIDController(driveMotor);
    driveMotor.getEncoder().setVelocityConversionFactor(42);
    driveMotorPID.setFF(D_PID_params.FF);
    driveMotorPID.setP(D_PID_params.P);
    driveMotorPID.setI(D_PID_params.I);
    driveMotorPID.setD(D_PID_params.D);
    driveMotorPID.setIZone(D_PID_params.I_ZONE);
    driveMotorPID.setOutputRange(-D_PID_params.PEAK_OUTPUT, D_PID_params.PEAK_OUTPUT);
    
  }

  public void setSteerPIDParams(PID_PARAMETERS PID_params) {
    steerMotor.config_kF(Constants.PID_IDX, PID_params.FF, Constants.PID_TIMEOUT);
    steerMotor.config_kP(Constants.PID_IDX, PID_params.P, Constants.PID_TIMEOUT);
    steerMotor.config_kI(Constants.PID_IDX, PID_params.I, Constants.PID_TIMEOUT);
    steerMotor.config_kD(Constants.PID_IDX, PID_params.D, Constants.PID_TIMEOUT);
  }

  public void setDrivePIDParams(PID_PARAMETERS PID_params) {
    driveMotorPID.setFF(PID_params.FF);
    driveMotorPID.setP(PID_params.P);
    driveMotorPID.setI(PID_params.I);
    driveMotorPID.setD(PID_params.D);
    driveMotorPID.setIZone(PID_params.I_ZONE);
    driveMotorPID.setOutputRange(-PID_params.PEAK_OUTPUT, PID_params.PEAK_OUTPUT);
  }

  // Gets the steering motor for the selected module
  public WPI_TalonSRX getSteerMotor() {
    return steerMotor;
  }

  // Gets the drive motor for the selected module
  public CANSparkMax getDriveMotor() {
    return driveMotor;
  }

  // Sets the direction of the wheel, in degrees
  public void setDriveAngle(double targetAngle) {
 
    // Get our current position and calculate the angle from it
    double currentPosition = steerMotor.getSelectedSensorPosition(0);
    double currentAngle = (currentPosition * 360.0 / Constants.ENCODER_COUNTS_PER_REVOLUTION) % 360.0;
    
    // The angle from the encoder is in the range [0, 360], but the swerve computations
    // return angles in the range [-180, 180], so transform the encoder angle to this range
    if (currentAngle > 180.0) {
      currentAngle -= 360.0;
    }

    // The degrees we need to turn
    double deltaDegrees = targetAngle - currentAngle;

    // If we need to turn more than 180 degrees, it's faster to turn in the opposite direction
    if (Math.abs(deltaDegrees) > 180.0) {
      deltaDegrees -= 360.0 * Math.signum(deltaDegrees);
    }

    // If we need to turn more than 90 degrees, we can invert the motor direction instead and
    // only rotate by the complement
    if (Math.abs(deltaDegrees) > 90.0) {
      deltaDegrees -= 180.0 * Math.signum(deltaDegrees);
      driveMotor.setInverted(!driveMotorInverted);
      driveMotorInverted = !driveMotorInverted;
      
    }

    // Calculate the target position for the steering motor and tell it to go there
    double targetPosition = currentPosition + deltaDegrees * Constants.ENCODER_COUNTS_PER_REVOLUTION / 360.0;
    steerMotor.set(ControlMode.Position, targetPosition);

  }

  // Sets the speed of the drive motor
  public void setDriveSpeed(double speed) {
    driveMotor.set(speed);
  }

  public void setDriveSpeedM(double metersPerSec) {
    // Do some things
    

  }

  public void setDesiredState(SwerveModuleState state) {
    // Set module to the right angles and speeds
    setDriveAngle(state.angle.getDegrees()); 
    setDriveSpeedM(state.speedMetersPerSecond);

  }

  public SwerveModuleState getState() {

    // Get the current position of the encoder and then calculate the degrees
    double currentPosition = steerMotor.getSelectedSensorPosition(0);
    double currentAngle = (currentPosition * 360.0 / Constants.ENCODER_COUNTS_PER_REVOLUTION) % 360.0;

    // Get RPM and multiply by the circumference of the wheel in meters
    double speedMetersPerSecond = driveMotor.getEncoder().getVelocity() * (Constants.SWERVE_WHEEL_DIA / 39.37) * Math.PI;

    return new SwerveModuleState(speedMetersPerSecond, new Rotation2d(Math.toRadians(currentAngle)));
  }
  
}