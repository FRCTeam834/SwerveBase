/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.swerve;

// Constants
import frc.robot.Constants;

// Vendor Libs
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;

public class SwerveModule {

  private WPI_TalonSRX steerMotor;
  private CANSparkMax driveMotor;
  private boolean driveMotorInverted = false;
  
  public SwerveModule(int steerID, int driveID) {

    // Steering motor
    steerMotor = new WPI_TalonSRX(steerID);
    steerMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.Analog, 0, 10);
    //steerMotor
    steerMotor.config_kF(Constants.PID_IDX, Constants.PID_PARAM.F, Constants.PID_TIMEOUT);
    steerMotor.config_kP(Constants.PID_IDX, Constants.PID_PARAM.P, Constants.PID_TIMEOUT);
    steerMotor.config_kI(Constants.PID_IDX, Constants.PID_PARAM.I, Constants.PID_TIMEOUT);
    steerMotor.config_kD(Constants.PID_IDX, Constants.PID_PARAM.D, Constants.PID_TIMEOUT);
    // Drive motor
    driveMotor = new CANSparkMax(driveID, CANSparkMax.MotorType.kBrushless);
    driveMotor.setOpenLoopRampRate(Constants.DRIVE_RAMP_RATE);
    driveMotor.setIdleMode(IdleMode.kBrake);
    
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

  
}