/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


// Constants
import frc.robot.Constants;

// Vendor Libs
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;

public class SwerveModule {

  private WPI_TalonSRX steerMotor;
  private CANSparkMax driveMotor;
  
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

  }

  // Gets the steering motor for the selected module
  public WPI_TalonSRX getSteerMotor() {
    return steerMotor;
  }

  // Gets the drive motor for the selected module
  public CANSparkMax getDriveMotor() {
    return driveMotor;
  }

  // Sets the direction of the wheel
  public void setHeading(double angle) {
    //float encoder_position = 1023 * (angle/360);
    //steerMotor.set(ControlMode.Position, encoder_position);
  }

  // Sets the speed of the drive motor
  public void setDrive(double speed) {
    driveMotor.set(speed);
  }

  
}