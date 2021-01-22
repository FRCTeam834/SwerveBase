/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.swerve;

// Parameters
import frc.robot.Parameters;

// Internal libraries
import frc.robot.swerve.PID_PARAMETERS;

// Vendor Libs
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderFaults;
import com.ctre.phoenix.sensors.CANCoderStickyFaults;
import com.ctre.phoenix.sensors.MagnetFieldStrength;
import edu.wpi.first.wpilibj.controller.PIDController;

// WPI Libraries
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.geometry.Rotation2d;


public class SwerveModule {

  // Define all of the variables in the global scope
  private CANSparkMax steerMotor;
  private CANSparkMax driveMotor;
  private boolean driveMotorInverted = false;
  private CANPIDController driveMotorPID;
  private PIDController steerMotorPID;
  private CANCoder steeringCANCoder;

  // Set up the module and address each of the motor controllers
  public SwerveModule(int steerMID, int driveMID, int CANCoderID, PID_PARAMETERS T_PID_params, PID_PARAMETERS D_PID_params) {

    // CANCoder
    steeringCANCoder = new CANCoder(CANCoderID);

    // Steering motor
    steerMotor = new CANSparkMax(steerMID, CANSparkMax.MotorType.kBrushless);
    steerMotor.setOpenLoopRampRate(Parameters.currentDriverProfile.DRIVE_RAMP_RATE);
    steerMotor.setIdleMode(Parameters.currentDriverProfile.DRIVE_IDLE_MODE);

    // Steering PID controller
    steerMotorPID = new PIDController(T_PID_params.P, T_PID_params.I, T_PID_params.D);
    steerMotorPID.setP(T_PID_params.P);
    steerMotorPID.setI(T_PID_params.I);
    steerMotorPID.setD(T_PID_params.D);
    steerMotorPID.setIntegratorRange(-T_PID_params.I_ZONE, T_PID_params.I_ZONE);

    // Drive motor
    driveMotor = new CANSparkMax(driveMID, CANSparkMax.MotorType.kBrushless);
    driveMotor.setOpenLoopRampRate(Parameters.currentDriverProfile.DRIVE_RAMP_RATE);
    driveMotor.setIdleMode(Parameters.currentDriverProfile.DRIVE_IDLE_MODE);

    // Drive motor PID controller
    driveMotorPID = driveMotor.getPIDController();
    driveMotorPID.setSmartMotionAccelStrategy(CANPIDController.AccelStrategy.kSCurve, 0);
    driveMotor.getEncoder().setVelocityConversionFactor(42);
    driveMotorPID.setFF(D_PID_params.FF, 0);
    driveMotorPID.setP(D_PID_params.P, 0);
    driveMotorPID.setI(D_PID_params.I, 0);
    driveMotorPID.setD(D_PID_params.D, 0);
    driveMotorPID.setIZone(D_PID_params.I_ZONE, 0);
    driveMotorPID.setOutputRange(-D_PID_params.PEAK_OUTPUT, D_PID_params.PEAK_OUTPUT, 0);
    
  }

  public void setSteerMParams(PID_PARAMETERS PID_params) {
    steerMotorPID.setP(PID_params.P);
    steerMotorPID.setI(PID_params.I);
    steerMotorPID.setD(PID_params.D);
  }

  public void setDriveMParams(PID_PARAMETERS PID_params, double rampRate, IdleMode idleMode) {

    // PID parameters
    driveMotorPID.setFF(PID_params.FF);
    driveMotorPID.setP(PID_params.P);
    driveMotorPID.setI(PID_params.I);
    driveMotorPID.setD(PID_params.D);
    driveMotorPID.setIZone(PID_params.I_ZONE);
    driveMotorPID.setOutputRange(-PID_params.PEAK_OUTPUT, PID_params.PEAK_OUTPUT);

    // Set the motor parameters
    driveMotor.setOpenLoopRampRate(rampRate);
    driveMotor.setIdleMode(idleMode);
  }

  // Gets the steering motor for the selected module
  public CANSparkMax getSteerMotor() {
    return steerMotor;
  }

  // Gets the drive motor for the selected module
  public CANSparkMax getDriveMotor() {
    return driveMotor;
  }

  // Gets the CANCoder for the selected module
  public CANCoder getCANCoder() {
    return steeringCANCoder;
  }

  // Sets the direction of the wheel, in degrees
  public void setAngle(double targetAngle) {
 
    // Get our current position and calculate the angle from it
    double currentPosition = steeringCANCoder.getAbsolutePosition();
    double currentAngle = (currentPosition * 360.0 / Parameters.ENCODER_COUNTS_PER_REVOLUTION) % 360.0;

    // Calculate the target position for the PID loop and tell it to go there
    double targetPosition = currentPosition + deltaDegrees * Parameters.ENCODER_COUNTS_PER_REVOLUTION / 360.0;
    steerMotorPID.setSetpoint(targetPosition);

    // Tell the steering motor to move to the desired position
    steerMotor.set(steerMotorPID.calculate(measurement));

  }

  // Sets the speed of the drive motor
  public void setRawSpeed(double speed) {
    driveMotor.set(speed);
  }

  public void setSpeed(double value, ControlType controlType) {

    // Do some things
    driveMotorPID.setReference(value, controlType);
  }

  public void setDesiredState(SwerveModuleState state) {

    // Set module to the right angles and speeds
    setAngle(state.angle.getDegrees());

    // Seconds to minutes conversion * Circumference of wheel * desired m/s
    double driveRPM = (60 / (Parameters.SWERVE_WHEEL_DIA_M * 2 * Math.PI)) * state.speedMetersPerSecond;
    setSpeed(driveRPM, ControlType.kVelocity);
  }

  public SwerveModuleState getState() {
    // Get the current position of the encoder and then calculate the degrees
    double currentPosition = steeringCANCoder.getAbsolutePosition();
    double currentAngle = (currentPosition * 360.0 / Parameters.ENCODER_COUNTS_PER_REVOLUTION) % 360.0;

    // Get RPM and multiply by the circumference of the wheel in meters
    double speedMetersPerSecond = driveMotor.getEncoder().getVelocity() * (Parameters.SWERVE_WHEEL_DIA / 39.37) * Math.PI;

    return new SwerveModuleState(speedMetersPerSecond, new Rotation2d(Math.toRadians(currentAngle)));
  }

  public double getAngle() {
    return steeringCANCoder.getAbsolutePosition();
  }
  
}