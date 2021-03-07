/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.swerve;

// Parameters
import frc.robot.Parameters;

// Vendor Libs
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANEncoder;

// WPI Libraries
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;


public class SwerveModule {

  // Define all of the variables in the global scope
  private CANSparkMax steerMotor;
  private CANSparkMax driveMotor;
  private ProfiledPIDController steerMotorPID;
  private PIDController driveMotorPID;
  private SimpleMotorFeedforward driveMotorFF;
  private SimpleMotorFeedforward steerMotorFF;
  private CANCoder steerCANCoder;
  private CANEncoder driveMotorEncoder;
  private double cancoderOffset;

  // Set up the module and address each of the motor controllers
  public SwerveModule(int steerMID, int driveMID, int CANCoderID, PID_PARAMS T_PID_params, PID_PARAMS D_PID_params) {

    // CANCoder
    steerCANCoder = new CANCoder(CANCoderID);
    steerCANCoder.setPositionToAbsolute();
    steerCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    steerCANCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

    // Steering motor
    steerMotor = new CANSparkMax(steerMID, CANSparkMax.MotorType.kBrushless);
    steerMotor.setOpenLoopRampRate(Parameters.driver.CURRENT_PROFILE.DRIVE_RAMP_RATE);
    steerMotor.setIdleMode(Parameters.driver.CURRENT_PROFILE.DRIVE_IDLE_MODE);

    // Steering PID controller
    steerMotorPID = new ProfiledPIDController(T_PID_params.P, T_PID_params.I, T_PID_params.D, new TrapezoidProfile.Constraints(Math.toRadians(Parameters.driveTrain.maximums.MAX_MODULE_ANGULAR_VELOCITY), Math.toRadians(Parameters.driveTrain.maximums.MAX_MODULE_ANGULAR_ACCEL)));
    steerMotorPID.setP(T_PID_params.P);
    steerMotorPID.setI(T_PID_params.I);
    steerMotorPID.setD(T_PID_params.D);
    //steerMotorPID.setIntegratorRange(-T_PID_params.I_ZONE, T_PID_params.I_ZONE);
    steerMotorPID.enableContinuousInput(-Math.PI, Math.PI);

    // Steering motor feed forward
    steerMotorFF = new SimpleMotorFeedforward(T_PID_params.SFF, T_PID_params.VFF);

    // Drive motor
    driveMotor = new CANSparkMax(driveMID, CANSparkMax.MotorType.kBrushless);
    driveMotor.setOpenLoopRampRate(Parameters.driver.CURRENT_PROFILE.DRIVE_RAMP_RATE);
    driveMotor.setIdleMode(Parameters.driver.CURRENT_PROFILE.DRIVE_IDLE_MODE);

    // Drive motor PID controller
    driveMotorPID = new PIDController(D_PID_params.P, D_PID_params.I, D_PID_params.D);

    // Drive motor encoder
    driveMotorEncoder = driveMotor.getEncoder(EncoderType.kHallSensor, 42);
    driveMotorEncoder.setVelocityConversionFactor(Math.PI * Parameters.driveTrain.dimensions.MODULE_WHEEL_DIA_M / 60);
    
  }

  public void setSteerMParams(PID_PARAMS PID_params, IdleMode idleMode) {

    // PID parameters
    steerMotorPID.setP(PID_params.P);
    steerMotorPID.setI(PID_params.I);
    steerMotorPID.setD(PID_params.D);

    // Ramp rate
    steerMotor.setOpenLoopRampRate(Parameters.driver.CURRENT_PROFILE.DRIVE_RAMP_RATE);

    // Idle mode of the motor
    steerMotor.setIdleMode(idleMode);
  }

  public void setDriveMParams(PID_PARAMS PID_params, IdleMode idleMode) {

    // PID parameters
    driveMotorPID.setP(PID_params.P);
    driveMotorPID.setI(PID_params.I);
    driveMotorPID.setD(PID_params.D);

    // Ramp rate
    driveMotor.setOpenLoopRampRate(Parameters.driver.CURRENT_PROFILE.DRIVE_RAMP_RATE);

    // Idle mode of the motor
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
    return steerCANCoder;
  }

  // Sets the direction of the wheel, in degrees
  public void setDesiredAngle(double targetAngle) {
 
    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = steerMotorPID.calculate(Math.toRadians(getAngle()), Math.toRadians(targetAngle));

    // Calculate the feedforward for the motor
    final double turnFeedforward = steerMotorFF.calculate(steerMotorPID.getSetpoint().velocity);

    // Set the motor to the correct values
    steerMotor.setVoltage(turnOutput + turnFeedforward);

  }

  // Sets the power of the drive motor
  public void setRawPower(double speed) {
    driveMotor.set(speed);
  }

  // Set the speed in m/s
  public void setDesiredSpeed(double speed) {

    // Calculate the output of the drive
    final double driveOutput = driveMotorPID.calculate(driveMotorEncoder.getVelocity(), speed);

    // Calculate the feed forward for the motor
    final double driveFeedforward = driveMotorFF.calculate(speed);

    // Set the motor to the calculated values
    driveMotor.setVoltage(driveOutput + driveFeedforward);
  }

  public void setDesiredState(SwerveModuleState setState) {

    // Optimize the state of the module
    SwerveModuleState optimizedState = SwerveModuleState.optimize(setState, Rotation2d.fromDegrees(getAngle()));

    // Set module to the right angles and speeds
    setDesiredAngle(optimizedState.angle.getDegrees());
    setDesiredSpeed(optimizedState.speedMetersPerSecond);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotorEncoder.getVelocity(), new Rotation2d(Math.toRadians(getAngle())));
  }

  // Gets the position of the encoder (in deg)
  public double getAngle() {
    return steerCANCoder.getAbsolutePosition();
  }

  public double getVelocity() {
    return driveMotorEncoder.getVelocity();
  }

  // Sets the position of the encoder
  public void setEncoderOffset(double correctPosition) {
    steerCANCoder.configMagnetOffset(correctPosition - getAngle());
  }
  
}