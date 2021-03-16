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
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

// WPI Libraries
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;


public class SwerveModule {

  // Define all of the variables in the global scope
  private CANSparkMax steerMotor;
  private CANSparkMax driveMotor;
  private ProfiledPIDController steerMotorPID;
  //private CANPIDController steerMotorPID;
  private PIDController driveMotorPID;
  //private SimpleMotorFeedforward driveMotorFF;
  //private SimpleMotorFeedforward steerMotorFF;
  private CANCoder steerCANCoder;
  private CANEncoder driveMotorEncoder;
  private double cancoderOffset;
  private String name;
  private boolean enabled = true;

  // NetworkTable values
  private NetworkTableEntry steerPEntry;
  private NetworkTableEntry steerIEntry;
  private NetworkTableEntry steerDEntry;
  //private NetworkTableEntry steerSFFEntry;
  //private NetworkTableEntry steerVFFEntry;

  private NetworkTableEntry drivePEntry;
  private NetworkTableEntry driveIEntry;
  private NetworkTableEntry driveDEntry;
  //private NetworkTableEntry driveSFFEntry;
  //private NetworkTableEntry driveVFFEntry;

  private NetworkTableEntry velocity;
  private NetworkTableEntry angle;


  // Set up the module and address each of the motor controllers
  public SwerveModule(String moduleName, int steerMID, int driveMID, int CANCoderID, PID_PARAMS steerPIDParams, PID_PARAMS drivePIDParams) {

    // Set the name
    name = moduleName;

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
    Constraints constraints = new Constraints(Parameters.driveTrain.maximums.MAX_MODULE_ANGULAR_VELOCITY, Parameters.driveTrain.maximums.MAX_MODULE_ANGULAR_ACCEL);
    steerMotorPID = new ProfiledPIDController(steerPIDParams.P, steerPIDParams.I, steerPIDParams.D, constraints);
    // Steering PID controller (from motor)
    //steerMotorPID = steerMotor.getPIDController();
    steerMotorPID.setP(steerPIDParams.P);
    steerMotorPID.setI(steerPIDParams.I);
    steerMotorPID.setD(steerPIDParams.D);

    // Convert the angular velocity and acceleration to RPM values
    //steerMotorPID.setSmartMotionMaxAccel((Parameters.driveTrain.maximums.MAX_MODULE_ANGULAR_ACCEL / 360), 0);
    //steerMotorPID.setSmartMotionMaxVelocity((Parameters.driveTrain.maximums.MAX_MODULE_ANGULAR_VELOCITY / 360), 0);
    steerMotorPID.enableContinuousInput(-180, 180);
    //steerMotorPID.setIntegratorRange(-steerPIDParams.I_ZONE, steerPIDParams.I_ZONE);

    // Steering motor feed forward
    //steerMotorFF = new SimpleMotorFeedforward(steerPIDParams.SFF, steerPIDParams.VFF);

    // Drive motor
    driveMotor = new CANSparkMax(driveMID, CANSparkMax.MotorType.kBrushless);
    driveMotor.setOpenLoopRampRate(Parameters.driver.CURRENT_PROFILE.DRIVE_RAMP_RATE);
    driveMotor.setIdleMode(Parameters.driver.CURRENT_PROFILE.DRIVE_IDLE_MODE);

    // Drive motor PID controller (from motor)
    driveMotorPID = new PIDController(drivePIDParams.P, drivePIDParams.I, drivePIDParams.D);//driveMotor.getPIDController();
    //driveMotorPID.setP(drivePIDParams.P);
    //driveMotorPID.setI(drivePIDParams.I);
    //driveMotorPID.setD(drivePIDParams.D);

    // Drive motor feed forward
    //driveMotorFF = new SimpleMotorFeedforward(drivePIDParams.SFF, drivePIDParams.VFF);

    // Drive motor encoder
    driveMotorEncoder = driveMotor.getEncoder(EncoderType.kHallSensor, 42);
    driveMotorEncoder.setVelocityConversionFactor(Math.PI * Parameters.driveTrain.dimensions.MODULE_WHEEL_DIA_M / 60);

    // Set up the module's table on NetworkTables
    NetworkTable swerveTable = NetworkTableInstance.getDefault().getTable("Swerve");
    NetworkTable moduleTable = swerveTable.getSubTable(name + "_MODULE");

    // Put all of the module's current values on NetworkTables
    // Steer PID
    steerPEntry = moduleTable.getEntry("STEER_P");
    steerIEntry = moduleTable.getEntry("STEER_I");
    steerDEntry = moduleTable.getEntry("STEER_D");
    //steerSFFEntry = moduleTable.getEntry("STEER_SFF");
    //steerVFFEntry = moduleTable.getEntry("STEER_VFF");

    // Drive PID
    drivePEntry = moduleTable.getEntry("DRIVE_P");
    driveIEntry = moduleTable.getEntry("DRIVE_I");
    driveDEntry = moduleTable.getEntry("DRIVE_D");
    //driveSFFEntry = moduleTable.getEntry("DRIVE_SFF");
    //driveVFFEntry = moduleTable.getEntry("DRIVE_VFF");

    // Performance data
    velocity = moduleTable.getEntry("CURRENT_VELOCITY");
    angle = moduleTable.getEntry("CURRENT_ANGLE");
  }


  // Sets the steer motor parameters
  public void setSteerMParams(PID_PARAMS pidParams, IdleMode idleMode) {

    // PID parameters
    steerMotorPID.setP(pidParams.P);
    steerMotorPID.setI(pidParams.I);
    steerMotorPID.setD(pidParams.D);

    // Feedforward
    //steerMotorFF = new SimpleMotorFeedforward(pidParams.SFF, pidParams.VFF);

    // Ramp rate
    steerMotor.setOpenLoopRampRate(Parameters.driver.CURRENT_PROFILE.DRIVE_RAMP_RATE);

    // Idle mode of the motor
    steerMotor.setIdleMode(idleMode);
  }


  // Sets the drive motor parameters
  public void setDriveMParams(PID_PARAMS pidParams, IdleMode idleMode) {

    // PID parameters
    driveMotorPID.setP(pidParams.P);
    driveMotorPID.setI(pidParams.I);
    driveMotorPID.setD(pidParams.D);

    // Feedforward
    //driveMotorFF = new SimpleMotorFeedforward(pidParams.SFF, pidParams.VFF);

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

    // Check to see if the module is enabled
    if (enabled) {

      // Calculate the turning motor output from the turning PID controller.
      final double steerOutput = steerMotorPID.calculate(getAngle(), targetAngle);
      //steerMotorPID.setReference(targetAngle, ControlType.kPosition);

      // Calculate the feedforward for the motor
      //final double steerFeedforward = steerMotorFF.calculate(steerMotorPID.getSetpoint().velocity);

      // Set the motor to the correct values
      steerMotor.setVoltage(steerOutput /* + steerFeedforward */);
    }

    System.out.println(name + ": " + targetAngle + " : " + getAngle());
  }


  // Moves the module to the desired angle
  public void moveToAngle(double angle) {

    int maxCount = 10000;
    int counter = 0;

    // Continuously move the motor at the calculated speeds until it reaches the angle
    while (!(getAngle() < (angle + Parameters.driveTrain.angleTolerance) && (getAngle() > (angle - Parameters.driveTrain.angleTolerance)))) {
      if (counter < maxCount) {
        counter++;
      }
      else {
        break;
      }
      setDesiredAngle(angle);
      publishPerformanceData();
    }

    // Shut off the motor once done
    steerMotor.set(0);
  }


  // Sets the power of the drive motor
  public void setRawPower(double speed) {

    // Check to see if the module is enabled
    if (enabled) {
      driveMotor.set(speed);
    }

  }


  // Set the speed in m/s
  public void setDesiredVelocity(double speed) {

    // Check to see if the module is enabled
    if (enabled) {

      // Calculate the output of the drive
      final double driveOutput = driveMotorPID.calculate(driveMotorEncoder.getVelocity(), speed);
      //driveMotorPID.setReference(speed, ControlType.kVelocity);

      // Calculate the feed forward for the motor
      //final double driveFeedforward = driveMotorFF.calculate(speed);

      // Set the motor to the calculated values
      driveMotor.setVoltage(driveOutput /* + driveFeedforward */);
    }
  }


  // Moves the wheel to a desired speed 
  public void reachVelocity(double speed) {

    // Continuously move the motor at the calculated speeds until it reaches the angle
    while (!(getVelocity() < (speed + Parameters.driveTrain.speedTolerance) && (getVelocity() > (speed - Parameters.driveTrain.speedTolerance)))) {
      setDesiredVelocity(speed);
    }

    // Shut off the motor once done
    driveMotor.set(0);
  }


  // Sets the desired state of the module
  public void setDesiredState(SwerveModuleState setState) {

    // Optimize the state of the module
    SwerveModuleState optimizedState = SwerveModuleState.optimize(setState, Rotation2d.fromDegrees(getAngle()));

    // Set module to the right angles and speeds
    setDesiredAngle(optimizedState.angle.getDegrees());
    setDesiredVelocity(optimizedState.speedMetersPerSecond);
  }


  // Gets the state of the module
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotorEncoder.getVelocity(), Rotation2d.fromDegrees(getAngle()));
  }


  // Gets the position of the encoder (in deg)
  public double getAngle() {
    return steerCANCoder.getAbsolutePosition();
  }


  // Returns the velocity of the module (from the wheel)
  public double getVelocity() {
    return driveMotorEncoder.getVelocity();
  }


  // Sets the position of the encoder
  public void setEncoderOffset(double correctPosition) {

    // Set the cancoder offset variable
    cancoderOffset = correctPosition - getAngle();

    // Set the offset on the encoder
    steerCANCoder.configMagnetOffset(cancoderOffset);
  }


  // Command for disable
  public void disable() {

    // Shut off all of the motors
    steerMotor.set(0);
    driveMotor.set(0);

    enabled = false;
  }


  // Command for enable
  public void enable() {
    enabled = true;
  }


  // Saves all parameters relevant to the swerve module
  public void saveParameters() {
    // Saves the configured configuration in the present tense

    // Steer PID
    Parameters.SAVED_PARAMS.putDouble(name + "_STEER_P", steerMotorPID.getP());
    Parameters.SAVED_PARAMS.putDouble(name + "_STEER_I", steerMotorPID.getI());
    Parameters.SAVED_PARAMS.putDouble(name + "_STEER_D", steerMotorPID.getD());

    // Drive PID
    Parameters.SAVED_PARAMS.putDouble(name + "_DRIVE_P", driveMotorPID.getP());
    Parameters.SAVED_PARAMS.putDouble(name + "_DRIVE_I", driveMotorPID.getI());
    Parameters.SAVED_PARAMS.putDouble(name + "_DRIVE_D", driveMotorPID.getD());

    // Encoder offset
    Parameters.SAVED_PARAMS.putDouble(name + "_ENCODER_OFFSET", cancoderOffset);

    // Steer motor feedforward
    //Parameters.SAVED_PARAMS.putDouble(name + "_STEER_SFF", steerMotorFF.ks);
    //Parameters.SAVED_PARAMS.putDouble(name + "_STEER_VFF", steerMotorFF.kv);

    // Drive motor feedforward
    //Parameters.SAVED_PARAMS.putDouble(name + "_DRIVE_SFF", driveMotorFF.ks);
    //Parameters.SAVED_PARAMS.putDouble(name + "_DRIVE_VFF", driveMotorFF.kv);

  }


  // Loads all of the parameters from the Rio's saved data
  public void loadParameters() {
    // Loads the saved configurations

    // Steer PID
    steerMotorPID.setP(Parameters.SAVED_PARAMS.getDouble(name + "_STEER_P", steerMotorPID.getP()));
    steerMotorPID.setI(Parameters.SAVED_PARAMS.getDouble(name + "_STEER_I", steerMotorPID.getI()));
    steerMotorPID.setD(Parameters.SAVED_PARAMS.getDouble(name + "_STEER_D", steerMotorPID.getD()));

    // Drive PID
    driveMotorPID.setP(Parameters.SAVED_PARAMS.getDouble(name + "_DRIVE_P", driveMotorPID.getP()));
    driveMotorPID.setI(Parameters.SAVED_PARAMS.getDouble(name + "_DRIVE_I", driveMotorPID.getI()));
    driveMotorPID.setD(Parameters.SAVED_PARAMS.getDouble(name + "_DRIVE_D", driveMotorPID.getD()));

    // Encoder offset
    steerCANCoder.configMagnetOffset(Parameters.SAVED_PARAMS.getDouble(name + "_ENCODER_OFFSET", cancoderOffset));

    // Steer motor feedforward
    //double newSteerKS = Parameters.SAVED_PARAMS.getDouble(name + "_STEER_SFF", steerMotorFF.ks);
    //double newSteerKV = Parameters.SAVED_PARAMS.getDouble(name + "_STEER_VFF", steerMotorFF.kv);
    //steerMotorFF = new SimpleMotorFeedforward(newSteerKS, newSteerKV);

    // Drive motor feedforward
    //double newDriveKS = Parameters.SAVED_PARAMS.getDouble(name + "_DRIVE_SFF", driveMotorFF.ks);
    //double newDriveKV = Parameters.SAVED_PARAMS.getDouble(name + "_DRIVE_VFF", driveMotorFF.kv);
    //driveMotorFF = new SimpleMotorFeedforward(newDriveKS, newDriveKV);

    // Push the new values to the table
    publishTuningValues();
  }


  // Push the values to NetworkTables
  public void publishTuningValues() {

    // Steer PIDs
    steerPEntry.setDouble(steerMotorPID.getP());
    steerIEntry.setDouble(steerMotorPID.getI());
    steerDEntry.setDouble(steerMotorPID.getD());
    //steerSFFEntry.setDouble(steerMotorFF.ks);
    //steerVFFEntry.setDouble(steerMotorFF.kv);

    // Drive PIDs
    drivePEntry.setDouble(driveMotorPID.getP());
    driveIEntry.setDouble(driveMotorPID.getI());
    driveDEntry.setDouble(driveMotorPID.getD());
    //driveSFFEntry.setDouble(driveMotorFF.ks);
    //driveVFFEntry.setDouble(driveMotorFF.kv);
  }


  // Get the values from NetworkTables
  public void pullTuningValues() {

    // Steer PIDs
    steerMotorPID.setP(steerPEntry.getDouble(steerMotorPID.getP()));
    steerMotorPID.setI(steerIEntry.getDouble(steerMotorPID.getI()));
    steerMotorPID.setD(steerDEntry.getDouble(steerMotorPID.getD()));
    //steerMotorFF = new SimpleMotorFeedforward(steerSFFEntry.getDouble(steerMotorFF.ks), steerVFFEntry.getDouble(steerMotorFF.kv));

    // Drive PIDs
    driveMotorPID.setP(drivePEntry.getDouble(driveMotorPID.getP()));
    driveMotorPID.setI(driveIEntry.getDouble(driveMotorPID.getI()));
    driveMotorPID.setD(driveDEntry.getDouble(driveMotorPID.getD()));
    //driveMotorFF = new SimpleMotorFeedforward(driveSFFEntry.getDouble(driveMotorFF.ks), driveVFFEntry.getDouble(driveMotorFF.kv));
  }


  // Pushes the performance data to the NetworkTable
  public void publishPerformanceData() {
    velocity.setDouble(getVelocity());
    angle.setDouble(getAngle());
  }
}