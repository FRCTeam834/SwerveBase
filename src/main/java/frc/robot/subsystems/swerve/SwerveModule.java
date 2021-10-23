/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve;

// Parameters
import frc.robot.Parameters;

// Vendor Libs
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax.IdleMode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

// WPI Libraries
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class SwerveModule {

  // Define all of the variables in the global scope
  private CANSparkMax steerMotor;
  private CANSparkMax driveMotor;
  private CANPIDController steerMotorPID;
  private CANPIDController driveMotorPID;
  private CANCoder steerCANCoder;
  private CANEncoder steerMotorEncoder;
  private CANEncoder driveMotorEncoder;
  private double cancoderOffset;
  private double angularOffset;
  private String name;
  private boolean enabled = true;

  // NetworkTable values
  private NetworkTableEntry steerPEntry;
  private NetworkTableEntry steerIEntry;
  private NetworkTableEntry steerDEntry;
  private NetworkTableEntry steerFFEntry;

  private NetworkTableEntry drivePEntry;
  private NetworkTableEntry driveIEntry;
  private NetworkTableEntry driveDEntry;
  private NetworkTableEntry driveFFEntry;

  private NetworkTableEntry velocity;
  private NetworkTableEntry angle;


  // Set up the module and address each of the motor controllers
  public SwerveModule(String moduleName, int steerMID, int driveMID, int CANCoderID, PID_PARAMS steerPIDParams, PID_PARAMS drivePIDParams, boolean reversedDrive) {

    // Set the name
    name = moduleName;

    // CANCoder
    steerCANCoder = new CANCoder(CANCoderID);
    steerCANCoder.setPositionToAbsolute();
    steerCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    steerCANCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

    // Steering motor
    steerMotor = new CANSparkMax(steerMID, CANSparkMax.MotorType.kBrushless);
    steerMotor.enableVoltageCompensation(Parameters.driveTrain.nominalVoltage);
    //steerMotor.setOpenLoopRampRate(Parameters.driver.CURRENT_PROFILE.DRIVE_RAMP_RATE);
    steerMotor.setIdleMode(Parameters.driver.currentProfile.steerIdleMode);
    steerMotor.setInverted(false);

    // Steer motor encoder (position is converted from rotations to degrees)
    steerMotorEncoder = steerMotor.getEncoder();
    steerMotorEncoder.setPositionConversionFactor(360/Parameters.driveTrain.ratios.STEER_GEAR_RATIO);
    steerMotorEncoder.setPosition(getAngle());

    // Steering PID controller (from motor)
    steerMotorPID = steerMotor.getPIDController();
    steerMotorPID.setP(steerPIDParams.P);
    steerMotorPID.setI(steerPIDParams.I);
    steerMotorPID.setD(steerPIDParams.D);
    steerMotorPID.setIZone(steerPIDParams.I_ZONE);
    steerMotorPID.setFF(Parameters.driveTrain.pid.MODULE_S_FF);
    steerMotorPID.setOutputRange(-1, 1);

    // Convert the angular velocity and acceleration to RPM values
    steerMotorPID.setSmartMotionMaxAccel(Parameters.driveTrain.maximums.MAX_ACCEL, 0);
    steerMotorPID.setSmartMotionMaxVelocity(Parameters.driveTrain.maximums.MAX_VELOCITY, 0);
    steerMotorPID.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
    //steerMotorPID.setSmartMotionMinOutputVelocity(Parameters.driveTrain.maximums.MIN_VELOCITY, 0);
    //steerMotorPID.setSmartMotionAllowedClosedLoopError(Parameters.driveTrain.maximums.ALLOWABLE_ERROR, 0);
    //steerMotorPID.enableContinuousInput(-180, 180);

    // Drive motor
    driveMotor = new CANSparkMax(driveMID, CANSparkMax.MotorType.kBrushless);
    driveMotor.enableVoltageCompensation(Parameters.driveTrain.nominalVoltage);
    //driveMotor.setOpenLoopRampRate(Parameters.driver.CURRENT_PROFILE.DRIVE_RAMP_RATE);
    driveMotor.setIdleMode(Parameters.driver.currentProfile.driveIdleMode);

    // Reverse the motor direction if specified
    driveMotor.setInverted(reversedDrive);

    // Drive motor encoder
    driveMotorEncoder = driveMotor.getEncoder();
    driveMotorEncoder.setVelocityConversionFactor((Math.PI * Parameters.driveTrain.dimensions.MODULE_WHEEL_DIA_M) / (60 * Parameters.driveTrain.ratios.DRIVE_GEAR_RATIO));

    // Drive motor PID controller (from motor)
    driveMotorPID = driveMotor.getPIDController();
    driveMotorPID.setP(drivePIDParams.P);
    driveMotorPID.setI(drivePIDParams.I);
    driveMotorPID.setD(drivePIDParams.D);
    driveMotorPID.setIZone(drivePIDParams.I_ZONE);
    driveMotorPID.setFF(Parameters.driveTrain.pid.MODULE_D_FF);
    driveMotorPID.setOutputRange(-1, 1);

    // Set up the module's table on NetworkTables
    NetworkTable swerveTable = NetworkTableInstance.getDefault().getTable("Swerve");
    NetworkTable moduleTable = swerveTable.getSubTable(name + "_MODULE");

    // Put all of the module's current values on NetworkTables
    // Steer PID
    steerPEntry  = moduleTable.getEntry("STEER_P");
    steerIEntry  = moduleTable.getEntry("STEER_I");
    steerDEntry  = moduleTable.getEntry("STEER_D");
    steerFFEntry = moduleTable.getEntry("STEER_FF");

    // Drive PID
    drivePEntry  = moduleTable.getEntry("DRIVE_P");
    driveIEntry  = moduleTable.getEntry("DRIVE_I");
    driveDEntry  = moduleTable.getEntry("DRIVE_D");
    driveFFEntry = moduleTable.getEntry("DRIVE_FF");

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
    steerMotorPID.setFF(pidParams.FF);

    // Ramp rate
    steerMotor.setOpenLoopRampRate(Parameters.driver.currentProfile.driveRampRate);

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
    driveMotorPID.setFF(pidParams.FF);

    // Ramp rate
    //driveMotor.setOpenLoopRampRate(Parameters.driver.CURRENT_PROFILE.DRIVE_RAMP_RATE);

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
  public boolean setDesiredAngle(double targetAngle) {

    // Check to see if the module is enabled
    if (enabled) {

      // Motor angle optimization code (makes sure that the motor doesn't go all the way around)
      while (Math.abs(getAdjustedSteerMotorAngle() - targetAngle) >= 90) {

        // Full rotation optimizations
        if((getAdjustedSteerMotorAngle() - targetAngle) >= 180) {
          angularOffset += 360;
        }
        else if ((getAdjustedSteerMotorAngle() - targetAngle) <= -180) {
          angularOffset -= 360;
        }

        // Half rotation optimizations (full are prioritized first)
        else if ((getAdjustedSteerMotorAngle() - targetAngle) >= 90) {
          angularOffset -= 180;
          driveMotor.setInverted(!driveMotor.getInverted());
        }
        else if ((getAdjustedSteerMotorAngle() - targetAngle) <= -90) {
          angularOffset -= 180;
          driveMotor.setInverted(!driveMotor.getInverted());
        }
      }

      // Calculate the optimal angle for the motor (needs to be corrected as it thinks that the position is 0 at it's startup location)
      double desiredAngle = targetAngle + angularOffset;

      // Set the PID reference
      steerMotorPID.setReference(desiredAngle, ControlType.kSmartMotion);

      // Print out info (for debugging)
      if (Parameters.debug) {
        printDebugString(targetAngle);
      }

      // Return if the module has reached the desired angle
      return (getAngle() < (targetAngle + Parameters.driveTrain.angleTolerance) && (getAngle() > (targetAngle - Parameters.driveTrain.angleTolerance)));
    }
    else {

      // Just return true if the module isn't enabled
      return true;
    }
  }


  // Moves the module to the desired angle
  public void moveToAngle(double angle) {

    // Continuously move the motor at the calculated speeds until it reaches the angle
    while (!setDesiredAngle(angle)) {
      publishPerformanceData();
    }

    // Shut off the motor once done (probably shouldn't be done to ensure position is held)
    //steerMotor.set(0);
  }


  // Sets the power of the drive motor
  public void setRawPower(double speed) {

    // Check to see if the module is enabled
    if (enabled) {
      driveMotor.set(speed);
    }
  }


  // Set the speed in m/s
  public boolean setDesiredVelocity(double speed) {

    // Check to see if the module is enabled
    if (enabled) {

      // Calculate the output of the drive
      driveMotorPID.setReference(speed, ControlType.kVelocity);

      if (Parameters.debug) {
        System.out.println("D_SPD: " + speed + " | A_SPD: " + driveMotorEncoder.getVelocity());
      }

      // Return if the velocity is within tolerance
      return ((getVelocity() < (speed + Parameters.driveTrain.speedTolerance)) && (getVelocity() > (speed - Parameters.driveTrain.speedTolerance)));
    }

    // Return a true, module is disabled
    return true;
  }


  // Sets the speed in m/s (proportional to the error of the angle)
  public boolean setDesiredVelocity(double speed, double desiredAngle) {

    // Check to make sure that the value is within 90 degrees (no movement until within 90)
    if (Math.abs((desiredAngle + angularOffset) - getActualSteerMotorAngle()) <= 90) {

      // Compute the error factor (based on how close the actual angle is to the desired)
      double percentError = 1 - Math.abs(((desiredAngle + angularOffset) - getActualSteerMotorAngle()) / 90);

      // Print the percent error if debugging is enabled
      if (Parameters.debug) {
        System.out.println("% E: " + percentError);
      }

      // Set the adjusted speed
      return setDesiredVelocity(speed * percentError);

    }
    else {
      // We're not within range, therefore the speed should be set to zero
      return setDesiredVelocity(0);
    }
  }


  // Moves the wheel to a desired speed
  public void reachVelocity(double speed) {

    // Continuously move the motor at the calculated speeds until it reaches the angle
    while (!setDesiredVelocity(speed));

    // Shut off the motor once done
    driveMotor.stopMotor();
  }


  // Sets the desired state of the module
  public void setDesiredState(SwerveModuleState setState) {

    // Set module to the right angles and speeds
    setDesiredAngle(setState.angle.getDegrees());
    setDesiredVelocity(setState.speedMetersPerSecond, setState.angle.getDegrees());
  }


  // Gets the state of the module
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotorEncoder.getVelocity(), Rotation2d.fromDegrees(getAngle()));
  }


  // Gets the position of the encoder (in deg)
  public double getAngle() {
    return steerCANCoder.getAbsolutePosition();
  }


  // Gets the adjusted steer motor's angle
  public double getAdjustedSteerMotorAngle() {
    return (getActualSteerMotorAngle() - angularOffset);
  }


  // Gets the actual steer motor's angle
  public double getActualSteerMotorAngle() {
    return steerMotorEncoder.getPosition();
  }


  // Returns the velocity of the module (from the wheel)
  public double getVelocity() {
    return driveMotorEncoder.getVelocity();
  }


  // Sets the position of the encoder
  public void setEncoderOffset(double correctPosition) {

    // Set the cancoder offset variable
    cancoderOffset = correctPosition - (getAngle() - steerCANCoder.configGetMagnetOffset());

    // Set the offset on the encoder
    steerCANCoder.configMagnetOffset(cancoderOffset);

    // Set the encoder's position to zero
    steerMotorEncoder.setPosition(getAngle());
  }


  // Stop both of the motors
  public void stopMotors() {

    // Shut off all of the motors
    steerMotor.stopMotor();
    driveMotor.stopMotor();
  }


  // Command for disable
  public void disable() {

    // Stop the motors
    stopMotors();

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
    Parameters.savedParams.putDouble(name + "_STEER_P", steerMotorPID.getP());
    Parameters.savedParams.putDouble(name + "_STEER_I", steerMotorPID.getI());
    Parameters.savedParams.putDouble(name + "_STEER_D", steerMotorPID.getD());

    // Drive PID
    Parameters.savedParams.putDouble(name + "_DRIVE_P", driveMotorPID.getP());
    Parameters.savedParams.putDouble(name + "_DRIVE_I", driveMotorPID.getI());
    Parameters.savedParams.putDouble(name + "_DRIVE_D", driveMotorPID.getD());

    // Encoder offset
    Parameters.savedParams.putDouble(name + "_ENCODER_OFFSET", cancoderOffset);

    // Steer motor feedforward
    Parameters.savedParams.putDouble(name + "_STEER_FF", steerMotorPID.getFF());

    // Drive motor feedforward
    Parameters.savedParams.putDouble(name + "_DRIVE_FF", driveMotorPID.getFF());
  }


  // Loads all of the parameters from the Rio's saved data
  public void loadParameters() {
    // Loads the saved configurations

    // Steer PID
    steerMotorPID.setP(Parameters.savedParams.getDouble(name + "_STEER_P", steerMotorPID.getP()));
    steerMotorPID.setI(Parameters.savedParams.getDouble(name + "_STEER_I", steerMotorPID.getI()));
    steerMotorPID.setD(Parameters.savedParams.getDouble(name + "_STEER_D", steerMotorPID.getD()));

    // Drive PID
    driveMotorPID.setP(Parameters.savedParams.getDouble(name + "_DRIVE_P", driveMotorPID.getP()));
    driveMotorPID.setI(Parameters.savedParams.getDouble(name + "_DRIVE_I", driveMotorPID.getI()));
    driveMotorPID.setD(Parameters.savedParams.getDouble(name + "_DRIVE_D", driveMotorPID.getD()));

    // Encoder offset
    steerCANCoder.configMagnetOffset(Parameters.savedParams.getDouble(name + "_ENCODER_OFFSET", cancoderOffset));

    // Steer motor feedforward
    steerMotorPID.setFF(Parameters.savedParams.getDouble(name + "_STEER_FF", steerMotorPID.getFF()));

    // Drive motor feedforward
    driveMotorPID.setFF(Parameters.savedParams.getDouble(name + "_DRIVE_FF", driveMotorPID.getFF()));

    // Push the new values to the table
    publishTuningValues();
  }


  // Push the values to NetworkTables
  public void publishTuningValues() {

    // Steer PIDs
    steerPEntry.setDouble(steerMotorPID.getP());
    steerIEntry.setDouble(steerMotorPID.getI());
    steerDEntry.setDouble(steerMotorPID.getD());
    steerFFEntry.setDouble(steerMotorPID.getFF());
    //steerSFFEntry.setDouble(steerMotorFF.ks);
    //steerVFFEntry.setDouble(steerMotorFF.kv);

    // Drive PIDs
    drivePEntry.setDouble(driveMotorPID.getP());
    driveIEntry.setDouble(driveMotorPID.getI());
    driveDEntry.setDouble(driveMotorPID.getD());
    driveFFEntry.setDouble(driveMotorPID.getFF());
    //driveSFFEntry.setDouble(driveMotorFF.ks);
    //driveVFFEntry.setDouble(driveMotorFF.kv);
  }


  // Get the values from NetworkTables
  public void pullTuningValues() {

    // Steer PIDs
    steerMotorPID.setP(steerPEntry.getDouble(steerMotorPID.getP()));
    steerMotorPID.setI(steerIEntry.getDouble(steerMotorPID.getI()));
    steerMotorPID.setD(steerDEntry.getDouble(steerMotorPID.getD()));
    steerMotorPID.setFF(steerFFEntry.getDouble(steerMotorPID.getFF()));

    // Drive PIDs
    driveMotorPID.setP(drivePEntry.getDouble(driveMotorPID.getP()));
    driveMotorPID.setI(driveIEntry.getDouble(driveMotorPID.getI()));
    driveMotorPID.setD(driveDEntry.getDouble(driveMotorPID.getD()));
    driveMotorPID.setFF(driveFFEntry.getDouble(driveMotorPID.getFF()));
  }


  // Pushes the performance data to the NetworkTable
  public void publishPerformanceData() {
    velocity.setDouble(getVelocity());
    angle.setDouble(getAngle());
  }

  // Print out a debug string
  public void printDebugString(double targetAngle) {
    System.out.println(name + ": TAR_A: " + Math.round(targetAngle) + " ACT_A: " + Math.round(getAngle()) + " ADJ_A: " + Math.round(getAdjustedSteerMotorAngle()) + " STR_A: " + Math.round(getActualSteerMotorAngle()) + " OFF_A: " + Math.round(angularOffset));
  }
}