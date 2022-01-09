/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * @author Christian Piper (@CAP1Sup)
 * @since 5/8/20
 */
package frc.robot.subsystems.swerve;

// Imports
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ControlType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

import frc.robot.Parameters;
import frc.robot.utilityClasses.CachedPIDController;
import frc.robot.utilityClasses.PIDParams;

public class SwerveModule {

    // Define all of the variables in the global scope

    // Motors
    private CANSparkMax steerMotor;
    private CANSparkMax driveMotor;
    private CachedPIDController steerMotorPID;
    private CachedPIDController driveMotorPID;
    private CANCoder steerCANCoder;
    private CANEncoder steerMotorEncoder;
    private CANEncoder driveMotorEncoder;

    // General info
    private double cancoderOffset = 0;
    private double angularOffset = 0;
    private double desiredAngle = 0; // in deg
    private double desiredVelocity = 0; // in m/s
    private String name;
    private boolean enabled = true;

    // PID control types
    ControlType steerMControlType;
    ControlType driveMControlType;

    // NetworkTable values
    private NetworkTableEntry steerPEntry;
    private NetworkTableEntry steerIEntry;
    private NetworkTableEntry steerDEntry;
    private NetworkTableEntry steerFFEntry;

    private NetworkTableEntry drivePEntry;
    private NetworkTableEntry driveIEntry;
    private NetworkTableEntry driveDEntry;
    private NetworkTableEntry driveFFEntry;

    private NetworkTableEntry currentVelocity;
    private NetworkTableEntry currentAngle;

    /**
     * Set up the module and address each of the motor controllers
     *
     * @param moduleName The name of the module (used on NetworkTables)
     * @param steerMID The CAN ID of the steer motor
     * @param driveMID The CAN ID of the drive motor
     * @param CANCoderID The CAN ID of the CANCoder angle sensor
     * @param steerPIDParams The PID parameters object for the steer motor
     * @param drivePIDParams The PID parameters object for the drive motor
     * @param reversedDrive If the drive motor should be reversed
     */
    public SwerveModule(
            String moduleName,
            int steerMID,
            int driveMID,
            int CANCoderID,
            PIDParams steerPIDParams,
            PIDParams drivePIDParams,
            boolean reversedDrive) {

        // Set the name
        name = moduleName;

        // CANCoder
        steerCANCoder = new CANCoder(CANCoderID);
        steerCANCoder.setPositionToAbsolute();
        steerCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        steerCANCoder.configSensorInitializationStrategy(
                SensorInitializationStrategy.BootToAbsolutePosition);

        // Steering motor
        steerMotor = new CANSparkMax(steerMID, CANSparkMax.MotorType.kBrushless);
        steerMotor.restoreFactoryDefaults();
        steerMotor.enableVoltageCompensation(Parameters.driveTrain.nominalVoltage);
        steerMotor.setIdleMode(Parameters.driver.currentProfile.steerIdleMode);
        steerMotor.setSmartCurrentLimit(Parameters.driveTrain.maximums.MAX_STEER_CURRENT);
        steerMotor.setInverted(false);
        steerMotor.setSmartCurrentLimit(20);

        // Steer motor encoder (position is converted from rotations to degrees)
        // (For the conversion factor) First we multiply by 360 to convert rotations to degrees,
        // then divide by the steer gear ratio because the motor must move that many times for a
        // full
        // module rotation
        // For the velocity, we can use the same conversion factor and divide by 60 to convert RPM
        // to
        // deg/s
        steerMotorEncoder = steerMotor.getEncoder();
        steerMotorEncoder.setPositionConversionFactor(
                360.0 / Parameters.driveTrain.ratios.STEER_GEAR_RATIO);
        steerMotorEncoder.setVelocityConversionFactor(
                360.0 / (Parameters.driveTrain.ratios.STEER_GEAR_RATIO * 60));
        steerMotorEncoder.setPosition(getAngle());

        // Steering PID controller (from motor)
        // Note that we use a "cached" controller.
        // This version of the PID controller checks if the desired setpoint is already set.
        // This reduces the load on the CAN bus, as we can only send a set amount across at once.
        steerMotorPID = (CachedPIDController)steerMotor.getPIDController();
        steerMotorPID.setP(steerPIDParams.kP);
        steerMotorPID.setI(steerPIDParams.kI);
        steerMotorPID.setD(steerPIDParams.kD);
        steerMotorPID.setIZone(steerPIDParams.iZone);
        steerMotorPID.setOutputRange(-1, 1);

        // Only set feedforward if it's a non-zero
        if (steerPIDParams.kFF != 0) {
            steerMotorPID.setFF(steerPIDParams.kFF);
        }

        // Set the angular velocity and acceleration values (if smart motion is being used)
        if (steerPIDParams.controlType.equals(ControlType.kSmartMotion)) {
            steerMotorPID.setSmartMotionMaxAccel(Parameters.driveTrain.maximums.MAX_ACCEL, 0);
            steerMotorPID.setSmartMotionMaxVelocity(Parameters.driveTrain.maximums.MAX_VELOCITY, 0);
            steerMotorPID.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
        }

        // Save the control type for the steering motor
        steerMControlType = steerPIDParams.controlType;

        // Drive motor
        driveMotor = new CANSparkMax(driveMID, CANSparkMax.MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.enableVoltageCompensation(Parameters.driveTrain.nominalVoltage);
        driveMotor.setSmartCurrentLimit(Parameters.driveTrain.maximums.MAX_DRIVE_CURRENT);
        driveMotor.setIdleMode(Parameters.driver.currentProfile.driveIdleMode);
        driveMotor.setSmartCurrentLimit(30);

        // Reverse the motor direction if specified
        driveMotor.setInverted(reversedDrive);

        // Drive motor encoder
        // First we need to multiply by min/sec (1/60) to get to rotations/s
        // Then we divide by the drive gear ratio, converting motor rotations/s to wheel rotations/s
        // Finally, we multiply by Pi * d, which is the circumference of the wheel, converting it to
        // wheel m/s
        // It's similar with position, we just don't need to divide by 60. Converts rotations to
        // meters
        driveMotorEncoder = driveMotor.getEncoder();
        driveMotorEncoder.setVelocityConversionFactor(
                (Math.PI * Parameters.driveTrain.dimensions.MODULE_WHEEL_DIA_M)
                        / (60.0 * Parameters.driveTrain.ratios.DRIVE_GEAR_RATIO));
        driveMotorEncoder.setPositionConversionFactor(
                (Math.PI * Parameters.driveTrain.dimensions.MODULE_WHEEL_DIA_M)
                        / Parameters.driveTrain.ratios.DRIVE_GEAR_RATIO);

        // Drive motor PID controller (from motor)
        // Note that we use a "cached" controller.
        // This version of the PID controller checks if the desired setpoint is already set.
        // This reduces the load on the CAN bus, as we can only send a set amount across at once.
        driveMotorPID = (CachedPIDController)driveMotor.getPIDController();
        driveMotorPID.setP(drivePIDParams.kP);
        driveMotorPID.setI(drivePIDParams.kI);
        driveMotorPID.setD(drivePIDParams.kD);
        driveMotorPID.setIZone(drivePIDParams.iZone);
        driveMotorPID.setOutputRange(-1, 1);

        // Only set feedforward if it's a non-zero
        if (drivePIDParams.kFF != 0) {
            driveMotorPID.setFF(drivePIDParams.kFF);
        }

        // Save the control type for the drive motor
        driveMControlType = drivePIDParams.controlType;

        // Burn the flash parameters to the Sparks (prevents loss of parameters after brownouts)
        steerMotor.burnFlash();
        driveMotor.burnFlash();

        // Don't mess with NetworkTables unless we have to
        if (Parameters.networkTables) {

            // Set up the module's table on NetworkTables
            NetworkTable swerveTable = NetworkTableInstance.getDefault().getTable("Swerve");
            NetworkTable moduleTable = swerveTable.getSubTable(name + "_MODULE");

            // Put all of the module's current values on NetworkTables
            // Steer PID
            steerPEntry = moduleTable.getEntry("STEER_P");
            steerIEntry = moduleTable.getEntry("STEER_I");
            steerDEntry = moduleTable.getEntry("STEER_D");
            steerFFEntry = moduleTable.getEntry("STEER_FF");

            // Drive PID
            drivePEntry = moduleTable.getEntry("DRIVE_P");
            driveIEntry = moduleTable.getEntry("DRIVE_I");
            driveDEntry = moduleTable.getEntry("DRIVE_D");
            driveFFEntry = moduleTable.getEntry("DRIVE_FF");

            // Performance data
            currentVelocity = moduleTable.getEntry("CURRENT_VELOCITY");
            currentAngle = moduleTable.getEntry("CURRENT_ANGLE");
        }
    }

    /**
     * Sets the steer motor parameters
     *
     * @param pidParams The PID parameters
     * @param idleMode The idle mode of the motor
     */
    public void setSteerMParams(PIDParams pidParams, IdleMode idleMode) {

        // PID parameters
        steerMotorPID.setP(pidParams.kP);
        steerMotorPID.setI(pidParams.kI);
        steerMotorPID.setD(pidParams.kD);
        steerMotorPID.setFF(pidParams.kFF);

        // Idle mode of the motor
        steerMotor.setIdleMode(idleMode);

        // Set the control type
        steerMControlType = pidParams.controlType;

        // Save the parameters (prevents loss of parameters after brownouts)
        steerMotor.burnFlash();
    }

    /**
     * Sets the drive motor parameters
     *
     * @param pidParams The PID parameters
     * @param idleMode The idle mode of the motor
     */
    public void setDriveMParams(PIDParams pidParams, IdleMode idleMode) {

        // PIDF parameters
        driveMotorPID.setP(pidParams.kP);
        driveMotorPID.setI(pidParams.kI);
        driveMotorPID.setD(pidParams.kD);
        driveMotorPID.setFF(pidParams.kFF);

        // Idle mode of the motor
        driveMotor.setIdleMode(idleMode);

        // Set the control type
        driveMControlType = pidParams.controlType;

        // Save the parameters (prevents loss of parameters after brownouts)
        driveMotor.burnFlash();
    }

    /**
     * Gets the steering motor object for the selected module
     *
     * @return The steering motor object
     */
    public CANSparkMax getSteerMotor() {
        return steerMotor;
    }

    /**
     * Gets the drive motor object for the selected module
     *
     * @return The drive motor object
     */
    public CANSparkMax getDriveMotor() {
        return driveMotor;
    }

    /**
     * Gets the CANCoder object for the selected module
     *
     * @return The CANCoder object
     */
    public CANCoder getCANCoder() {
        return steerCANCoder;
    }

    /**
     * Moves the wheel to the target angle, complete with optimizations
     *
     * @param targetAngle The angle to move the module to
     */
    public void setDesiredAngle(double targetAngle) {

        // Check to see if the module is enabled
        if (enabled) {

            // Motor angle optimization code (makes sure that the motor doesn't go all the way
            // around)
            while (Math.abs(getAdjSteerMotorAng() - targetAngle) >= 90) {

                // Calculate the angular deviation
                double angularDev = getAdjSteerMotorAng() - targetAngle;

                // Full rotation optimizations
                if (angularDev >= 180) {
                    angularOffset += 360;
                } else if (angularDev <= -180) {
                    angularOffset -= 360;
                }

                // Half rotation optimizations (full are prioritized first)
                else if (angularDev >= 90) {
                    angularOffset -= 180;
                    driveMotor.setInverted(!driveMotor.getInverted());
                } else if (angularDev <= -90) {
                    angularOffset -= 180;
                    driveMotor.setInverted(!driveMotor.getInverted());
                }
            }

            // Calculate the optimal angle for the motor (needs to be corrected as it thinks that
            // the
            // position is 0 at it's startup location)
            desiredAngle = targetAngle + angularOffset;

            // Set the PID reference
            steerMotorPID.setReference(desiredAngle, steerMControlType);

            // Print out info (for debugging)
            if (Parameters.debug) {
                printDebugString(targetAngle);
            }
        }
    }

    /**
     * Checks if the module is at it's desired angle
     *
     * @return Has the module reached it's desired angle?
     */
    public boolean isAtDesiredAngle() {

        // We need to check if the module is supposed to be enabled or not
        if (enabled) {

            // Get the current angle of the module
            double currentAngle = getAngle();

            // Return if the module has reached the desired angle
            return (currentAngle < (desiredAngle + Parameters.driveTrain.angleTolerance)
                    && (currentAngle > (desiredAngle - Parameters.driveTrain.angleTolerance)));
        } else {

            // Just return true if the module isn't enabled
            return true;
        }
    }

    // Sets the power of the drive motor
    public void setRawDrivePower(double percentage) {

        // Check to see if the module is enabled
        if (enabled) {
            driveMotor.set(percentage);
        }
    }

    // Set the desired velocity in m/s
    public void setDesiredVelocity(double targetVelocity) {

        // Check to see if the module is enabled
        if (enabled) {

            // Calculate the output of the drive
            driveMotorPID.setReference(targetVelocity, driveMControlType);

            // Print out debug info if needed
            if (Parameters.debug) {
                System.out.println("D_SPD: " + targetVelocity + " | A_SPD: " + getVelocity());
            }

            // Save the desired velocity
            desiredVelocity = targetVelocity;
        }
    }

    // Sets the desired velocity in m/s (proportional to the error of the angle)
    public boolean setDesiredVelocity(double targetVelocity, double targetAngle) {

        // Check to make sure that the value is within 90 degrees (no movement until within 90)
        if (Math.abs((targetAngle + angularOffset) - getActualSteerMotorAngle()) <= 90) {

            // Compute the error factor (based on how close the actual angle is to the desired)
            double percentError =
                    1 - Math.abs(((targetAngle + angularOffset) - getActualSteerMotorAngle()) / 90);

            // Print the percent error if debugging is enabled
            if (Parameters.debug) {
                System.out.println("% E: " + percentError);
            }

            // Set the adjusted velocity
            setDesiredVelocity(targetVelocity * percentError);
        }

        // Return if we have reached our desired velocity (should always return correctly,
        // regardless of
        // enable state)
        return isAtDesiredVelocity();
    }

    // Checks if a module's velocity is within tolerance
    public boolean isAtDesiredVelocity() {

        // Check to see if the module is enabled
        if (enabled) {

            // Get the current velocity of the drive motor
            double currentVelocity = getVelocity();

            // Return if the velocity is within tolerance
            return ((currentVelocity < (desiredVelocity + Parameters.driveTrain.velocityTolerance))
                    && (currentVelocity
                            > (desiredVelocity - Parameters.driveTrain.velocityTolerance)));
        }

        // Return a true, module is disabled
        return true;
    }

    // Sets the desired state of the module
    public void setDesiredState(SwerveModuleState setState) {

        // Set module to the right angles and velocities
        setDesiredAngle(setState.angle.getDegrees());
        setDesiredVelocity(setState.speedMetersPerSecond, setState.angle.getDegrees());
    }

    // Gets the state of the module
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), Rotation2d.fromDegrees(getAngle()));
    }

    // Gets the position of the encoder (in deg)
    public double getAngle() {
        return steerCANCoder.getAbsolutePosition();
    }

    // Gets the adjusted steer motor's angle
    public double getAdjSteerMotorAng() {
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
        // The getAngle reference should be changed now, so we need to re-request it
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

        // Disable the motors
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
        Parameters.savedParams.putDouble(name + "_STEER_FF", steerMotorPID.getFF());

        // Drive PID
        Parameters.savedParams.putDouble(name + "_DRIVE_P", driveMotorPID.getP());
        Parameters.savedParams.putDouble(name + "_DRIVE_I", driveMotorPID.getI());
        Parameters.savedParams.putDouble(name + "_DRIVE_D", driveMotorPID.getD());
        Parameters.savedParams.putDouble(name + "_DRIVE_FF", driveMotorPID.getFF());

        // Encoder offset
        Parameters.savedParams.putDouble(name + "_ENCODER_OFFSET", cancoderOffset);
    }

    // Loads all of the parameters from the Rio's saved data
    public void loadParameters() {

        // Steer PID
        steerMotorPID.setP(
                Parameters.savedParams.getDouble(name + "_STEER_P", steerMotorPID.getP()));
        steerMotorPID.setI(
                Parameters.savedParams.getDouble(name + "_STEER_I", steerMotorPID.getI()));
        steerMotorPID.setD(
                Parameters.savedParams.getDouble(name + "_STEER_D", steerMotorPID.getD()));
        steerMotorPID.setFF(
                Parameters.savedParams.getDouble(name + "_STEER_FF", steerMotorPID.getFF()));

        // Drive PID
        driveMotorPID.setP(
                Parameters.savedParams.getDouble(name + "_DRIVE_P", driveMotorPID.getP()));
        driveMotorPID.setI(
                Parameters.savedParams.getDouble(name + "_DRIVE_I", driveMotorPID.getI()));
        driveMotorPID.setD(
                Parameters.savedParams.getDouble(name + "_DRIVE_D", driveMotorPID.getD()));
        driveMotorPID.setFF(
                Parameters.savedParams.getDouble(name + "_DRIVE_FF", driveMotorPID.getFF()));

        // Encoder offset
        steerCANCoder.configMagnetOffset(
                Parameters.savedParams.getDouble(name + "_ENCODER_OFFSET", cancoderOffset));
        steerMotorEncoder.setPosition(getAngle());

        // Push the new values to the table
        publishTuningValues();
    }

    // Push the values to NetworkTables
    public void publishTuningValues() {

        // Don't mess with NetworkTables unless we have to
        if (Parameters.networkTables) {

            // Steer PIDs
            steerPEntry.setDouble(steerMotorPID.getP());
            steerIEntry.setDouble(steerMotorPID.getI());
            steerDEntry.setDouble(steerMotorPID.getD());
            steerFFEntry.setDouble(steerMotorPID.getFF());

            // Drive PIDs
            drivePEntry.setDouble(driveMotorPID.getP());
            driveIEntry.setDouble(driveMotorPID.getI());
            driveDEntry.setDouble(driveMotorPID.getD());
            driveFFEntry.setDouble(driveMotorPID.getFF());
        }
    }

    // Get the values from NetworkTables
    public void pullTuningValues() {

        // Don't mess with NetworkTables unless we have to
        if (Parameters.networkTables) {

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
    }

    // Pushes the performance data to the NetworkTable
    public void publishPerformanceData() {

        // Don't mess with NetworkTables unless we have to
        if (Parameters.networkTables) {
            currentVelocity.setDouble(getVelocity());
            currentAngle.setDouble(getAngle());
        }
    }

    // Print out a debug string
    public void printDebugString(double targetAngle) {
        System.out.println(
                name
                        + ": TAR_A: "
                        + Math.round(targetAngle)
                        + " ACT_A: "
                        + Math.round(getAngle())
                        + " ADJ_A: "
                        + Math.round(getAdjSteerMotorAng())
                        + " STR_A: "
                        + Math.round(getActualSteerMotorAngle())
                        + " OFF_A: "
                        + Math.round(angularOffset));
    }
}
