/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * @author Christian Piper (@CAP1Sup), Mohammed Durrani (@mdurrani808), Krishna Dihora (@kjdih2)
 * @since 5/8/20
 */

package frc.robot;

// Internal libraries
import frc.robot.DriverProfiles.DriverProfile;
import frc.robot.DriverProfiles.JoystickParams;
import frc.robot.enums.JOYSTICK_OUTPUT_TYPES;
import frc.robot.subsystems.swerve.PID_PARAMS;

// Vendor libraries
import com.revrobotics.CANSparkMax.IdleMode;

// WPI Libraries
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.numbers.*;
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Preferences;

/**
 * The Parameters class provides a convenient place for teams to hold robot-wide
 * numerical or boolean variables. This class should not be used for any other
 * purpose. All parameters should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the parameters are needed, to reduce verbosity.
 */
public final class Parameters {

    // Enables all debug statements
    public static final boolean debug = false;

    // All of the fun parameters
    public static final class funParameters {

        public static final int SSN = 123352357; // Social Security Number [DO NOT LOSE]
        public static final double USD_TO_GHS = 5.748; // US Dollar to Gana Cedi conversion rate
        public static final double MM_TO_IK = 2.15; // Mozambican metical to Icelandic Krona conversion rate
        public static final double MIN_IN_HR = 60; // Minutes in an hour
        public static final int BUILD_TEAM_BRAIN_CELLS = 1; // Brain cells owned by the build team
        public static final int CODING_TEAM_BRAIN_CELLS = 5; // Same as the amount of coding team members
        public static final int SHRIMP_ON_THE_BARBIE = 3; // Number of shrimp on the barbecue
        public static final int ANDREWS_PROGRESS_WHEN_AROUND_SAFETY_TEAM = -10; // What happens when Andrew is around the safety team... backwards progress
        public static final int CHRISTIAN_FORTNITE_WINS = 38; // The number of the lead programmer's Fortnite wins
    }

    // All of the driver parameters
    public static final class driver {

        /**
         * A quick way of referencing driver parameters
         *
         * @param NAME                 The name of the driver
         * @param JOYSTICK_PARAMS      The joystick parameters to use
         * @param MAX_STEER_SPEED      Maximum deg/s of rotational speed
         * @param DRIVE_RAMP_RATE      The speed at which the drivetrain ramps to full speed. Prevents sudden jerks. Maybe something to override?
         * @param lockemUp            If the swerve should lock the modules at 45 degrees, effectively hitting the brakes. Hard on the modules, but worth it in competition
         * @param fieldCentric        If the robot should treat itself as forward or if the field's forward should be forward
         * @param maxModSpeed            Maximum speed of modules in m/s
         * @param driveIdleMode      If the drive motors should coast or brake after they exceed the current set speed. Coasting makes the driving smoother, but braking makes it more aggressive
         * @param steerIdleMode      If the steering motor should coast of brake after they exceed the current set speed. Modules will most likely only work with braking enabled
        */
        public static DriverProfile[] driverProfiles = {
            new DriverProfile("CAP1Sup", new JoystickParams(0.075, JOYSTICK_OUTPUT_TYPES.ZEROED_QUAD_LINEAR, 2), 180.0, 0.5, true, false, 8.0, IdleMode.kBrake, IdleMode.kBrake),
            new DriverProfile("Test", new JoystickParams(0.1, JOYSTICK_OUTPUT_TYPES.ZEROED_LINEAR), 45.0, 0.5, true, false, 1.0, IdleMode.kBrake, IdleMode.kBrake)
        };

        // Default profile (must be kept!)
        public static final DriverProfile defaultDriverProfile = driverProfiles[0]; //new DriverProfile("Default", 0.15, 45.0, 0.5, true, true, 1.0, IdleMode.kBrake, IdleMode.kBrake);

        // Current Driver Profile being used
        public static DriverProfile currentProfile = driverProfiles[0];

    }

    // the saved preferences for the current driver
    public static Preferences savedParams = Preferences.getInstance();

    // All of the drivetrain parameters
    public static final class driveTrain {

        // Tolerances for completing movements
        public static final double angleTolerance = 2; // deg
        public static final double speedTolerance = 0.01; // m/s

        // Nominal voltage
        public static final double nominalVoltage = 12;

        // All of the CAN IDs
        public static final class can {

            // CAN parameters
            public static final int FL_STEER_ID = 1;
            public static final int FR_STEER_ID = 2;
            public static final int BL_STEER_ID = 3;
            public static final int BR_STEER_ID = 4;

            public static final int FL_DRIVE_ID = 5;
            public static final int FR_DRIVE_ID = 6;
            public static final int BL_DRIVE_ID = 7;
            public static final int BR_DRIVE_ID = 8;

            public static final int FL_CODER_ID = 9;
            public static final int FR_CODER_ID = 10;
            public static final int BL_CODER_ID = 11;
            public static final int BR_CODER_ID = 12;
        }


        // All of the chassis dimensions
        public static final class dimensions {

            // Swerve calculation parameters (in meters)
            public static final double DRIVE_LENGTH = Units.inchesToMeters(22.4);
            public static final double DRIVE_WIDTH = Units.inchesToMeters(22.4);
            public static final double MODULE_WHEEL_DIA_IN = 4; // Inches
            public static final double MODULE_WHEEL_DIA_M = Units.inchesToMeters(MODULE_WHEEL_DIA_IN); // Meters (for odometry calculations)
        }


        // All of the maximums
        public static final class maximums {
            public static final double MAX_MODULE_SPEED = 8; // (m/s)
            public static final double MAX_VELOCITY = 10000; // (RPM)
            public static final double MAX_ACCEL = 500000000; // (RPMM)
        }


        // All of the PID parameters
        public static final class pid {
            /**
             * PID parameters
             * Gains used in each module's steering motor, to be adjusted accordingly
             * Gains(kp, ki, kd, feedforward, iZone, peak output);
             */
            public static PID_PARAMS FL_STEER_PID = new PID_PARAMS(0.00000009, 0.0, 0.000, driveTrain.pid.MODULE_S_FF, 0, driver.currentProfile.maxModSpeed);
            public static PID_PARAMS FR_STEER_PID = new PID_PARAMS(0.00000009, 0.0, 0.000, driveTrain.pid.MODULE_S_FF, 0, driver.currentProfile.maxModSpeed);
            public static PID_PARAMS BL_STEER_PID = new PID_PARAMS(0.00000009, 0.0, 0.000, driveTrain.pid.MODULE_S_FF, 0, driver.currentProfile.maxModSpeed);
            public static PID_PARAMS BR_STEER_PID = new PID_PARAMS(0.00000009, 0.0, 0.000, driveTrain.pid.MODULE_S_FF, 0, driver.currentProfile.maxModSpeed);

            public static PID_PARAMS FL_DRIVE_PID = new PID_PARAMS(0.500, 0.0, 0.00, driveTrain.pid.MODULE_D_FF, 0, driver.currentProfile.maxModSpeed);
            public static PID_PARAMS FR_DRIVE_PID = new PID_PARAMS(0.500, 0.0, 0.00, driveTrain.pid.MODULE_D_FF, 0, driver.currentProfile.maxModSpeed);
            public static PID_PARAMS BL_DRIVE_PID = new PID_PARAMS(0.500, 0.0, 0.00, driveTrain.pid.MODULE_D_FF, 0, driver.currentProfile.maxModSpeed);
            public static PID_PARAMS BR_DRIVE_PID = new PID_PARAMS(0.500, 0.0, 0.00, driveTrain.pid.MODULE_D_FF, 0, driver.currentProfile.maxModSpeed);

            public static final double MODULE_S_FF = 0.000000; // Must be tuned for the modules!
            public static final double MODULE_D_FF = 0.000000; // Maybe: 0.000156;
        }


        // All of the movement control parameters
        public static final class movement {

            // Timeout for all movements (break if position not reached in time)
            public static final double TIMEOUT = 10; // seconds

            // Pose estimator parameters (units are m, m, radians)
            public static final Matrix<N3,N1> POSE_STD_DEV = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, Math.toRadians(0.125));
            public static final Matrix<N1,N1> ENCODER_GYRO_DEV = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(Math.toRadians(0.125));
            public static final Matrix<N3,N1> VISION_DEVIATION = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, Math.toRadians(0.125));

            // PID controller (rotation constraints are max velocity and max acceleration)
            public static PIDController movementPID = new PIDController(1, 0, 0);
            public static Constraints rotationConstraints = new Constraints(Math.toRadians(360), Math.toRadians(180));
            public static ProfiledPIDController rotationPID = new ProfiledPIDController(1, 0, 0, rotationConstraints);
        }

        // The gear ratios of the module
        public static final class ratios {

            // For converting CANCoder data to steer motor data
            public static final double STEER_GEAR_RATIO = 12.8;
            public static final double DRIVE_GEAR_RATIO = 8.16;
        }
    }


    // DriverStation instance
    public static DriverStation driverStation = DriverStation.getInstance();


    // All of the starting position data
    public static final class positions {

        // All of the possible starting positions (and their angles)
        public static final Pose2d[] POSSIBLE_STARTING_POSITIONS = {new Pose2d(0, 0, Rotation2d.fromDegrees(0)), new Pose2d(0, 0, Rotation2d.fromDegrees(0)), new Pose2d(0, 0, Rotation2d.fromDegrees(0))};

        // Actual starting position (declared in the global scope)
        public static final Pose2d STARTING_POS = Parameters.positions.POSSIBLE_STARTING_POSITIONS[Parameters.driverStation.getLocation() - 1];
    }


    // All of the joystick variables
    public static final class joysticks {

        // Dynamically allocated Joysticks
        // Joysticks have 11 buttons
        public static final int JOYSTICK_BUTTON_COUNT = 11;
    }


    // Vision parameters - used for distance calculations
    public static final class camera {

        // Camera-specific parameters (pixels)
        public static final double CAMERA_FOCAL_LENGTH = 333.82;

        // Game-specific parameters (inches)
        public static final double GOAL_HEIGHT = 98.25;
        public static final double POWER_CELL_HEIGHT = 7;
    }



}
