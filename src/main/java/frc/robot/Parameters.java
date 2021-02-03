/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// Internal libraries
import frc.robot.DriverProfiles.DriverProfile;
import frc.robot.swerve.PID_PARAMETERS;

// Vendor libraries
import com.revrobotics.CANSparkMax.IdleMode;

// WPI Libraries
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.numbers.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;


/**
 * The Parameters class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Parameters {

    public static final int SSN = 123352357; //Social Security Number [DO NOT LOSE]
    public static final double USD_TO_GHS = 5.748; //US Dollar to Gana Cedi conversion rate
    public static final double MM_TO_IK = 2.15; //Mozambican metical to Icelandic Krona conversion rate
    public static final double MIN_IN_HR = 60; //Minutes in an hour
    public static final int BUILD_TEAM_BRAIN_CELLS = 1; //Brain cells owned by the build team
    public static final int CODING_TEAM_BRAIN_CELLS = 5; //Same as the amount of coding team members
    public static final int SHRIMP_ON_THE_BARBIE = 3; //Number of shrimp on the barbecue
    public static final int ANDREWS_PROGRESS_WHEN_AROUND_SAFETY_TEAM = -10; //What happens when Andrew is around the safety team... backwards progress

    // Driver Profiles
    public static DriverProfile[] DRIVER_PROFILES = {
        // DriverProfile NAME, double JOYSTICK_DEADZONE, double MAX_TURN_SPEED (deg/s), double DRIVE_RAMP_RATE, boolean LOCKEM_UP, boolean FIELD_CENTRIC, double MAX_SPEED (m/s), IdleMode DRIVE_IDLE_MODE) 
        new DriverProfile("CAP1Sup",         0.05, 45.0, 0.5, true, true, 1.0, IdleMode.kBrake, IdleMode.kBrake),
        new DriverProfile("Christian Velez", 0.15, 45.0, 0.5, true, true, 1.0, IdleMode.kBrake, IdleMode.kBrake),
        new DriverProfile("Test",            0.15, 45.0, 0.5, true, true, 1.0, IdleMode.kBrake, IdleMode.kBrake)
    };

    // Default profile (must be kept!)
    public static DriverProfile DEFAULT_DRIVER_PROFILE = new DriverProfile("Default", 0.15, 1.0, 0.5, true, true, 1.0, IdleMode.kBrake, IdleMode.kBrake);


    // Current Driver Profile being used
    public static DriverProfile CURRENT_DRIVER_PROFILE = DEFAULT_DRIVER_PROFILE;

    
    // CAN parameters
    public static final int FRONT_LEFT_STEER_ID = 1;
    public static final int FRONT_RIGHT_STEER_ID = 2;
    public static final int BACK_LEFT_STEER_ID = 3;
    public static final int BACK_RIGHT_STEER_ID = 4;

    public static final int FRONT_LEFT_DRIVE_ID = 5;
    public static final int FRONT_RIGHT_DRIVE_ID = 6;
    public static final int BACK_LEFT_DRIVE_ID = 7;
    public static final int BACK_RIGHT_DRIVE_ID = 8;

    public static final int FRONT_LEFT_CODER_ID = 9;
    public static final int FRONT_RIGHT_CODER_ID = 10;
    public static final int BACK_LEFT_CODER_ID = 11;
    public static final int BACK_RIGHT_CODER_ID = 12;


    // Swerve calculation parameters (in meters)
    public static final double DRIVE_LENGTH = 0.4;
    public static final double DRIVE_WIDTH = 0.3;
    public static final double DRIVE_RADIUS = Math.sqrt( (Math.pow(DRIVE_LENGTH, 2) + Math.pow(DRIVE_WIDTH, 2)) / 2);
    public static final double MAX_MODULE_SPEED = 2; // (m/s)
    public static final double MAX_MODULE_ANGULAR_VELOCITY = 180; // (deg/s)
    public static final double MAX_MODULE_ANGULAR_ACCEL = 360; // (deg/s/s)

    public static final double MODULE_T_STATIC_FF = 1; // Must be tuned for the modules!
    public static final double MODULE_T_VELOCITY_FF = 0.5;
    public static final double MODULE_D_STATIC_FF = 1; 
    public static final double MODULE_D_VELOCITY_FF = 3;

    public static final double MODULE_WHEEL_DIA = 4; // Inches
    public static final double MODULE_WHEEL_DIA_M = Units.inchesToMeters(MODULE_WHEEL_DIA); // Meters (for odometry calculations)

    /**
     * PID parameters
	 * Gains used in each module's steering motor, to be adjusted accordingly
     * Gains(kp, ki, kd, static ff, velocity ff, izone, peak output);
     */
    public static PID_PARAMETERS FL_T_PID_PARAM = new PID_PARAMETERS(0.15, 0.0, 1.0, MODULE_T_STATIC_FF, MODULE_T_VELOCITY_FF, 0, CURRENT_DRIVER_PROFILE.MAX_SPEED);
    public static PID_PARAMETERS FR_T_PID_PARAM = new PID_PARAMETERS(0.15, 0.0, 1.0, MODULE_T_STATIC_FF, MODULE_T_VELOCITY_FF, 0, CURRENT_DRIVER_PROFILE.MAX_SPEED);
    public static PID_PARAMETERS BL_T_PID_PARAM = new PID_PARAMETERS(0.15, 0.0, 1.0, MODULE_T_STATIC_FF, MODULE_T_VELOCITY_FF, 0, CURRENT_DRIVER_PROFILE.MAX_SPEED);
    public static PID_PARAMETERS BR_T_PID_PARAM = new PID_PARAMETERS(0.15, 0.0, 1.0, MODULE_T_STATIC_FF, MODULE_T_VELOCITY_FF, 0, CURRENT_DRIVER_PROFILE.MAX_SPEED);

    public static PID_PARAMETERS FL_D_PID_PARAM = new PID_PARAMETERS(1.0, 0.0, 0, MODULE_D_STATIC_FF, MODULE_D_VELOCITY_FF, 0, CURRENT_DRIVER_PROFILE.MAX_SPEED);
    public static PID_PARAMETERS FR_D_PID_PARAM = new PID_PARAMETERS(1.0, 0.0, 0, MODULE_D_STATIC_FF, MODULE_D_VELOCITY_FF, 0, CURRENT_DRIVER_PROFILE.MAX_SPEED);
    public static PID_PARAMETERS BL_D_PID_PARAM = new PID_PARAMETERS(1.0, 0.0, 0, MODULE_D_STATIC_FF, MODULE_D_VELOCITY_FF, 0, CURRENT_DRIVER_PROFILE.MAX_SPEED);
    public static PID_PARAMETERS BR_D_PID_PARAM = new PID_PARAMETERS(1.0, 0.0, 0, MODULE_D_STATIC_FF, MODULE_D_VELOCITY_FF, 0, CURRENT_DRIVER_PROFILE.MAX_SPEED);


    // Pose estimator parameters (units are m, m, radians)
    public static Matrix<N3,N1> POSE_STANDARD_DEVIATION = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, Math.toRadians(0.125));
    public static Matrix<N1,N1> ENCODER_GYRO_DEVIATION = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(Math.toRadians(0.125));
    public static Matrix<N3,N1> VISION_DEVIATION = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, Math.toRadians(0.125));

    // PID controller (rotation constraints are max velocity and max acceleration)
    public static PIDController MOVEMENT_PID = new PIDController(1, 0, 0);
    public static ProfiledPIDController ROTATION_PID = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(Math.toRadians(360), Math.toRadians(180)));
    
    // DriverStation instance
    public static DriverStation driverStation = DriverStation.getInstance();

    // All of the possible starting positions (and their angles)
    public static Pose2d[] POSSIBLE_STARTING_POSITIONS = {new Pose2d(0, 0, Rotation2d.fromDegrees(0)), new Pose2d(0, 0, Rotation2d.fromDegrees(0)), new Pose2d(0, 0, Rotation2d.fromDegrees(0))};

    // Actual starting position (declared in the global scope)
    public static Pose2d STARTING_POSITION = Parameters.POSSIBLE_STARTING_POSITIONS[Parameters.driverStation.getLocation() - 1];


    // Dynamically allocated Joysticks
    // Joysticks have 11 buttons
    public static int JOYSTICK_BUTTON_COUNT = 11;


    // Vision parameters - used for distance calculations 

    // Camera-specific parameters (pixels)
    public static final double CAMERA_FOCAL_LENGTH = 333.82;


    // Game-specific parameters (inches)
    public static final double GOAL_HEIGHT = 98.25; 
    public static final double POWER_CELL_HEIGHT = 7;



}
