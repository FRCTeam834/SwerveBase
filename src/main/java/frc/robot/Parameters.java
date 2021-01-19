/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// Robot 
import frc.robot.Robot;

// Internal libraries
import frc.robot.DriverProfiles.DriverProfile;
import frc.robot.DriverProfiles.ProfilingManagement;
import frc.robot.swerve.PID_PARAMETERS;

// Vendor libraries
import com.revrobotics.CANSparkMax.IdleMode;

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
    public static final int SHRIMP_ON_THE_BARBIE = 3; //Number of shrimp on the barbeque
    public static final int ANDREWS_PROGRESS_WHEN_AROUND_SAFETY_TEAM = -10; //What happens when Andrew is around the safety team... backwards progress

    // Driver Profiles
    public static DriverProfile[] driverProfiles = {
            // DriverProfile NAME, double JOYSTICK_DEADZONE, double TURN_SCALE, double DRIVE_RAMP_RATE, boolean LOCKEM_UP, boolean FIELD_CENTRIC, double MAX_SPEED, IdleMode DRIVE_IDLE_MODE) 
            new DriverProfile("CAP1Sup",         0.05, 1.0, 0.5, true, true, 1.0, IdleMode.kBrake),
            new DriverProfile("Christian Velez", 0.15, 1.0, 0.5, true, true, 1.0, IdleMode.kBrake),
            new DriverProfile("Test",            0.15, 1.0, 0.5, true, true, 1.0, IdleMode.kBrake)
    };

    public static DriverProfile defaultDriverProfile = new DriverProfile("Default", 0.15, 1.0, 0.5, true, true, 1.0, IdleMode.kBrake);


    // Current Driver Profile being used
    public static DriverProfile currentDriverProfile = Robot.profilingManagement.loadSavedProfile();

    
    // CAN parameters
    public static final int FRONT_LEFT_STEER_ID = 1;
    public static final int FRONT_RIGHT_STEER_ID = 2;
    public static final int BACK_LEFT_STEER_ID = 3;
    public static final int BACK_RIGHT_STEER_ID = 4;

    public static final int FRONT_LEFT_DRIVE_ID = 5;
    public static final int FRONT_RIGHT_DRIVE_ID = 6;
    public static final int BACK_LEFT_DRIVE_ID = 7;
    public static final int BACK_RIGHT_DRIVE_ID = 8;


    // Swerve calculation parameters (in meters)
    public static final double DRIVE_LENGTH = 0.4;
    public static final double DRIVE_WIDTH = 0.3;
    public static final double DRIVE_RADIUS = Math.sqrt( (Math.pow(DRIVE_LENGTH, 2) + Math.pow(DRIVE_WIDTH, 2)) / 2);

    public static final int ENCODER_COUNTS_PER_REVOLUTION = 1024;
    public static final double SWERVE_WHEEL_DIA = 4; // Inches
    public static final double SWERVE_WHEEL_DIA_M = SWERVE_WHEEL_DIA / 39.37; // Meters (for odometry calculations)

    /**
     * PID parameters
	 * Gains used in each module's steering motor, to be adjusted accordingly
     * Gains(kp, ki, kd, kf, izone, PID timeout, peak output);
     */
    public static PID_PARAMETERS FL_T_PID_PARAM = new PID_PARAMETERS(0.15, 0.0, 1.0, 0.0, 0, currentDriverProfile.MAX_SPEED);
    public static PID_PARAMETERS FR_T_PID_PARAM = new PID_PARAMETERS(0.15, 0.0, 1.0, 0.0, 0, currentDriverProfile.MAX_SPEED);
    public static PID_PARAMETERS BL_T_PID_PARAM = new PID_PARAMETERS(0.15, 0.0, 1.0, 0.0, 0, currentDriverProfile.MAX_SPEED);
    public static PID_PARAMETERS BR_T_PID_PARAM = new PID_PARAMETERS(0.15, 0.0, 1.0, 0.0, 0, currentDriverProfile.MAX_SPEED);

    public static PID_PARAMETERS FL_D_PID_PARAM = new PID_PARAMETERS(1.0, 0.0, 0, 0.0, 0, currentDriverProfile.MAX_SPEED);
    public static PID_PARAMETERS FR_D_PID_PARAM = new PID_PARAMETERS(1.0, 0.0, 0, 0.0, 0, currentDriverProfile.MAX_SPEED);
    public static PID_PARAMETERS BL_D_PID_PARAM = new PID_PARAMETERS(1.0, 0.0, 0, 0.0, 0, currentDriverProfile.MAX_SPEED);
    public static PID_PARAMETERS BR_D_PID_PARAM = new PID_PARAMETERS(1.0, 0.0, 0, 0.0, 0, currentDriverProfile.MAX_SPEED);

    public static int PID_IDX = 0;
    public static int PID_TIMEOUT = 30;


    // Vision parameters - used for distance calculations 

    // Camera-specific parameters (pixels)
    public static final double CAMERA_FOCAL_LENGTH = 333.82;


    // Game-specific parameters (inches)
    public static final double GOAL_HEIGHT = 34; 
    public static final double POWER_CELL_HEIGHT = 7;



}
