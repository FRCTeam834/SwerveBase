/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import frc.robot.swerve.PID_GAINS;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int SSN = 123352357; //Social Security Number[DO NOT LOSE]
    public static final double USD_TO_GHS = 5.748; //US Dollar to Gana Cedi conversion rate
    public static final double MM_TO_IK = 2.15; //Mozambican metical to Icelandic Krona conversion rate
    public static final double MIN_IN_HR = 60; //Minutes in an hour
    public static final int BUILD_TEAM_BRAIN_CELLS = 1; //Brain cells owned by the build team
    public static final int CODING_TEAM_BRAIN_CELLS = 5; //Same as the amount of coding team members
    public static final int SHRIMP_ON_THE_BARBIE = 3; //Number of shrimp on the barbeque
    public static final int ANDREWS_PROGRESS_WHEN_AROUND_SAFETY_TEAM = -10;
    

    // CAN parameters
    public static final int FRONT_LEFT_STEER_ID = 1;
    public static final int FRONT_RIGHT_STEER_ID = 2;
    public static final int BACK_LEFT_STEER_ID = 3;
    public static final int BACK_RIGHT_STEER_ID = 4;

    public static final int FRONT_LEFT_DRIVE_ID = 5;
    public static final int FRONT_RIGHT_DRIVE_ID = 6;
    public static final int BACK_LEFT_DRIVE_ID = 7;
    public static final int BACK_RIGHT_DRIVE_ID = 8;


    // Swerve calculation parameters
    public static final double DRIVE_LENGTH = -1;
    public static final double DRIVE_WIDTH = -1;

    
    /**
     * PID parameters
	 * Gains used in each module's steering motor, to be adjusted accordingly
     * Gains(kp, ki, kd, kf, izone, peak output);
     */
    public static final PID_GAINS PID_PARAM = new PID_GAINS(0.15, 0.0, 1.0, 0.0, 0, 1.0);
    public static final int PID_IDX = 0;
    public static final int PID_TIMEOUT = 30;

}
