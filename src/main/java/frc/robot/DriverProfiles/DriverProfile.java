/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.DriverProfiles;

import com.revrobotics.CANSparkMax.IdleMode;


/**
 * A quick way of referencing driver parameters
 */
public class DriverProfile {
    public String NAME;
    public double JOYSTICK_DEADZONE, TURN_SCALE, DRIVE_RAMP_RATE, MAX_SPEED;
    public boolean LOCKEM_UP, FIELD_CENTRIC;
    public IdleMode DRIVE_IDLE_MODE;
    
    // Fully defined profile
    public DriverProfile(String NAME, double JOYSTICK_DEADZONE, double TURN_SCALE, double DRIVE_RAMP_RATE, boolean LOCKEM_UP, boolean FIELD_CENTRIC, double MAX_SPEED, IdleMode DRIVE_IDLE_MODE) {
        this.NAME              = NAME;
        this.JOYSTICK_DEADZONE = JOYSTICK_DEADZONE;
        this.TURN_SCALE        = TURN_SCALE;
        this.DRIVE_RAMP_RATE   = DRIVE_RAMP_RATE;
        this.LOCKEM_UP         = LOCKEM_UP;
        this.FIELD_CENTRIC     = FIELD_CENTRIC;
        this.MAX_SPEED         = MAX_SPEED;
        this.DRIVE_IDLE_MODE   = DRIVE_IDLE_MODE;
    }

    // Empty profile
    public DriverProfile() {

    }

    // Update a profile with new values
    public void updateProfile(DriverProfile profile) {
        this.NAME              = profile.NAME;
        this.JOYSTICK_DEADZONE = profile.JOYSTICK_DEADZONE;
        this.TURN_SCALE        = profile.TURN_SCALE;
        this.DRIVE_RAMP_RATE   = profile.DRIVE_RAMP_RATE;
        this.LOCKEM_UP         = profile.LOCKEM_UP;
        this.FIELD_CENTRIC     = profile.FIELD_CENTRIC;
        this.MAX_SPEED         = profile.MAX_SPEED;
        this.DRIVE_IDLE_MODE   = profile.DRIVE_IDLE_MODE;
    }

    
}
