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
 *
 * @param NAME                 The name of the driver
 * @param JOYSTICK_PARAMS      The joystick parameters to use
 * @param MAX_STEER_SPEED      Maximum deg/s of rotational speed
 * @param DRIVE_RAMP_RATE      The speed at which the drivetrain ramps to full speed. Prevents sudden jerks. Maybe something to override?
 * @param LOCKEM_UP            If the swerve should lock the modules at 45 degrees, effectively hitting the brakes. Hard on the modules, but worth it in competition
 * @param FIELD_CENTRIC        If the robot should treat itself as forward or if the field's forward should be forward
 * @param MAX_SPEED            Maximum speed of modules in m/s
 * @param DRIVE_IDLE_MODE      If the drive motors should coast or brake after they exceed the current set speed. Coasting makes the driving smoother, but braking makes it more aggressive
 * @param STEER_IDLE_MODE      If the steering motor should coast of brake after they exceed the current set speed. Modules will most likely only work with braking enabled
 */
public class DriverProfile {
    public String NAME;
    public JoystickParams JOYSTICK_PARAMS;
    public double MAX_STEER_SPEED, DRIVE_RAMP_RATE, MAX_SPEED;
    public boolean LOCKEM_UP, FIELD_CENTRIC;
    public IdleMode DRIVE_IDLE_MODE, STEER_IDLE_MODE;

    /**
     * Creates a new DriverProfile
     *
     * @param NAME                 The name of the driver
     * @param JOYSTICK_PARAMS      The joystick parameters to use
     * @param MAX_STEER_SPEED      Maximum deg/s of rotational speed
     * @param DRIVE_RAMP_RATE      The speed at which the drivetrain ramps to full speed. Prevents sudden jerks. Maybe something to override?
     * @param LOCKEM_UP            If the swerve should lock the modules at 45 degrees, effectively hitting the brakes. Hard on the modules, but worth it in competition
     * @param FIELD_CENTRIC        If the robot should treat itself as forward or if the field's forward should be forward
     * @param MAX_SPEED            Maximum speed of modules in m/s
     * @param DRIVE_IDLE_MODE      If the drive motors should coast or brake after they exceed the current set speed. Coasting makes the driving smoother, but braking makes it more aggressive
     * @param STEER_IDLE_MODE      If the steering motor should coast of brake after they exceed the current set speed. Modules will most likely only work with braking enabled
     */
    public DriverProfile(String NAME, JoystickParams JOYSTICK_PARAMS, double MAX_STEER_SPEED,
                         double DRIVE_RAMP_RATE, boolean LOCKEM_UP, boolean FIELD_CENTRIC,
                         double MAX_SPEED, IdleMode DRIVE_IDLE_MODE, IdleMode STEER_IDLE_MODE) {

        // Save each parameter in the profile
        this.NAME            = NAME;
        this.JOYSTICK_PARAMS = JOYSTICK_PARAMS;
        this.MAX_STEER_SPEED = MAX_STEER_SPEED;
        this.DRIVE_RAMP_RATE = DRIVE_RAMP_RATE;
        this.LOCKEM_UP       = LOCKEM_UP;
        this.FIELD_CENTRIC   = FIELD_CENTRIC;
        this.MAX_SPEED       = MAX_SPEED;
        this.DRIVE_IDLE_MODE = DRIVE_IDLE_MODE;
        this.STEER_IDLE_MODE = STEER_IDLE_MODE;
    }

    // Empty profile
    public DriverProfile() {}

    /**
     * Update the profile with new parameters
     * @param new_params The old profile to copy from
     */
    public void updateProfile(DriverProfile new_params) {
        this.NAME            = new_params.NAME;
        this.JOYSTICK_PARAMS = new_params.JOYSTICK_PARAMS;
        this.MAX_STEER_SPEED = new_params.MAX_STEER_SPEED;
        this.DRIVE_RAMP_RATE = new_params.DRIVE_RAMP_RATE;
        this.LOCKEM_UP       = new_params.LOCKEM_UP;
        this.FIELD_CENTRIC   = new_params.FIELD_CENTRIC;
        this.MAX_SPEED       = new_params.MAX_SPEED;
        this.DRIVE_IDLE_MODE = new_params.DRIVE_IDLE_MODE;
        this.STEER_IDLE_MODE = new_params.STEER_IDLE_MODE;
    }
}
