/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * @author Christian Piper (@CAP1Sup)
 * @since 6/3/20
 */

package frc.robot.DriverProfiles;

import com.revrobotics.CANSparkMax.IdleMode;

/**
 * A quick way of referencing driver parameters
 *
 * @param name           The name of the driver
 * @param joystickParams The joystick parameters to use
 * @param maxSteerRate   Maximum deg/s of rotational speed
 * @param driveRampRate  The speed at which the drivetrain ramps to full speed. Prevents sudden jerks. Maybe something to override?
 * @param lockemUp       If the swerve should lock the modules at 45 degrees, effectively hitting the brakes. Hard on the modules, but worth it in competition
 * @param fieldCentric   If the robot should treat itself as forward or if the field's forward should be forward
 * @param maxModSpeed    Maximum speed of modules in m/s
 * @param driveIdleMode  If the drive motors should coast or brake after they exceed the current set speed. Coasting makes the driving smoother, but braking makes it more aggressive
 * @param steerIdleMode  If the steering motor should coast of brake after they exceed the current set speed. Modules will most likely only work with braking enabled
 */
public class DriverProfile {
    public String name;
    public JoystickParams joystickParams;
    public double maxSteerRate, driveRampRate, maxModSpeed;
    public boolean lockemUp, fieldCentric;
    public IdleMode driveIdleMode, steerIdleMode;

    /**
     * Creates a new DriverProfile
     *
     * @param name           The name of the driver
     * @param joystickParams The joystick parameters to use
     * @param maxSteerRate   Maximum deg/s of rotational speed
     * @param driveRampRate  The speed at which the drivetrain ramps to full speed. Prevents sudden jerks. Maybe something to override?
     * @param lockemUp       If the swerve should lock the modules at 45 degrees, effectively hitting the brakes. Hard on the modules, but worth it in competition
     * @param fieldCentric   If the robot should treat itself as forward or if the field's forward should be forward
     * @param maxModSpeed    Maximum speed of modules in m/s
     * @param driveIdleMode  If the drive motors should coast or brake after they exceed the current set speed. Coasting makes the driving smoother, but braking makes it more aggressive
     * @param steerIdleMode  If the steering motor should coast of brake after they exceed the current set speed. Modules will most likely only work with braking enabled
     */
    public DriverProfile(String name, JoystickParams joystickParams, double maxSteerRate,
                         double driveRampRate, boolean lockemUp, boolean fieldCentric,
                         double maxModSpeed, IdleMode driveIdleMode, IdleMode steerIdleMode) {

        // Save each parameter in the profile
        this.name           = name;
        this.joystickParams = joystickParams;
        this.maxSteerRate   = maxSteerRate;
        this.driveRampRate  = driveRampRate;
        this.lockemUp       = lockemUp;
        this.fieldCentric   = fieldCentric;
        this.maxModSpeed    = maxModSpeed;
        this.driveIdleMode  = driveIdleMode;
        this.steerIdleMode  = steerIdleMode;
    }

    // Empty profile
    public DriverProfile() {}

    /**
     * Update the profile with new parameters
     * @param new_params The old profile to copy from
     */
    public void updateProfile(DriverProfile new_params) {
        this.name           = new_params.name;
        this.joystickParams = new_params.joystickParams;
        this.maxSteerRate   = new_params.maxSteerRate;
        this.driveRampRate  = new_params.driveRampRate;
        this.lockemUp       = new_params.lockemUp;
        this.fieldCentric   = new_params.fieldCentric;
        this.maxModSpeed    = new_params.maxModSpeed;
        this.driveIdleMode  = new_params.driveIdleMode;
        this.steerIdleMode  = new_params.steerIdleMode;
    }
}
