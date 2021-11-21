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

// Imports
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Parameters;
import frc.robot.enums.ControlInputs;

/**
 * A quick way of referencing driver parameters
 *
 * @param name The name of the driver
 * @param joystickParams The joystick parameters to use
 * @param maxSteerRate Maximum deg/s of rotational velocity
 * @param lockemUp If the swerve should lock the modules at 45 degrees, effectively hitting the
 *     brakes. Hard on the modules, but worth it in competition
 * @param fieldCentric If the robot should treat itself as forward or if the field's forward should
 *     be forward
 * @param maxModVelocity Maximum velocity of modules in m/s
 * @param driveIdleMode If the drive motors should coast or brake after they exceed the current set
 *     velocity. Coasting makes the driving smoother, but braking makes it more aggressive
 * @param steerIdleMode If the steering motor should coast of brake after they exceed the current
 *     set velocity. Modules will most likely only work with braking enabled
 * @param inputType The devices used to control the robot
 */
public class DriverProfile {
    public String name;
    public JoystickParams joystickParams;
    public double maxSteerRate, maxModVelocity;
    public boolean lockemUp, fieldCentric;
    public IdleMode driveIdleMode, steerIdleMode;
    public ControlInputs inputType;

    /**
     * Creates a new DriverProfile
     *
     * @param name The name of the driver
     * @param joystickParams The joystick parameters to use
     * @param maxSteerRate Maximum deg/s of rotational velocity
     * @param lockemUp If the swerve should lock the modules at 45 degrees, effectively hitting the
     *     brakes. Hard on the modules, but worth it in competition
     * @param fieldCentric If the robot should treat itself as forward or if the field's forward
     *     should be forward
     * @param maxModVelocity Maximum velocity of modules in m/s
     * @param driveIdleMode If the drive motors should coast or brake after they exceed the current
     *     set velocity. Coasting makes the driving smoother, but braking makes it more aggressive
     * @param steerIdleMode If the steering motor should coast of brake after they exceed the
     *     current set velocity. Modules will most likely only work with braking enabled
     * @param inputType The devices used to control the robot
     */
    public DriverProfile(
            String name,
            JoystickParams joystickParams,
            double maxSteerRate,
            boolean lockemUp,
            boolean fieldCentric,
            double maxModVelocity,
            IdleMode driveIdleMode,
            IdleMode steerIdleMode,
            ControlInputs inputType) {

        // Save each parameter in the profile
        this.name = name;
        this.joystickParams = joystickParams;
        this.maxSteerRate = maxSteerRate;
        this.lockemUp = lockemUp;
        this.fieldCentric = fieldCentric;
        this.maxModVelocity = maxModVelocity;
        this.driveIdleMode = driveIdleMode;
        this.steerIdleMode = steerIdleMode;
        this.inputType = inputType;
    }

    // Empty profile
    public DriverProfile() {

        // Save each parameter from the default profile
        this.name = Parameters.driver.defaultDriverProfile.name;
        this.joystickParams = Parameters.driver.defaultDriverProfile.joystickParams;
        this.maxSteerRate = Parameters.driver.defaultDriverProfile.maxSteerRate;
        this.lockemUp = Parameters.driver.defaultDriverProfile.lockemUp;
        this.fieldCentric = Parameters.driver.defaultDriverProfile.fieldCentric;
        this.maxModVelocity = Parameters.driver.defaultDriverProfile.maxModVelocity;
        this.driveIdleMode = Parameters.driver.defaultDriverProfile.driveIdleMode;
        this.steerIdleMode = Parameters.driver.defaultDriverProfile.steerIdleMode;
        this.inputType = Parameters.driver.defaultDriverProfile.inputType;
    }

    /**
     * Update the profile with new parameters
     *
     * @param new_params The old profile to copy from
     */
    public void updateProfile(DriverProfile new_params) {
        this.name = new_params.name;
        this.joystickParams = new_params.joystickParams;
        this.maxSteerRate = new_params.maxSteerRate;
        this.lockemUp = new_params.lockemUp;
        this.fieldCentric = new_params.fieldCentric;
        this.maxModVelocity = new_params.maxModVelocity;
        this.driveIdleMode = new_params.driveIdleMode;
        this.steerIdleMode = new_params.steerIdleMode;
        this.inputType = new_params.inputType;
    }
}
