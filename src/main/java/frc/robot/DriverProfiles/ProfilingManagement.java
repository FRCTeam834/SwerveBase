/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * @author Christian Piper (@CAP1Sup)
 * @since 5/22/20
 */
package frc.robot.DriverProfiles;

// Imports
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Parameters;
import frc.robot.Robot;
import frc.robot.enums.ControlInputs;
import frc.robot.enums.JoystickOutputTypes;

public class ProfilingManagement extends SubsystemBase {

  // Create a selector on the Dashboard
  private final SendableChooser<DriverProfile> driverProfileChooser = new SendableChooser<>();

  // Sets up the profiling management
  public ProfilingManagement() {

    // Set the current profile to the one loaded from memory
    loadSavedProfile();

    // Set up the drop down for driver profiles
    driverProfileChooser.setDefaultOption("Default", Parameters.driver.defaultDriverProfile);

    // Add each one of the profiles available to the SmartDashboard
    for (int profileIndex = 0;
        profileIndex < Parameters.driver.driverProfiles.length;
        profileIndex++) {
      driverProfileChooser.addOption(
          Parameters.driver.driverProfiles[profileIndex].name,
          Parameters.driver.driverProfiles[profileIndex]);
    }
  }

  // Checks to see if there is an update to the current set profile (from SmartDashboard)
  public void checkForUpdate() {

    // Get the latest
    DriverProfile selectedProfile = driverProfileChooser.getSelected();

    // Check to make sure that the profile isn't the same as the previous one
    if (!selectedProfile.equals(Parameters.driver.currentProfile)) {

      // Update the current profile with the new one
      updateCurrentProfile(selectedProfile);
    }
  }

  // Updates all of the current settings with new ones
  public void updateCurrentProfile(DriverProfile newProfile) {

    // Set the global current profile
    Parameters.driver.currentProfile = newProfile;

    // Update the swerve modules with the new values
    Robot.driveTrain.updateParameters();
  }

  // Saves the current profile to memory
  public void saveProfileSettings() {
    saveProfileSettings(Parameters.driver.currentProfile);
  }

  // Saves the specified profile to memory for next boot
  public void saveProfileSettings(DriverProfile profile) {
    // Saves the input profile for next boot

    // Strings
    Parameters.savedParams.putString("NAME", profile.name);

    // Ints / Doubles
    Parameters.savedParams.putDouble("JOYSTICK_DEADZONE", profile.joystickParams.getDeadzone());
    Parameters.savedParams.putDouble("JOYSTICK_RAMP_RATE", profile.joystickParams.getRampRate());
    Parameters.savedParams.putInt(
        "JOYSTICK_OUTPUT_TYPE", profile.joystickParams.getOutputType().getInt());
    Parameters.savedParams.putInt("CONTROL_INPUTS", profile.inputType.getInt());
    Parameters.savedParams.putDouble("MAX_STEER_RATE", profile.maxSteerRate);
    Parameters.savedParams.putDouble("MAX_MOD_VELOCITY", profile.maxModVelocity);
    Parameters.savedParams.putInt("DRIVE_BRAKE_MODE", profile.driveIdleMode.value);
    Parameters.savedParams.putInt("STEER_BRAKE_MODE", profile.steerIdleMode.value);

    // Booleans
    Parameters.savedParams.putBoolean("LOCKEM_UP", profile.lockemUp);
    Parameters.savedParams.putBoolean("FIELD_CENTRIC", profile.fieldCentric);
  }

  public void loadSavedProfile() {
    // Loads the saved settings

    // Create an empty profile
    DriverProfile profile = new DriverProfile();

    // Strings
    profile.name =
        Parameters.savedParams.getString("NAME", Parameters.driver.defaultDriverProfile.name);

    // Ints / Doubles
    double deadzone =
        Parameters.savedParams.getDouble(
            "JOYSTICK_DEADZONE",
            Parameters.driver.defaultDriverProfile.joystickParams.getDeadzone());
    double rampRate =
        Parameters.savedParams.getDouble(
            "JOYSTICK_RAMP_RATE",
            Parameters.driver.defaultDriverProfile.joystickParams.getRampRate());
    JoystickOutputTypes outputType =
        JoystickOutputTypes.fromInt(
            Parameters.savedParams.getInt(
                "JOYSTICK_OUTPUT_TYPE",
                Parameters.driver.defaultDriverProfile.joystickParams.getOutputType().getInt()));
    profile.inputType =
        ControlInputs.fromInt(
            Parameters.savedParams.getInt(
                "CONTROL_INPUTS", Parameters.driver.defaultDriverProfile.inputType.getInt()));
    profile.joystickParams = new JoystickParams(deadzone, outputType, rampRate);
    profile.maxSteerRate =
        Parameters.savedParams.getDouble(
            "MAX_STEER_RATE", Parameters.driver.defaultDriverProfile.maxSteerRate);
    profile.maxModVelocity =
        Parameters.savedParams.getDouble(
            "MAX_MOD_VELOCITY", Parameters.driver.defaultDriverProfile.maxModVelocity);

    // Booleans
    profile.lockemUp =
        Parameters.savedParams.getBoolean(
            "LOCKEM_UP", Parameters.driver.defaultDriverProfile.lockemUp);
    profile.fieldCentric =
        Parameters.savedParams.getBoolean(
            "FIELD_CENTRIC", Parameters.driver.defaultDriverProfile.fieldCentric);

    // Special (the braking modes)
    profile.driveIdleMode =
        Parameters.savedParams.getInt("DRIVE_BRAKE_MODE", profile.driveIdleMode.value) == 0
            ? IdleMode.kCoast
            : IdleMode.kBrake;
    profile.steerIdleMode =
        Parameters.savedParams.getInt("STEER_BRAKE_MODE", profile.steerIdleMode.value) == 0
            ? IdleMode.kCoast
            : IdleMode.kBrake;

    // Set the current profile to the values we just obtained
    Parameters.driver.currentProfile = profile;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
