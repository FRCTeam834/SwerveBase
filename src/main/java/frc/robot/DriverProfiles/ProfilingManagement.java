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

// Import Parameters
import frc.robot.Parameters;

// Robot instance
import frc.robot.Robot;
import frc.robot.enums.JOYSTICK_OUTPUT_TYPES;

// Vendor Libraries
import com.revrobotics.CANSparkMax.IdleMode;

// WPI Libraries
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


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
    for(int profileIndex = 0; profileIndex < Parameters.driver.driverProfiles.length; profileIndex++) {
      driverProfileChooser.addOption(Parameters.driver.driverProfiles[profileIndex].name, Parameters.driver.driverProfiles[profileIndex]);
    }
  }

  // Checks to see if there is an update to the current set profile (from SmartDashboard)
  public void checkForUpdate() {

    // Get the latest
    DriverProfile selectedProfile = driverProfileChooser.getSelected();

    // Check to make sure that the profile isn't the same as the previous one
    if(selectedProfile != Parameters.driver.currentProfile) {

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
    Parameters.savedParams.putString("NAME",              profile.name);

    // Ints / Doubles
    Parameters.savedParams.putDouble("JOYSTICK_DEADZONE",   profile.joystickParams.getDeadzone());
    Parameters.savedParams.putDouble("JOYSTICK_RAMP_CONST", profile.joystickParams.getRampRate());
    Parameters.savedParams.putInt("JOYSTICK_OUTPUT_TYPE",   profile.joystickParams.getOutputType().getInt());
    Parameters.savedParams.putDouble("MAX_STEER_SPEED",     profile.maxSteerRate);
    Parameters.savedParams.putDouble("DRIVE_RAMP_RATE",     profile.driveRampRate);
    Parameters.savedParams.putDouble("MAX_SPEED",           profile.maxModSpeed);

    // Booleans
    Parameters.savedParams.putBoolean("LOCKEM_UP",        profile.lockemUp);
    Parameters.savedParams.putBoolean("FIELD_CENTRIC",    profile.fieldCentric);

    // Special

    // IdleMode is not a supported type of the Preferences class, so brake will be true and coast will be false
    if (profile.driveIdleMode == IdleMode.kBrake) {
      // Brake
      Parameters.savedParams.putBoolean("BRAKE", true);
    }
    else {
      // Coast
      Parameters.savedParams.putBoolean("BRAKE", false);
    }
  }


  public void loadSavedProfile() {
    // Loads the saved settings

    // Create an empty profile
    DriverProfile profile = new DriverProfile();

    // Strings
    profile.name              = Parameters.savedParams.getString("NAME",              Parameters.driver.defaultDriverProfile.name);

    // Ints / Doubles
    double deadzone                  = Parameters.savedParams.getDouble("JOYSTICK_DEADZONE", Parameters.driver.defaultDriverProfile.joystickParams.getDeadzone());
    double rampRate                  = Parameters.savedParams.getDouble("JOYSTICK_RAMP_RATE", Parameters.driver.defaultDriverProfile.joystickParams.getRampRate());
    JOYSTICK_OUTPUT_TYPES outputType = JOYSTICK_OUTPUT_TYPES.fromInt(Parameters.savedParams.getInt("JOYSTICK_OUTPUT_TYPE", Parameters.driver.defaultDriverProfile.joystickParams.getOutputType().getInt()));
    profile.joystickParams           = new JoystickParams(deadzone, outputType, rampRate);
    profile.maxSteerRate             = Parameters.savedParams.getDouble("MAX_STEER_SPEED",   Parameters.driver.defaultDriverProfile.maxSteerRate);
    profile.driveRampRate            = Parameters.savedParams.getDouble("DRIVE_RAMP_RATE",   Parameters.driver.defaultDriverProfile.driveRampRate);
    profile.maxModSpeed              = Parameters.savedParams.getDouble("MAX_SPEED",         Parameters.driver.defaultDriverProfile.maxModSpeed);

    // Booleans
    profile.lockemUp         =  Parameters.savedParams.getBoolean("LOCKEM_UP",        Parameters.driver.defaultDriverProfile.lockemUp);
    profile.fieldCentric     =  Parameters.savedParams.getBoolean("FIELD_CENTRIC",    Parameters.driver.defaultDriverProfile.fieldCentric);

    // Special

    // IdleMode is not a supported type of the Preferences class, so brake will be true and coast will be false
    boolean defaultBrakeMode;

    if (Parameters.driver.defaultDriverProfile.driveIdleMode == IdleMode.kBrake) {
      // Brake
      defaultBrakeMode = true;
    }
    else {
      // Coast
      defaultBrakeMode = false;
    }

    if (Parameters.savedParams.getBoolean("BRAKE", defaultBrakeMode)) {
      // Brake
      profile.driveIdleMode = IdleMode.kBrake;
    }
    else {
      // Coast
      profile.driveIdleMode = IdleMode.kCoast;
    }

    // Set the current profile to the values we just obtained
    Parameters.driver.currentProfile = profile;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
