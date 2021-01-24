/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.DriverProfiles;

// Import Parameters
import frc.robot.Parameters;

// Robot instance
import frc.robot.Robot;

// Vendor Libraries
import com.revrobotics.CANSparkMax.IdleMode;

// WPI Libraries
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ProfilingManagement extends SubsystemBase {

  // Gets the saved parameters instance, allows pulling values after reboot
  Preferences SavedParameters = Preferences.getInstance();
  
  // Create a selector on the Dashboard
  private final SendableChooser<DriverProfile> driverProfileChooser = new SendableChooser<>();

  // Sets up the profiling management
  public ProfilingManagement() {

    // Set the current profile to the one loaded from memory
    loadSavedProfile();

    // Set up the drop down for driver profiles
    driverProfileChooser.setDefaultOption("Default", Parameters.DEFAULT_DRIVER_PROFILE);

    // Add each one of the profiles available to the SmartDashboard
    for(int profileIndex = 0; profileIndex < Parameters.DRIVER_PROFILES.length; profileIndex++) {
      driverProfileChooser.addOption(Parameters.DRIVER_PROFILES[profileIndex].NAME, Parameters.DRIVER_PROFILES[profileIndex]);
    }
  }

  // Checks to see if there is an update to the current set profile (from SmartDashboard)
  public void checkForUpdate() {

    // Get the latest
    DriverProfile selectedProfile = driverProfileChooser.getSelected();

    // Check to make sure that the profile isn't the same as the previous one
    if(selectedProfile != Parameters.CURRENT_DRIVER_PROFILE) {
      
      // Update the current profile with the new one
      updateCurrentProfile(selectedProfile);
    }
  }

  // Updates all of the current settings with new ones
  public void updateCurrentProfile(DriverProfile newProfile) {

    // Set the global current profile
    Parameters.CURRENT_DRIVER_PROFILE = newProfile;

    // Update the swerve modules with the new values
    Robot.driveTrain.updateParameters();
  }

  // Saves the current profile to memory
  public void saveProfileSettings() {
    saveProfileSettings(Parameters.CURRENT_DRIVER_PROFILE);
  }

  // Saves the specified profile to memory for next boot
  public void saveProfileSettings(DriverProfile profile) {
    // Saves the input profile for next boot

    // Strings
    SavedParameters.putString("NAME",              profile.NAME);

    // Doubles
    SavedParameters.putDouble("JOYSTICK_DEADZONE", profile.JOYSTICK_DEADZONE);
    SavedParameters.putDouble("MAX_TURN_SPEED",    profile.MAX_TURN_SPEED);
    SavedParameters.putDouble("DRIVE_RAMP_RATE",   profile.DRIVE_RAMP_RATE);
    SavedParameters.putDouble("MAX_SPEED",         profile.MAX_SPEED);

    // Booleans
    SavedParameters.putBoolean("LOCKEM_UP",        profile.LOCKEM_UP);
    SavedParameters.putBoolean("FIELD_CENTRIC",    profile.FIELD_CENTRIC);

    // Special

    // IdleMode is not a supported type of the Preferences class, so brake will be true and coast will be false
    if (profile.DRIVE_IDLE_MODE == IdleMode.kBrake) {
      // Brake
      SavedParameters.putBoolean("BRAKE", true);
    }
    else {
      // Coast 
      SavedParameters.putBoolean("BRAKE", false);
    }
  }

  
  public void loadSavedProfile() {
    // Loads the saved settings

    // Create an empty profile
    DriverProfile profile = new DriverProfile();

    // Strings
    profile.NAME              = SavedParameters.getString("NAME",              Parameters.DEFAULT_DRIVER_PROFILE.NAME);

    // Doubles
    profile.JOYSTICK_DEADZONE = SavedParameters.getDouble("JOYSTICK_DEADZONE", Parameters.DEFAULT_DRIVER_PROFILE.JOYSTICK_DEADZONE);
    profile.MAX_TURN_SPEED    = SavedParameters.getDouble("MAX_TURN_SPEED",    Parameters.DEFAULT_DRIVER_PROFILE.MAX_TURN_SPEED);
    profile.DRIVE_RAMP_RATE   = SavedParameters.getDouble("DRIVE_RAMP_RATE",   Parameters.DEFAULT_DRIVER_PROFILE.DRIVE_RAMP_RATE);
    profile.MAX_SPEED         = SavedParameters.getDouble("MAX_SPEED",         Parameters.DEFAULT_DRIVER_PROFILE.MAX_SPEED);

    // Booleans
    profile.LOCKEM_UP         = SavedParameters.getBoolean("LOCKEM_UP",        Parameters.DEFAULT_DRIVER_PROFILE.LOCKEM_UP);
    profile.FIELD_CENTRIC     = SavedParameters.getBoolean("FIELD_CENTRIC",    Parameters.DEFAULT_DRIVER_PROFILE.FIELD_CENTRIC);

    // Special

    // IdleMode is not a supported type of the Preferences class, so brake will be true and coast will be false
    boolean defaultBrakeMode;

    if (Parameters.DEFAULT_DRIVER_PROFILE.DRIVE_IDLE_MODE == IdleMode.kBrake) {
      // Brake
      defaultBrakeMode = true;
    }
    else {
      // Coast 
      defaultBrakeMode = false;
    }

    if (SavedParameters.getBoolean("BRAKE", defaultBrakeMode)) {
      // Brake
      profile.DRIVE_IDLE_MODE = IdleMode.kBrake;
    }
    else {
      // Coast 
      profile.DRIVE_IDLE_MODE = IdleMode.kCoast;
    }

    // Set the current profile to the values we just obtained
    Parameters.CURRENT_DRIVER_PROFILE = profile;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
