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
    driverProfileChooser.setDefaultOption("Default", Parameters.driver.DEFAULT_DRIVER_PROFILE);

    // Add each one of the profiles available to the SmartDashboard
    for(int profileIndex = 0; profileIndex < Parameters.driver.DRIVER_PROFILES.length; profileIndex++) {
      driverProfileChooser.addOption(Parameters.driver.DRIVER_PROFILES[profileIndex].NAME, Parameters.driver.DRIVER_PROFILES[profileIndex]);
    }
  }

  // Checks to see if there is an update to the current set profile (from SmartDashboard)
  public void checkForUpdate() {

    // Get the latest
    DriverProfile selectedProfile = driverProfileChooser.getSelected();

    // Check to make sure that the profile isn't the same as the previous one
    if(selectedProfile != Parameters.driver.CURRENT_PROFILE) {
      
      // Update the current profile with the new one
      updateCurrentProfile(selectedProfile);
    }
  }

  // Updates all of the current settings with new ones
  public void updateCurrentProfile(DriverProfile newProfile) {

    // Set the global current profile
    Parameters.driver.CURRENT_PROFILE = newProfile;

    // Update the swerve modules with the new values
    Robot.driveTrain.updateParameters();
  }

  // Saves the current profile to memory
  public void saveProfileSettings() {
    saveProfileSettings(Parameters.driver.CURRENT_PROFILE);
  }

  // Saves the specified profile to memory for next boot
  public void saveProfileSettings(DriverProfile profile) {
    // Saves the input profile for next boot

    // Strings
    Parameters.SAVED_PARAMS.putString("NAME",              profile.NAME);

    // Doubles
    Parameters.SAVED_PARAMS.putDouble("JOYSTICK_DEADZONE", profile.JOYSTICK_DEADZONE);
    Parameters.SAVED_PARAMS.putDouble("MAX_STEER_SPEED",   profile.MAX_STEER_SPEED);
    Parameters.SAVED_PARAMS.putDouble("DRIVE_RAMP_RATE",   profile.DRIVE_RAMP_RATE);
    Parameters.SAVED_PARAMS.putDouble("MAX_SPEED",         profile.MAX_SPEED);

    // Booleans
    Parameters.SAVED_PARAMS.putBoolean("LOCKEM_UP",        profile.LOCKEM_UP);
    Parameters.SAVED_PARAMS.putBoolean("FIELD_CENTRIC",    profile.FIELD_CENTRIC);

    // Special

    // IdleMode is not a supported type of the Preferences class, so brake will be true and coast will be false
    if (profile.DRIVE_IDLE_MODE == IdleMode.kBrake) {
      // Brake
      Parameters.SAVED_PARAMS.putBoolean("BRAKE", true);
    }
    else {
      // Coast 
      Parameters.SAVED_PARAMS.putBoolean("BRAKE", false);
    }
  }

  
  public void loadSavedProfile() {
    // Loads the saved settings

    // Create an empty profile
    DriverProfile profile = new DriverProfile();

    // Strings
    profile.NAME              = Parameters.SAVED_PARAMS.getString("NAME",              Parameters.driver.DEFAULT_DRIVER_PROFILE.NAME);

    // Doubles
    profile.JOYSTICK_DEADZONE = Parameters.SAVED_PARAMS.getDouble("JOYSTICK_DEADZONE", Parameters.driver.DEFAULT_DRIVER_PROFILE.JOYSTICK_DEADZONE);
    profile.MAX_STEER_SPEED   = Parameters.SAVED_PARAMS.getDouble("MAX_STEER_SPEED",   Parameters.driver.DEFAULT_DRIVER_PROFILE.MAX_STEER_SPEED);
    profile.DRIVE_RAMP_RATE   = Parameters.SAVED_PARAMS.getDouble("DRIVE_RAMP_RATE",   Parameters.driver.DEFAULT_DRIVER_PROFILE.DRIVE_RAMP_RATE);
    profile.MAX_SPEED         = Parameters.SAVED_PARAMS.getDouble("MAX_SPEED",         Parameters.driver.DEFAULT_DRIVER_PROFILE.MAX_SPEED);

    // Booleans
    profile.LOCKEM_UP         =  Parameters.SAVED_PARAMS.getBoolean("LOCKEM_UP",        Parameters.driver.DEFAULT_DRIVER_PROFILE.LOCKEM_UP);
    profile.FIELD_CENTRIC     =  Parameters.SAVED_PARAMS.getBoolean("FIELD_CENTRIC",    Parameters.driver.DEFAULT_DRIVER_PROFILE.FIELD_CENTRIC);

    // Special

    // IdleMode is not a supported type of the Preferences class, so brake will be true and coast will be false
    boolean defaultBrakeMode;

    if (Parameters.driver.DEFAULT_DRIVER_PROFILE.DRIVE_IDLE_MODE == IdleMode.kBrake) {
      // Brake
      defaultBrakeMode = true;
    }
    else {
      // Coast 
      defaultBrakeMode = false;
    }

    if (Parameters.SAVED_PARAMS.getBoolean("BRAKE", defaultBrakeMode)) {
      // Brake
      profile.DRIVE_IDLE_MODE = IdleMode.kBrake;
    }
    else {
      // Coast 
      profile.DRIVE_IDLE_MODE = IdleMode.kCoast;
    }

    // Set the current profile to the values we just obtained
    Parameters.driver.CURRENT_PROFILE = profile;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
