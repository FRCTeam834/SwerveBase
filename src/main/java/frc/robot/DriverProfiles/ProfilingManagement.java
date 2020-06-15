/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.DriverProfiles;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import Parameters
import frc.robot.Parameters;

import com.revrobotics.CANSparkMax.IdleMode;

// WPI Libaries
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class ProfilingManagement extends SubsystemBase {

  Preferences SavedParameters = Preferences.getInstance();
  
  private final SendableChooser<DriverProfile> driverProfileChooser = new SendableChooser<>();
  /**
   * Creates a new DriverProfiling.
   */
  public ProfilingManagement() {
    // Set up the drop down for driver profiles
    //driverProfileChooser.setDefaultOption("Default", new updateCurrentProfile(Parameters.defaultDriverProfile));
  }

  

  public void updateCurrentProfile(DriverProfile newProfile) {
    Parameters.currentDriverProfile = newProfile;
  }


  public void saveProfileSettings(DriverProfile profile) {
    // Saves the input profile for next boot

    // Strings
    SavedParameters.putString("NAME",              profile.NAME);

    // Doubles
    SavedParameters.putDouble("JOYSTICK_DEADZONE", profile.JOYSTICK_DEADZONE);
    SavedParameters.putDouble("TURN_SCALE",        profile.TURN_SCALE);
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

  
  public DriverProfile loadSavedProfile() {
    // Loads the saved settings

    // Create an empty profile
    DriverProfile profile = new DriverProfile();

    // Strings
    profile.NAME              = SavedParameters.getString("NAME",              Parameters.defaultDriverProfile.NAME);

    // Doubles
    profile.JOYSTICK_DEADZONE = SavedParameters.getDouble("JOYSTICK_DEADZONE", Parameters.defaultDriverProfile.JOYSTICK_DEADZONE);
    profile.TURN_SCALE        = SavedParameters.getDouble("TURN_SCALE",        Parameters.defaultDriverProfile.TURN_SCALE);
    profile.DRIVE_RAMP_RATE   = SavedParameters.getDouble("DRIVE_RAMP_RATE",   Parameters.defaultDriverProfile.DRIVE_RAMP_RATE);
    profile.MAX_SPEED         = SavedParameters.getDouble("MAX_SPEED",         Parameters.defaultDriverProfile.MAX_SPEED);

    // Booleans
    profile.LOCKEM_UP         = SavedParameters.getBoolean("LOCKEM_UP",        Parameters.defaultDriverProfile.LOCKEM_UP);
    profile.FIELD_CENTRIC     = SavedParameters.getBoolean("FIELD_CENTRIC",    Parameters.defaultDriverProfile.FIELD_CENTRIC);

    // Special

    // IdleMode is not a supported type of the Preferences class, so brake will be true and coast will be false
    boolean defaultBrakeMode;

    if (Parameters.defaultDriverProfile.DRIVE_IDLE_MODE == IdleMode.kBrake) {
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

    // Return the profile
    return profile;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
