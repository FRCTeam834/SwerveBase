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


public class ProfilingManagement extends SubsystemBase {

  /**
   * Creates a new DriverProfiling.
   */
  public ProfilingManagement() {

  }

  public void updateCurrentProfile(DriverProfile newProfile) {
    Parameters.currentDriverProfile = newProfile;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
