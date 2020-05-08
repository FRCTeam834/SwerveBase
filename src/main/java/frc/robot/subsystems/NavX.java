/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;

public class NavX extends SubsystemBase {
  /**
   * Creates a new NavX.
   */

  AHRS ahrs = new AHRS(SPI.Port.kMXP);

  public NavX() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Grabs the yaw
  public float getYaw() {
    return ahrs.getYaw();
  }

  // Grabs the roll
  public float getRoll() {
    return ahrs.getRoll();
  }

  // Grabs the pitch
  public float getPitch() {
    return ahrs.getPitch();
  }

  // Gets the x displacement
  public float getDisplacementX() {
    return ahrs.getDisplacementX();
  }

  // Gets the y displacement
  public float getDisplacementY() {
    return ahrs.getDisplacementY();
  }

  // Gets the z displacement
  public float getDisplacementZ() {
    return ahrs.getDisplacementZ();
  }

  // Gets the current degrees
  public float getCurrentDegrees() {
    return ahrs.getCompassHeading();
  }

  // Resets the NavX
  public void resetGyro() {
    ahrs.reset();
  }

  public void resetDisplacement() {
    resetDisplacement();
  }

}