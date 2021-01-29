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
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class NavX extends SubsystemBase {
  /**
   * Creates a new NavX.
   */

  AHRS navX = new AHRS(SPI.Port.kMXP);

  public NavX() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Grabs the yaw
  public float getYaw() {
    return navX.getYaw();
  }

  // Fused heading is like yaw, but on crack
  public float getFusedHeading() {
    return navX.getFusedHeading();
  }

  // The Rotation2D is the big brother of fused heading
  public Rotation2d getFusedRotation2d() {
    return Rotation2d.fromDegrees(navX.getFusedHeading());
  }

  // Grabs the roll
  public float getRoll() {
    return navX.getRoll();
  }

  // Grabs the pitch
  public float getPitch() {
    return navX.getPitch();
  }

  // Gets the x displacement
  public float getDisplacementX() {
    return navX.getDisplacementX();
  }

  // Gets the y displacement
  public float getDisplacementY() {
    return navX.getDisplacementY();
  }

  // Gets the z displacement
  public float getDisplacementZ() {
    return navX.getDisplacementZ();
  }

  // Gets the current degrees
  public float getCompassHeading() {
    return navX.getCompassHeading();
  }

  // Resets the NavX
  public void resetGyro() {
    navX.reset();
  }

  public void resetDisplacement() {
    resetDisplacement();
  }

}