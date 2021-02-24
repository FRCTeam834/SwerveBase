// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Ultrasonic;
import frc.robot.Parameters;

public class UltrasonicSensor extends SubsystemBase {
  /** Creates a new Ultrasonic. */

  // Create a new Ultrasonic sensor
  Ultrasonic sensor = new Ultrasonic(Parameters.ultrasonic.US_PING, Parameters.ultrasonic.US_ECHO);

  // Main constructor
  public UltrasonicSensor() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Reads ultrasonic sensor and returns distance in meters
  public double read(){
    return (sensor.getRangeMM() * 1000);
  }

  // Check if the ultrasonic reading is close to the desired value within the tolerance (units are in m)
  public boolean withinRange(double desired, double tolerance) {

    // Reading from the sensor
    double reading = read();

    // Return if it's in range
    return ((reading > (desired - tolerance)) && (reading < (desired + tolerance)));
  }

}
