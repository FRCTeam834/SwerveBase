/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Data structure for returned arrays is as follows: 
 * - Each column (first value) represents a space to store object values. 
 * The data structure of the rows is
 * as follows: - 0 or -1, with -1 being not used and 0 being used (meaning data
 * is valid and should be considered) 
 * - objectID - a unique ID for each different object that appears
 * - centerX - the x coordinate of the center
 * - centerY - the y coordinate of the center 
 * - endX - the x coordinate of the bottom right corner of the bounding box 
 * - endY - the y coordinate of the bottom right corner of the bounding box 
 * - area - the area of the bounding box
 * - confidence - the confidence level of the neural network that the object is indeed what it is tagged as
 * Camera resolution is 640 x 480 @ 7 fps.
 */

public class EVSNetworkTables extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

  @Override
  public void periodic() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

  }

  public NetworkTable getVisionTable() {

    // Get the NetworkTables instance and return it
    NetworkTable evs = networkTableInstance.getTable("EVS");
    return evs;

  }

  public ArrayList<double[]> getPowerCellArray() {

    // Create a new expandable array for the tables
    ArrayList<double[]> visionArray = new ArrayList<double[]>();

    // Create a new timing data array
    double timingData[] = new double[1];

    // Add the processing time to the timing array
    timingData[0] = getVisionTable().getSubTable("timing").getEntry("processing_time").getDouble(100);

    // Add the timing data to the output array
    visionArray.add(timingData);

    // Loop through the available objects
    for (int objectCount = 0; objectCount < 10; objectCount++) {
      
      // Create the name for the object
      String objectName = "Power_Cell" + objectCount;
      
      // Check if the entry is valid
      if (getVisionTable().getSubTable(objectName).getEntry("inUse").getBoolean(false)){

        // Create a new array for the values from NetworkTables
        double powerCellArray[] = getVisionTable().getSubTable(objectName).getEntry("values").getDoubleArray(new double[7]);
  
        // Add the data for the object to the array
        visionArray.add(powerCellArray);
        
      } else {
 
        // Break the loop if no object is found
        break;
        
      }
    }

    // Return our findings
    return visionArray;

  }

  public ArrayList<double[]> getGoalArray() {

    // Create a new expandable array for the tables
    ArrayList<double[]> visionArray = new ArrayList<double[]>();

    // Create a new timing data array
    double timingData[] = new double[1];

    // Add the processing time to the timing array
    timingData[0] = getVisionTable().getSubTable("timing").getEntry("processing_time").getDouble(100);

    // Add the timing data to the output array
    visionArray.add(timingData);

    // Loop through the available objects
    for (int objectCount = 0; objectCount < 1; objectCount++) {
      
      // Create the name for the object
      String objectName = "Goal" + objectCount;
      
      // Check if the entry is valid
      if (getVisionTable().getSubTable(objectName).getEntry("inUse").getBoolean(false)){

        // Create a new array for the values from NetworkTables
        double goalArray[] = getVisionTable().getSubTable(objectName).getEntry("values").getDoubleArray(new double[7]);
  
        // Add the data for the object to the array
        visionArray.add(goalArray);
        
      } else {
 
        // Break the loop if no object is found
        break;
        
      }
    }

    // Return our findings
    return visionArray;

  }

}