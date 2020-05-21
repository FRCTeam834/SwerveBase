/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.automove;

/**
 * Class for defining x and y coordinates. Makes it easier to work with. Coordinates are in inches.
 */
public class FieldCoordinates {
    public final double x;
    public final double y;

    public FieldCoordinates(double x, double y) {
        this.x = x;
        this.y = y;
    }

}
