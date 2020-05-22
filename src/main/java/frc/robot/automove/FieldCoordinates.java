/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.automove;

// WPI libraries
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * Class for defining x and y coordinates. Makes it easier to work with. Coordinates are in inches.
 */
public class FieldCoordinates {
    public final double x;
    public final double y;
    public final double angle;

    public FieldCoordinates(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    // Returns a Pose2d for use with the WPI odometry. Units are in meters
    public Pose2d toPose2D() {
        return new Pose2d(new Translation2d( x/39.79, y/39.79), Rotation2d.fromDegrees(angle));
    }

    // Returns a Translation2d for use with the WPI odometry. Units are in meters
    public Translation2d toTranslation2D() {
        return new Translation2d( x/39.79, y/39.79);
    }

}
