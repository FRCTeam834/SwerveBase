// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Parameters;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Turn180 extends CommandBase {

    // Stores the starting angle (so we know when we've reached 180)
    double startingAngle;

    /** Turns the robot around 180 degrees from the starting angle */
    public Turn180() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.driveTrain, Robot.navX);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // Get the angle of the NavX
        startingAngle = Robot.navX.getYaw();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Robot.driveTrain.drive(
                RobotContainer.constrainJoystick(RobotContainer.rightJoystick.getY())
                        * Parameters.driver.currentProfile.maxModVelocity,
                RobotContainer.constrainJoystick(RobotContainer.rightJoystick.getX())
                        * Parameters.driver.currentProfile.maxModVelocity,
                startingAngle > 0
                        ? -1
                        : 1
                                * Parameters.driveTrain.auton.TURN_180_STEER_RATE_PERCENT
                                * Parameters.driver.currentProfile.maxSteerRate,
                false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.driveTrain.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        // Check the sign of the starting angle
        if (startingAngle > 0) {

            // The starting angle is positive, so we need to subtract the yaw from it
            return ((startingAngle - Robot.navX.getYaw()) > 180);
        } else {
            // The starting angle is negative, so we need to subtract it from the yaw
            return ((Robot.navX.getYaw() - startingAngle) > 180);
        }
    }
}
