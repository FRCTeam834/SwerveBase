// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * @author Christian Piper (@CAP1Sup)
 * @since 4/8/21
 */
package frc.robot.commands.swerve;

// Imports
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;
import frc.robot.RobotContainer;

public class TestModulePositioning extends CommandBase {
    /** Creates a new TestModulePositioning. */

    // Joystick value array, in form (LX, LY, RX, RY)
    double[] joystickValues = new double[4];

    public TestModulePositioning() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.driveTrain.frontLeft.setDesiredAngle(0);
        Robot.driveTrain.frontRight.setDesiredAngle(0);
        Robot.driveTrain.backLeft.setDesiredAngle(0);
        Robot.driveTrain.backRight.setDesiredAngle(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Get the values of the joysticks
        joystickValues = RobotContainer.getJoystickValues();

        // Multiply each of the elements by the angle value
        for (int index = 0; index < joystickValues.length; index++) {
            joystickValues[index] = joystickValues[index] * 360;
        }

        // Move the modules to those positions
        Robot.driveTrain.frontLeft.setDesiredAngle(joystickValues[0]);
        Robot.driveTrain.frontRight.setDesiredAngle(joystickValues[1]);
        Robot.driveTrain.backLeft.setDesiredAngle(joystickValues[2]);
        Robot.driveTrain.backRight.setDesiredAngle(joystickValues[3]);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.driveTrain.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
