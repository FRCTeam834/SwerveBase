// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Parameters;
import frc.robot.Robot;

public class Drive extends CommandBase {
    /** Creates a new Drive. */
    Timer timer = new Timer();

    double time = 0;
    double speed = 0;

    public Drive(double time) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.driveTrain);
        this.time = time;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.driveTrain.straightenModules();
        timer.reset();
        timer.start();
        if (time == Parameters.driveTrain.auton.TIME_OFF_LINE) {
            speed = Parameters.driveTrain.auton.DRIVE_SPEED;
        } else {
            speed = Parameters.driveTrain.auton.LINE_UP_SPEED;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Robot.driveTrain.drive(speed, 0, 0, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.driveTrain.stopModules();
        timer.reset();
        timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(time);
    }
}
