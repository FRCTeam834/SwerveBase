// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;

public class DriveDistanceCheap extends CommandBase {
    /** Creates a new DriveTime. */
    double time = 0;

    double linVel = 0;
    Timer timer = new Timer();

    /**
     * @param distance distance to travel, direction agnostic
     * @param linVel speed to move at, direction dependent (+/-)
     */
    public DriveDistanceCheap(double distance, double linVel) {
        addRequirements(Robot.driveTrain);
        time = Math.abs(distance / linVel);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.driveTrain.setDesiredAngles(0, 0, 0, 0);
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Robot.driveTrain.drive(-5, 0, 0, false);
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
