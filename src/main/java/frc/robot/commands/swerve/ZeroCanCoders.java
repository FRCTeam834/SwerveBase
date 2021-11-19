// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * @author Christian Piper (@CAP1Sup)
 * @since 3/6/21
 */
package frc.robot.commands.swerve;

// Imports
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroCanCoders extends InstantCommand {
  public ZeroCanCoders() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Zero each of the modules
    Robot.driveTrain.zeroEncoders();
    System.out.println("All modules zeroed");
  }
}
