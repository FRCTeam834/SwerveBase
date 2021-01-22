/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Parameters;
import frc.robot.Robot;
import frc.robot.swerve.DriveTrain;

public class LetsRoll extends CommandBase {

  DriveTrain driveTrain;
  Joystick left = new Joystick(0);
  Joystick right = new Joystick(1);

  public LetsRoll() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
    driveTrain = Robot.driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Get all of the current joystick inputs
    double leftX = left.getX();
    double leftY = left.getY();
    double rightX = right.getX();

    // Do we need to move?
    boolean move = false;

    // Check to see if joysticks are out of range. If out, we need to move, otherwise set the sticks to zero.
    // Left X
    if (leftX < Parameters.currentDriverProfile.JOYSTICK_DEADZONE) {
      leftX = 0;
    }
    else {
      move = true;
    }

    // Left Y
    if (leftY < Parameters.currentDriverProfile.JOYSTICK_DEADZONE) {
      leftY = 0;
    }
    else {
      move = true;
    }

    // Right X
    if (rightX < Parameters.currentDriverProfile.JOYSTICK_DEADZONE) {
      rightX = 0;
    }
    else {
      move = true;
    }
    
    // If any of the sticks are out of range, then we need to move. Otherwise, get to it
    if (move) {

      // X and Y are flipped because WPI dumb
      driveTrain.drive((left.getY() * Parameters.currentDriverProfile.MAX_SPEED), (left.getX() * Parameters.currentDriverProfile.MAX_SPEED), Math.toRadians(right.getX() * Parameters.currentDriverProfile.MAX_TURN_SPEED), Parameters.currentDriverProfile.FIELD_CENTRIC);
    }
    else if (Parameters.currentDriverProfile.LOCKEM_UP) {
      driveTrain.lockemUp();
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.lockemUp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
