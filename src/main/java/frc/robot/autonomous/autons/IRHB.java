// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.autons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

//This is the auton for the Infinite Recharge at Home Auton Challenge; Part B

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IRHB extends SequentialCommandGroup {
  /** Creates a new IRHB. */
  public IRHB() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    /*
    Starting position: D1
    *detect distance to nearest ball(either d5 or d6) if distance = 0.12m its red path if = 0.15m its blue path*
    *move 0.060m forward(quickly)*

    *spin intake (stop when indexer counts + 1*
    *turn 45 degrees (pi/4 radians) right*
    *move 0.0848528m (quickly) to D5*
    *turn 45 degrees left*
    *move forward 0.030m slowly with intake*
    */


    super();
  }
}
