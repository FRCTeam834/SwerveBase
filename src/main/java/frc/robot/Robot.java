/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// WPI libraries
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// Subsystems
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.swerve.DriveTrain;
import frc.robot.DriverProfiles.ProfilingManagement;
//import frc.robot.commands.swerve.LetsRoll1Joystick;
//import frc.robot.commands.swerve.LetsRoll2Joysticks;
//import frc.robot.commands.swerve.PullNTSwerveParams;
//import frc.robot.commands.swerve.SaveSwerveParameters;
//import frc.robot.commands.swerve.TestModulePID;
//import frc.robot.commands.swerve.ZeroCanCoders;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  // Subsystems
  public static ProfilingManagement profilingManagement;
  public static NavX navX;
  public static DriveTrain driveTrain;

  // Commands
  //public static LetsRoll2Joysticks letsRoll2Joysticks;
  //public static LetsRoll1Joystick letsRoll1Joystick;
  //public static ZeroCanCoders zeroCanCoders;
  //public static PullNTSwerveParams pullNTSwerveParams;
  //public static TestModulePID testPID;
  //public static SaveSwerveParameters saveSwerveParameters;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Profiling management should be first to avoid errors!
    profilingManagement = new ProfilingManagement();
    navX = new NavX();
    driveTrain = new DriveTrain();

    // Commands
    //letsRoll2Joysticks = new LetsRoll2Joysticks();
    //letsRoll1Joystick = new LetsRoll1Joystick();
    //zeroCanCoders = new ZeroCanCoders();
    //pullNTSwerveParams = new PullNTSwerveParams();
    //testPID = new TestModulePID();
    //saveSwerveParameters = new SaveSwerveParameters();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    navX.resetYaw();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
