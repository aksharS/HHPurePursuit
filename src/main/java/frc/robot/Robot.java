/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.PursuitDrive;
import frc.robot.commands.TimedDrive;
import frc.robot.commands.autoDrive;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI m_oi;
  public static DriveBase m_DriveBase = new DriveBase(1);
  
  Command autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_DriveBase.resetLeftEncoder();
    m_DriveBase.resetRightEncoder();
    m_oi = new OI();
    m_DriveBase.reset();
    m_chooser.setDefaultOption("PurePursuit Rocket", new PursuitDrive(Paths.getRightHabRocket(), RobotMap.trackWidth + 1, 10));
    m_chooser.addOption("PurePursuit Straight", new PursuitDrive(Paths.getStraightPath(), RobotMap.trackWidth + 1, 10));
    m_chooser.addOption("Six Seconds, Half Power", new TimedDrive(6, .5));
    m_chooser.addOption("Go Forward 25 inches", new autoDrive(25));
    // m_chooser.addOption("Pathfinder Test", new PathDrive(Paths.straightPoints, Paths.configSlow));
    SmartDashboard.putData("Auto mode", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_DriveBase.update();
    SmartDashboard.putNumber("Gyro value",m_DriveBase.getGyro());
    SmartDashboard.putNumber("Left Ticks", m_DriveBase.getLeftTicks());
    SmartDashboard.putNumber("Right Ticks", m_DriveBase.getRightTicks());
    SmartDashboard.putNumber("X Location", RobotMap.x_Location);
    SmartDashboard.putNumber("Y Location", RobotMap.y_Location);
    SmartDashboard.putNumber("Left Velocity", m_DriveBase.lVelocity);
    SmartDashboard.putNumber("Right Velocity", m_DriveBase.rVelocity);

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    autonomousCommand = m_chooser.getSelected();
    Robot.m_DriveBase.resetLeftEncoder();
    Robot.m_DriveBase.resetRightEncoder();
    Robot.m_DriveBase.resetGyro();
    

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.start();
      System.out.println("Started Auto");
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    //m_DriveBase.arcadeDrive(1, 0);
  }

  @Override
  public void teleopInit() {
    System.out.println("Teleop enabled");
    RobotMap.x_Location = 0;
    RobotMap.y_Location = 0;
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    //System.out.printf("Left Encoder: %d Right Encoder: %d \n", m_DriveBase.getLeftTicks(), m_DriveBase.getRightTicks());
  }
}
