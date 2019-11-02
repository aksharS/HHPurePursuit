/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.Drive;
/**
 * Add your docs here.
 */
public class DriveBase extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  WPI_TalonSRX frontLeft = new WPI_TalonSRX(RobotMap.FrontLeftTalonPort);
  WPI_TalonSRX frontRight = new WPI_TalonSRX(RobotMap.FrontRightTalonPort);
  WPI_TalonSRX backLeft = new WPI_TalonSRX(RobotMap.BackLeftTalonPort);
  WPI_TalonSRX backRight = new WPI_TalonSRX(RobotMap.BackRightTalonPort);

  private double lastTime = Timer.getFPGATimestamp();
  private static double lastLeftVelocity = 0.0;
  private static double lastRightVelocity = 0.0;
  private static double lastLeftEncoder = 0.0;
  private static double lastRightEncoder = 0.0;
  public static double lVelocity, rVelocity;
  public static double lAcceleration, rAcceleration;

  Encoder leftEncoder = new Encoder(0, 1);
  Encoder rightEncoder = new Encoder(2, 3);

  ADXRS450_Gyro gyro = new ADXRS450_Gyro(Port.kOnboardCS0);

  SpeedControllerGroup leftMotors = new SpeedControllerGroup(frontLeft, backLeft);
  SpeedControllerGroup rightMotors = new SpeedControllerGroup(frontRight, backRight);

  DifferentialDrive dDrive = new DifferentialDrive(leftMotors, rightMotors);

  private double sensitivity = 1;
  private double deadzone = .05;

  private double distance = 0, oldLeftTicks = 0, oldRightTicks = 0;

  public void update(){
    double currentTime = Timer.getFPGATimestamp();
    double dt = currentTime - lastTime;

    // Update velocity
    double dl = getLeftTicks() - lastLeftEncoder;
    double dr = getRightTicks() - lastRightEncoder;
    lVelocity = dl/dt;
    rVelocity = dr/dt;

    double dlv = lVelocity - lastLeftVelocity;
    double drv = rVelocity - lastRightVelocity;

    // Update acceleration
    lAcceleration = dlv/dt;
    rAcceleration = drv/dt;

    lastTime = currentTime;
    lastLeftEncoder = getLeftTicks();
    lastRightEncoder = getRightTicks();
    lastLeftVelocity = lVelocity;
    lastRightVelocity = rVelocity;
    double deltaLeft = Robot.m_DriveBase.getLeftTicks() - oldLeftTicks;
    double deltaRight = Robot.m_DriveBase.getRightTicks() - oldRightTicks;
    distance = (deltaLeft + deltaRight)/2.0;
    //System.out.println("Lwft Gyro: " + Math.cos(Math.toRadians(m_DriveBase.getGyro())) + "Right Gyro" + Math.sin(Math.toRadians(m_DriveBase.getGyro())) + " Left Ticks: " + m_DriveBase.getLeftTicks() + " Right Ticks: " + m_DriveBase.getRightTicks());
    RobotMap.x_Location += distance * Math.cos(Math.toRadians(Robot.m_DriveBase.getGyro()));
    RobotMap.y_Location += distance * Math.sin(Math.toRadians(Robot.m_DriveBase.getGyro()));
    //System.out.printf("Distance: %f X Location: %f Y Location: %f \n", distance, RobotMap.x_Location, RobotMap.y_Location);
    oldLeftTicks = Robot.m_DriveBase.getLeftTicks();
    oldRightTicks = Robot.m_DriveBase.getRightTicks();
  }

  public void reset() {
    lastTime = Timer.getFPGATimestamp();
    resetLeftEncoder();
    resetRightEncoder();
    lastLeftEncoder = 0.0;
    lastRightEncoder = 0.0;
    lVelocity = rVelocity = 0.0;
    lAcceleration = rAcceleration = 0.0;
  }

  public DriveBase(double s) {
    sensitivity = s;
    leftEncoder.reset();
    rightEncoder.reset();
    frontLeft.setSafetyEnabled(false);
    frontRight.setSafetyEnabled(false);
    backLeft.setSafetyEnabled(false);
    backRight.setSafetyEnabled(false);
  }

  public void arcadeDrive(double moveSpeed, double rotateSpeed){
    // System.out.printf("Sending Arcade Drive Command with %f moveSpeed and %f turnSpeed \n", moveSpeed, rotateSpeed);
    if (Math.abs(moveSpeed) > deadzone || Math.abs(rotateSpeed) > deadzone) {
      dDrive.arcadeDrive(moveSpeed * sensitivity, -rotateSpeed * sensitivity);
    }
  }

  public double getLeftTicks(){
    //return -leftEncoder.get();
     return (leftEncoder.get() / (RobotMap.ticksPerRevolution/RobotMap.wheelCircumference));
  }

  public double getRightTicks(){
    // return rightEncoder.get();
    return (-rightEncoder.get() /  (RobotMap.ticksPerRevolution/RobotMap.wheelCircumference));
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new Drive());
  }

public void resetLeftEncoder() {
  leftEncoder.reset();
}

public void resetRightEncoder() {
  rightEncoder.reset();
}

public double getGyro() {
  return 90 - (gyro.getAngle() % 360);
}

public void resetGyro() {
  gyro.reset();
}

public void driveBaseTank(double d, double e) {
  leftMotors.set(d);
  rightMotors.set(-e);
}

public void setAllCoast(){
  frontLeft.setNeutralMode(NeutralMode.Coast);
  frontRight.setNeutralMode(NeutralMode.Coast);
  backLeft.setNeutralMode(NeutralMode.Coast);
  backRight.setNeutralMode(NeutralMode.Coast);
}

public void setAllBrake(){
  frontLeft.setNeutralMode(NeutralMode.Brake);
  frontRight.setNeutralMode(NeutralMode.Brake);
  backLeft.setNeutralMode(NeutralMode.Brake);
  backRight.setNeutralMode(NeutralMode.Brake);
}

}
