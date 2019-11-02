package Threads;

import java.util.TimerTask;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.PurePursuit.CoordinatePoint;
import frc.robot.PurePursuit.Path;
import frc.robot.PurePursuit.Vector;
import frc.robot.subsystems.DriveBase;

public class PurePursuitThread extends TimerTask {
	String threadName = null;
	Path path;

	public PurePursuitThread(String name, Path path) {
		threadName = name;
		this.path = path;
	}

	@Override
	public synchronized void run() {
		// TODO Auto-generated method stub
		long startTime = System.nanoTime();
		Robot.m_DriveBase.update();
		CoordinatePoint robotLocation = new CoordinatePoint(RobotMap.x_Location, RobotMap.y_Location);
		if (path.closestPointTo(robotLocation) != path.getSmoothedPoints()[path.getSmoothedPoints().length - 1]) {
			double angle = Math.toRadians(Robot.m_DriveBase.getGyro());
			Vector unitRobotX = new Vector(Math.sin(angle), -Math.cos(angle));
			CoordinatePoint lookAheadPoint = path.findLookaheadPoint(robotLocation, 10.0);
			Vector l = new Vector(robotLocation, lookAheadPoint);
			double x = l.proj(unitRobotX);
			double signedCurvature = (2 * x) / (l.getmagnitude() * l.getmagnitude());
			
			if (Math.abs(signedCurvature) < 0.1) {
                signedCurvature = 0; // If curvature is low enough, have robot go straight
            }

            double targetRobotVelocity = path.closestPointTo(robotLocation).getTargetVelocityAtPoint();
            double targetLeftWheelVelocity = targetRobotVelocity * (2 + signedCurvature * robotTrackWidth) / 2.0;
            double targetRightWheelVelocity = targetRobotVelocity * (2 - signedCurvature * robotTrackWidth) / 2.0;
            double leftOutput = ((targetLeftWheelVelocity - DriveBase.lVelocity) * RobotMap.kVelocityConstant * .5) + (RobotMap.kDriveD * (DriveBase.lAcceleration));
            double rightOutput = ((targetRightWheelVelocity - DriveBase.rVelocity) * RobotMap.kVelocityConstant * .5) + (RobotMap.kDriveD * (DriveBase.rAcceleration));
            Robot.m_DriveBase.driveBaseTank(leftOutput, rightOutput);

			System.out.println("robotLocation: " + robotLocation);
            System.out.println("targetRobotVelocity: " + targetRobotVelocity);
            System.out.println("lookAheadPoint: " + lookAheadPoint);
            System.out.println("unitRobotX: " + unitRobotX);
            System.out.println("l: " + l);
            System.out.println("x: " + x);
            System.out.println("signedCurvature: " + signedCurvature);
            System.out.println("targetLeftWheelVelocity: " + targetLeftWheelVelocity);
            System.out.println("targetRightWheelVelocity: " + targetRightWheelVelocity);
            System.out.println(path.toString());
            System.out.println("Left Output: " + leftOutput);
            System.out.println("Right Output: " + rightOutput);

            long endTime = System.currentTimeMillis();
            System.out.println("That took " + (endTime - startTime) + " milliseconds\n");
		}
	}
}
