package frc.robot.PurePursuit;

import javax.swing.*;
import java.awt.*;

public class PurePursuitTester extends JApplet{
	static WayPoint[] points = {
			new WayPoint(0, 0),
			new WayPoint(0, 10),
			new WayPoint(10, 20),
			new WayPoint(20, 30),
			new WayPoint(30, 35),
			new WayPoint(40, 40),
			new WayPoint(55, 45),
			new WayPoint(65, 50),
			new WayPoint(75, 55),
			new WayPoint(85, 60),
			new WayPoint(90, 68) };
	
	static Path path1 = new Path(points, 0.995, 0.005, 0.1);
	static CoordinatePoint robotLocation = new CoordinatePoint(30, 20);


	public void paint(Graphics canvas) {
		for (int i = 0; i < path1.getSmoothedPoints().length; i++) {
			System.out.println(path1.getSmoothedPoints()[i].getPoint().toString());
			canvas.drawString("" + i, 2 * (int) path1.getSmoothedPoints()[i].getPoint().getX(),2 * (200 - (int) path1.getSmoothedPoints()[i].getPoint().getY()));
			canvas.drawString("*", 2 * (int) robotLocation.getX(), 2 * (200 - (int) robotLocation.getY()));
		}
	}
		
		public static void main(String args[])
		{	
			int startIndex = path1.findClosestPointTo(robotLocation);
			WayPoint startWayPoint = path1.getSmoothedPoints()[startIndex];
			WayPoint endWayPoint = path1.getSmoothedPoints()[startIndex + 1];
			
			double lookAheadDistance = 10.0;
			CoordinatePoint lookAheadPoint= path1	.findLookaheadPoint(startWayPoint.getPoint(), endWayPoint.getPoint(), robotLocation, lookAheadDistance);
			System.out.println(lookAheadPoint.toString());
			
			double robotAngle = 60;
			Vector unitRobotHeadingY = new Vector(Math.cos(Math.toRadians(robotAngle)), Math.sin(Math.toRadians(robotAngle))); //unit vector in the direction of the robot heading
			Vector unitRobotHeadingX = new Vector(Math.sin(Math.toRadians(robotAngle)), -Math.cos(Math.toRadians(robotAngle))); //unit vector in the direction of 90 degrees clockwise from robot heading
	
			Vector l = new Vector(robotLocation, lookAheadPoint); //vector pointing from robot location to the lookahead point
			double x = l.proj(unitRobotHeadingX);
			double y = l.proj(unitRobotHeadingY);
			
			double curvature = 2.0*x/(lookAheadDistance*lookAheadDistance);
			System.out.println(curvature);
	
			if(curvature > 0.0)//lookahead point is to the right of the robot
			{
				System.out.println("right");
			}
			else if(curvature == 0.0)//lookahead point is straight ahead of robot
			{
				System.out.println("straight");
			}
			else if(curvature < 0.0)//lookahead point is to the left of the robot
			{
				System.out.println("left");
			}
		}
	}
}
