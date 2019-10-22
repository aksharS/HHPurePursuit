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

	public void paint(Graphics canvas) {
		for (int i = 0; i < path1.getSmoothedPoints().length; i++) {
			System.out.println(path1.getSmoothedPoints()[i].getPoint().toString());
			canvas.drawString("" + i, 2 * (int) path1.getSmoothedPoints()[i].getPoint().getX(),2 * (200 - (int) path1.getSmoothedPoints()[i].getPoint().getY()));
		}
		CoordinatePoint robotLocation = new CoordinatePoint(30, 20);
		canvas.drawString("*", 2 * (int) robotLocation.getX(), 2 * (200 - (int) robotLocation.getY()));

		int startIndex = path1.findClosestPointTo(robotLocation);
		WayPoint startWayPoint = path1.getSmoothedPoints()[startIndex];
		WayPoint endWayPoint = path1.getSmoothedPoints()[startIndex + 1];

		System.out.println(path1	.findLookaheadPoint(startWayPoint.getPoint(), endWayPoint.getPoint(), robotLocation, 0.5).toString());
	}
}
