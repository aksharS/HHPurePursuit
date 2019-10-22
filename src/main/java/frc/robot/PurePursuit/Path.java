package frc.robot.PurePursuit;

import java.util.ArrayList;
import java.util.Collections;

public class Path {
	private WayPoint[] unsmoothedPoints;
	private WayPoint[] smoothedPoints;
	private double a;
	private double b;
	private double tolerance;
	private int idxOfLastClosestPoint = 0;

	public Path(WayPoint[] unsmoothedPoints, double a, double b, double tolerance) {
		this.setunsmoothedPoints(unsmoothedPoints);
		this.a = a;
		this.b = b;
		this.tolerance = tolerance;

		WayPoint[] smoothedPoints = unsmoothedPoints.clone();
		double change = tolerance;
		while (change >= tolerance) {
			change = 0.0;
			for (int i = 1; i < unsmoothedPoints.length - 1; i++) {
				Vector previousPointSmooth = new Vector(smoothedPoints[i - 1].getPoint());
				Vector currentPointSmooth = new Vector(smoothedPoints[i].getPoint());
				Vector nextPointSmooth = new Vector(smoothedPoints[i + 1].getPoint());
				Vector currentPointUnsmooth = new Vector(smoothedPoints[i].getPoint());

				double temp1 = smoothedPoints[i].getPoint().getX();
				double temp2 = smoothedPoints[i].getPoint().getY();
				smoothedPoints[i].getPoint()
						.setCoordinatePoint(currentPointSmooth
								.Add(currentPointUnsmooth.Subtract(currentPointSmooth).ScalarMultiply(a)
										.Add(previousPointSmooth.Add(nextPointSmooth)
												.Subtract(currentPointSmooth.ScalarMultiply(2.0)).ScalarMultiply(b)))
								.toCoordinatePoint());
				change += Math.abs(temp1 - smoothedPoints[i].getPoint().getX());
				change += Math.abs(temp2 - smoothedPoints[i].getPoint().getY());
			}
		}
		this.setSmoothedPoints(smoothedPoints);

		smoothedPoints[0].setCurvatureAtPoint(0.0);
		smoothedPoints[0].setDistanceAlongPathAtPoint(0.0);
		for (int i = 0; i < smoothedPoints.length; i++) {
			smoothedPoints[i].setTargetVelocityAtPoint(10); // inches per second
			if (i != 0) {
				smoothedPoints[i].setDistanceAlongPathAtPoint(
						smoothedPoints[i-1].getPoint().distanceTo(smoothedPoints[i].getPoint()) + smoothedPoints[i - 1].getDistanceAlongPathAtPoint());
				if (i != smoothedPoints.length - 1) {
					smoothedPoints[i].setCurvatureAtPoint(smoothedPoints[i].getPoint().getCurvatureFromThreePoints(smoothedPoints[i - 1].getPoint(), smoothedPoints[i + 1].getPoint()));
				}
			}
		}
		smoothedPoints[smoothedPoints.length - 1].setCurvatureAtPoint(0.0);
	}

	public WayPoint[] getSmoothedPoints() {
		return smoothedPoints;
	}

	public void setSmoothedPoints(WayPoint[] smoothedPoints) {
		this.smoothedPoints = smoothedPoints;
	}

	public WayPoint[] getunsmoothedPoints() {
		return unsmoothedPoints;
	}

	public void setunsmoothedPoints(WayPoint[] unsmoothedPoints) {
		this.unsmoothedPoints = unsmoothedPoints;
	}

	/**
	 *
	 * @param path
	 *            object (contains an array of smoothed WayPoints)
	 * @param robotLocation
	 *            current robot coordinate position
	 * @return the index of the WayPoint on the path that is closest to the robot
	 */
	public int findClosestPointTo(CoordinatePoint robotLocation) {
		ArrayList<Double> distances = new ArrayList<Double>();
		for (int i = this.idxOfLastClosestPoint; i < this.smoothedPoints.length; i++) {
			distances.add(this.smoothedPoints[i].getPoint().distanceTo(robotLocation));
		}
		return distances.indexOf(Collections.min(distances));
	}

	/**
	 *
	 * @param startPointofSegment
	 *            starting point of the line segment
	 * @param endPointofSegment
	 *            end point of the line segment
	 * @param robotLocation
	 *            center point of the circle (Robot Location) as a
	 *            CoordinatePoint with radius of lookahead distance
	 * @param lookaheadDistance
	 *            radius of the circle (lookahead distance)
	 * @return Coordinate Point on the line segment (defined by startPointofSegment and endPointofSegment) and is lookaheadDistance away from the robot location
	 */
	public CoordinatePoint findLookaheadPoint(CoordinatePoint startPointofSegment, CoordinatePoint endPointofSegment,
			CoordinatePoint robotLocation, double lookaheadDistance) {
		Vector d = new Vector(startPointofSegment, endPointofSegment);
		Vector f = new Vector(robotLocation, startPointofSegment);

		double a = d.Dot(d);
		double b = 2 * (f.Dot(d));
		double c = f.Dot(f) - (lookaheadDistance * lookaheadDistance);
		double discriminant = b * b - 4 * a * c;
		double t = 0;

		if (discriminant < 0) {
			System.out.println("No Intersection");
		} else {
			discriminant = Math.sqrt(discriminant);
			double t1 = (-b - discriminant) / (2 * a);
			double t2 = (-b + discriminant) / (2 * a);

			if (t1 >= 0 && t1 <= 1) {
				t = t1;
			} else if (t2 >= 0 && t2 <= 1) {
				t = t2;
			} else {
				System.out.println("No Intersection");
			}
		}
		d=d.ScalarMultiply(t);
		return new CoordinatePoint((startPointofSegment.getX() + d.getXComponent()),
				(startPointofSegment.getY() + d.getYComponent()));
	}   
}
