package frc.robot.PurePursuit;

public class WayPoint {

	private CoordinatePoint point;
	private double curvatureAtPoint;
	private double TargetVelocityAtPoint;
	private double DistanceAlongPathAtPoint;
	
	public WayPoint(CoordinatePoint p1)
	{
		this.point = p1;
	}
	
	public WayPoint(double x, double y)
	{
		this.point = new CoordinatePoint(x,y);
	}

	public CoordinatePoint getPoint() {
		return point;
	}

	public void setPoint(CoordinatePoint point) {
		this.point = point;
	}

	public double getCurvatureAtPoint() {
		return curvatureAtPoint;
	}

	public void setCurvatureAtPoint(double curvatureAtPoint) {
		this.curvatureAtPoint = curvatureAtPoint;
	}

	public double getTargetVelocityAtPoint() {
		return TargetVelocityAtPoint;
	}

	public void setTargetVelocityAtPoint(double targetVelocityAtPoint) {
		TargetVelocityAtPoint = targetVelocityAtPoint;
	}

	public double getDistanceAlongPathAtPoint() {
		return DistanceAlongPathAtPoint;
	}

	public void setDistanceAlongPathAtPoint(double distanceAlongPathAtPoint) {
		DistanceAlongPathAtPoint = distanceAlongPathAtPoint;
	}
	
	public String toString()
	{
		return ("Point:" + point.toString() + " CurvatureAtPoint:" + curvatureAtPoint + " TargetVelocityAtPoint:"
				+ TargetVelocityAtPoint + " DistanceAlongPathAtPoint:" + DistanceAlongPathAtPoint);
	}
}
