package frc.robot;

import frc.robot.PurePursuit.Path;
import frc.robot.PurePursuit.WayPoint;

public class Paths {
    public static final WayPoint[] rightHabRightRocket = {
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
            new WayPoint(90, 68),
            new WayPoint(95, 76)
    };

    public static final WayPoint[] straightTest = {
            new WayPoint(0,5),
            new WayPoint(0,10),
            new WayPoint(0,15),
            new WayPoint(0,20),
            new WayPoint(0,25),
            new WayPoint(0,30),
            new WayPoint(0,35),
            new WayPoint(0,40),
            new WayPoint(0,45),
            new WayPoint(0,50),
            new WayPoint(0,55),
            new WayPoint(0,60)
    };

    public static Path getStraightPath(){
        return new Path(straightTest, 1, 0 , 0.01, 10, .1);
    }

    public static Path getRightHabRocket() {return new Path(rightHabRightRocket, 1, 0 , 0.01, 10, .1); }
}
