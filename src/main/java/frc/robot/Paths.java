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
            new WayPoint(90, 68)
    };

    public static final WayPoint[] straightTest = {
            new WayPoint(5,0),
            new WayPoint(10,0),
            new WayPoint(15,0),
            new WayPoint(20,0),
            new WayPoint(25,0),
            new WayPoint(30,0),
            new WayPoint(35,0),
            new WayPoint(40,0),
            new WayPoint(45,0),
            new WayPoint(50,0),
            new WayPoint(55,0),
            new WayPoint(60,0),
    };

    public static Path getStraightPath(){
        return new Path(straightTest, 1, 0 , 0.01, 10, .1);
    }
}
