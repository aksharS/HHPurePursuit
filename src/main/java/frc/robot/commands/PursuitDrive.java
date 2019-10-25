package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.PurePursuit.CoordinatePoint;
import frc.robot.PurePursuit.Path;
import frc.robot.PurePursuit.Vector;
import frc.robot.Robot;
import frc.robot.RobotMap;


public class PursuitDrive extends Command {
    private Path path;
    private double robotTrackWidth;
    public PursuitDrive(Path path, double robotTrackWidth, double timeout) {
        this.path=path;
        this.robotTrackWidth=robotTrackWidth;
        requires(Robot.m_DriveBase);
        setTimeout(timeout);
    }


    /**
     * The initialize method is called just before the first time
     * this Command is run after being started.
     */
    @Override
    protected void initialize(){


    }


    /**
     * The execute method is called repeatedly when this Command is
     * scheduled to run until this Command either finishes or is canceled.
     */
    @Override
    protected void execute() {
        while(robotPos != ){
            double xpos = RobotMap.x_Location;
            double ypos = RobotMap.y_Location;
            double angle = Math.toRadians(Robot.m_DriveBase.getGyro());

            CoordinatePoint robotLocation = new CoordinatePoint(xpos, ypos);
            double targetRobotVelocity = path.closestPointTo(robotLocation).getTargetVelocityAtPoint();
            CoordinatePoint lookAheadPoint = path.findLookaheadPoint(robotLocation, 10.0);

            Vector unitRobotY = new Vector(Math.cos(angle),Math.sin(angle));
            Vector unitRobotX = new Vector(Math.sin(angle),-Math.cos(angle));
            Vector l = new Vector(robotLocation, lookAheadPoint);

            double x = l.proj(unitRobotX);
            double y = l.proj(unitRobotY);

            double signedCurvature = (2*x)/(l.getmagnitude()*l.getmagnitude());
            double curvature = Math.abs(signedCurvature);

            double targetLeftWheelVelocity=targetRobotVelocity*(2+curvature*robotTrackWidth)/2.0;
            double targetRightWheelVelocity=targetRobotVelocity*(2-curvature*robotTrackWidth)/2.0;
            System.out.println("robotLocation: "+robotLocation);
            System.out.println("targetRobotVelocity: "+targetRobotVelocity);
            System.out.println("lookAheadPoint: "+lookAheadPoint);
            System.out.println("unitRobotY: "+unitRobotY);
            System.out.println("unitRobotX: "+unitRobotX);
            System.out.println("l: "+l);
            System.out.println("x: "+x);
            System.out.println("y: "+y);
            System.out.println("signedCurvature: "+signedCurvature);
            System.out.println("curvature: "+curvature);
            System.out.println("targetLeftWheelVelocity: "+targetLeftWheelVelocity);
            System.out.println("targetRightWheelVelocity: "+targetRightWheelVelocity);
        }
    }


    /**
     * <p>
     * Returns whether this command is finished. If it is, then the command will be removed and
     * {@link #end()} will be called.
     * </p><p>
     * It may be useful for a team to reference the {@link #isTimedOut()}
     * method for time-sensitive commands.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Returning true will result in the
     * command executing once and finishing immediately. It is recommended to use
     * {@link edu.wpi.first.wpilibj.command.InstantCommand} (added in 2017) for this.
     * </p>
     *
     * @return whether this command is finished.
     * @see Command#isTimedOut() isTimedOut()
     */
    @Override
    protected boolean isFinished() {
        return isTimedOut();
    }


    /**
     * Called once when the command ended peacefully; that is it is called once
     * after {@link #isFinished()} returns true. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the
     * command.
     */
    @Override
    protected void end() {

    }


    /**
     * <p>
     * Called when the command ends because somebody called {@link #cancel()} or
     * another command shared the same requirements as this one, and booted it out. For example,
     * it is called when another command which requires one or more of the same
     * subsystems is scheduled to run.
     * </p><p>
     * This is where you may want to wrap up loose ends, like shutting off a motor that was being
     * used in the command.
     * </p><p>
     * Generally, it is useful to simply call the {@link #end()} method within this
     * method, as done here.
     * </p>
     */
    @Override
    protected void interrupted() {
        super.interrupted();
    }
}