package org.firstinspires.ftc.teamcode.maths;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Driveable;
import org.firstinspires.ftc.teamcode.utility.ElapsedTimeW;
import org.opencv.core.Point;

public class GoToPoint {
    private final Driveable driver;
    private final Telemetry telemetry;
    private final FtcDashboard dashboard;
    private boolean isDone;
    private double distanceNow, headingError;
    private final ElapsedTimeW profileTime = new ElapsedTimeW();
    private final PID headingPID = new PID(6,0,5,0, 0.1);
    private final PID xPID = new PID(1,0,0,1, 0.1);
    private final PID yPID = new PID(1,0,0,1, 0.1);
    private MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0, 0, 0), new MotionState(1, 0, 0), 2, 3,4);

    public GoToPoint(Driveable driver, Telemetry telemetry, FtcDashboard dashboard){
        this.driver=driver;
        this.telemetry = telemetry;
        this.dashboard = dashboard;
    }

    /**
     * @param pose current pose of the robot
     * @param desiredPose target pose of the robot
     * @param startPose robot pose when starting the movement
     * @param update if the target should change
     */

    public void driveToPoint(Pose2d pose,Pose2d desiredPose,Pose2d startPose,boolean update){
        distanceNow = Math.abs(Math.hypot(desiredPose.position.x-pose.position.x,desiredPose.position.y-pose.position.y));
        double distanceAtStart = Math.abs(Math.hypot(desiredPose.position.x-startPose.position.x,desiredPose.position.y-startPose.position.y));
        //distance starts at 0 and goes to the distance from start as the robot gets closer
        double distance = distanceAtStart-distanceNow;
        //angle from start to finish
        double angleToEndPoint = Math.atan2(desiredPose.position.y-startPose.position.y,desiredPose.position.x-startPose.position.x);
        //if the target position has changed, then create a new motion profile and reset the profile timer
        if(update){
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0, 0, 0), new MotionState(distanceAtStart, 0, 0), 30, 50, 70);
            profileTime.reset();
        }
        MotionState state = profile.get(profileTime.seconds());
        double stateOut = state.getX();
        //create a point from the motion profile output, which is normally just a distance value
        Point statePoint = new Point(startPose.position.x+(stateOut * Math.cos(angleToEndPoint)),startPose.position.y+(stateOut * Math.sin(angleToEndPoint)));
        //distance from current position to the target point, determined by the motion profile
        double distanceToState = Math.abs(Math.hypot(statePoint.x-startPose.position.x,statePoint.y-startPose.position.y));
        //use pid on x and y position to move towards the target point determined by the state
        //distanceToState * sin or cos of the angle to end point will give the x or y component of the distance vector, which ensures that x and y arrive at the same time. pose-startpos is the current state
        double xOut = xPID.pidOut(distanceToState * Math.cos(angleToEndPoint) - (pose.position.x - startPose.position.x));
        double yOut = yPID.pidOut(distanceToState * Math.sin(angleToEndPoint) - (pose.position.y - startPose.position.y));
        headingError = AngleUnit.normalizeRadians(desiredPose.heading.log()-pose.heading.log());
        double headingOut = headingPID.pidOut(headingError);
        //feed the pid output into swerve kinematics and draw the robot on FTCdash field
        driver.drive(-yOut,-xOut,-headingOut);
        //drawField(pose,desiredPose,startPose,dashboard);

        isDone = profile.duration() < profileTime.seconds();

        telemetry.addData("distance: ",distance);
        telemetry.addData("timer,",profileTime.seconds());
        telemetry.addData("distanceAtStart: ",distanceAtStart);
        telemetry.addData("distanceToProfile",distanceToState);
        telemetry.addData("distanceNow: ",distanceNow);
        telemetry.addData("angleToEndPoint: ", angleToEndPoint);
        telemetry.addData("stateOut: ", stateOut);
        telemetry.addData("xOut: ", xOut);
        telemetry.addData("yOut: ", yOut);
        telemetry.addData("statepointX",statePoint.x);
        telemetry.addData("statepointY",statePoint.y);
        telemetry.addData("xerror",distanceToState*Math.cos(angleToEndPoint)-(pose.position.x-startPose.position.x));
        telemetry.addData("yerror",distanceToState*Math.sin(angleToEndPoint)-(pose.position.y-startPose.position.y));
    }

    public void driveToPointWithoutProfile(Pose2d pose,Pose2d desiredPose) {
        distanceNow = Math.abs(Math.hypot(desiredPose.position.x-pose.position.x,desiredPose.position.y-pose.position.y));
        //angle from start to finish
        double angleToEndPoint = Math.atan2(desiredPose.position.y-pose.position.y,desiredPose.position.y-pose.position.x);
        //create a point from the motion profile output, which is normally just a distance value
        double distanceToState = Math.abs(Math.hypot(desiredPose.position.x-pose.position.x,desiredPose.position.y-pose.position.y));
        //use pid on x and y position to move towards the target point determined by the state
        //distanceToState * sin or cos of the angle to end point will give the x or y component of the distance vector, which ensures that x and y arrive at the same time. pose-startpos is the current state
        double xOut = xPID.pidOut(distanceToState * Math.cos(angleToEndPoint));
        double yOut = yPID.pidOut(distanceToState * Math.sin(angleToEndPoint));
        headingError = AngleUnit.normalizeRadians(desiredPose.heading.log()-pose.heading.log());
        double headingOut = headingPID.pidOut(headingError);
        //feed the pid output into swerve kinematics and draw the robot on FTCdash field
        driver.drive(yOut,xOut,-headingOut);
        //drawField(pose,desiredPose,startPose,dashboard);

        isDone = profile.duration() < profileTime.seconds();


        telemetry.addData("distanceToProfile",distanceToState);
        telemetry.addData("distanceNow: ",distanceNow);
        telemetry.addData("angleToEndPoint: ", angleToEndPoint);
        telemetry.addData("xOut: ", xOut);
        telemetry.addData("yOut: ", yOut);
        telemetry.addData("xerror",distanceToState*Math.cos(angleToEndPoint));
        telemetry.addData("yerror",distanceToState*Math.sin(angleToEndPoint));
    }

    public boolean isTimeDone(){
        return isDone;
    }
    public boolean isDone() { return distanceNow < 1 || isDone; }
    public boolean isPositionDone() { return distanceNow < 1; }

    public double getHeadingError() {
        return headingError;
    }

    public void setPIDCoeffs(double Kp, double Kd,double Ki, double Kf, double limit){
        xPID.setPIDgains(Kp, Kd, Ki, Kf, limit);
        yPID.setPIDgains(Kp, Kd, Ki, Kf, limit);
    }

    public void setVPIDCoeffs(double Kv, double Ka, double Ks) {
        xPID.setFFgains(Kv, Ka, Ks);
        yPID.setFFgains(Kv, Ka, Ks);
    }

    public void setHeadingPIDcoeffs(double Kp,double Kd, double Ki, double Kf, double limit){
        headingPID.setPIDgains(Kp, Kd, Ki, Kf, limit);
    }

    public void setProfileConstraints(double maxVel, double maxAccel, double maxJerk){
        //this.maxVel = maxVel;
        //this.maxAccel = maxAccel;
        //this.maxJerk = maxJerk;
    }


    //draw the robot on the FTCdash field (copied from the roadrunner quickstart)
    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.position.x, pose.position.y, 9);
        Vector2d v = pose.heading.vec().times(9);
        double x1 = pose.position.x + v.x / 2, y1 = pose.position.y + v.y / 2;
        double x2 = pose.position.x + v.x, y2 = pose.position.y + v.y;
        canvas.strokeLine(x1, y1, x2, y2);
    }

    //draw all the robots on the field and send to the dashboard
    public void drawField(Pose2d pose, Pose2d desiredPose, Pose2d startPose, FtcDashboard dash){
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.setStroke("#3F51B5");
        drawRobot(fieldOverlay, pose);
        fieldOverlay.setStroke("#51B53F");
        drawRobot(fieldOverlay,desiredPose);
        fieldOverlay.setStroke("#B53F51");
        drawRobot(fieldOverlay,startPose);

        packet.put("x", pose.position.x);
        packet.put("y", pose.position.y);
        packet.put("heading (deg)", Math.toDegrees(pose.heading.log()));

        dash.sendTelemetryPacket(packet);

    }
}
