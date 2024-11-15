package org.firstinspires.ftc.teamcode.utility;


import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;

import org.firstinspires.ftc.teamcode.maths.ConstantsForPID;
import org.firstinspires.ftc.teamcode.maths.GainScheduledPID;
import org.firstinspires.ftc.teamcode.maths.PID;

public class RunMotionProfile {

    private double lastTarget;
    private double maxVel, maxAccel, maxJerk;
    private MotionState motionState;
    private final PID PID;
    private final ElapsedTime timer = new ElapsedTime();
    private MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0,0,0),new MotionState(1,0,0),1,1,1);

    public RunMotionProfile(double maxVel, double maxAccel, double maxJerk, ConstantsForPID constants){
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxJerk = maxJerk;

        PID = new PID(constants);
    }

    public void setMotionConstraints(double maxVel, double maxAccel, double maxJerk){
        this.maxVel=maxVel;
        this.maxAccel=maxAccel;
        this.maxJerk=maxJerk;
    }

    public double profiledMovement(double target, double state){
        if (lastTarget != target) {
            lastTarget = target;
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(state, 0, 0), new MotionState(target, 0, 0), maxVel, maxAccel,maxJerk);
            timer.reset();
        }
        else{ lastTarget = target; }
        motionState = profile.get(timer.seconds());
        return PID.pidOut(motionState.getX(), state);
    }

    public double profiledPivotMovement(double target, double state){
        if (lastTarget != target) {
            lastTarget = target;
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(state, 0, 0), new MotionState(target, 0, 0), maxVel, maxAccel,maxJerk);
            timer.reset();
        }
        else{ lastTarget = target; }
        motionState = profile.get(timer.seconds());
        return PID.pidPivotOut(motionState.getX(), state);
    }


    public double profiledServoMovement(double target, double state){
        if (lastTarget != target) {
            lastTarget = target;
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(state, 0, 0), new MotionState(target, 0, 0), maxVel, maxAccel,maxJerk);
            timer.reset();
        }
        else{ lastTarget = target; }
        motionState = profile.get(timer.seconds());
        return motionState.getX();
    }

    public double getMotionTarget(){
        if (motionState == null) {
            return 0;
        }
        return motionState.getX();
    }

    public double getProfileDuration() { return profile.duration(); }

    public double getCurrentTime() { return timer.seconds(); }
}
