package org.firstinspires.ftc.teamcode.Trajectories;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Test {

    public static final Pose startPose = new Pose(32.000, 135.000, Math.toRadians(270));
    private  final Pose shootPose = new Pose(55.000, 85.000, Math.toRadians(180));
    private  final Pose  ballPose1 = new Pose(20.000, 60.000, Math.toRadians(180));
    private  final Pose  ballPose2 = new Pose(20.000, 85.000, Math.toRadians(180));
    private  final Pose  gatePose = new Pose(14.000, 60.000, Math.toRadians(150));

    public  final Pose control1 = new Pose(67.000,63.000);
    public  final Pose control2 = new Pose(44.000,66.000);
    TelemetryManager telemetry;
    public  PathChain StartShootPos, ShootBall1Pos, Ball1ShootPos,ShootBall2Pos ,Ball2ShootPos, ShootGatePos, GateShootPos;
    public  Follower follower;
    public void buildPaths() {
        StartShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        ShootBall1Pos = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose,control1,ballPose1))
                .setLinearHeadingInterpolation(shootPose.getHeading(),ballPose1.getHeading())
                .build();
        Ball1ShootPos = follower.pathBuilder()
                .addPath(new BezierCurve(ballPose1,control2,shootPose))
                .setLinearHeadingInterpolation(ballPose1.getHeading(),shootPose.getHeading())
                .build();
        ShootGatePos = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose,control1,gatePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), gatePose.getHeading())
                .build();
        GateShootPos = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose,control2,shootPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), shootPose.getHeading())
                .build();
        ShootBall2Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,ballPose2))
                .setLinearHeadingInterpolation(shootPose.getHeading(),ballPose2.getHeading())
                .build();
        Ball2ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(ballPose2,shootPose))
                .setLinearHeadingInterpolation(ballPose2.getHeading(),shootPose.getHeading())
                .build();
    }
    public enum State{
        StartShoot,
        ShootBall1,
        Ball1Shoot,
        ShootGate,
        GateShoot,
        ShootBall2,
        Ball2Shoot,
    };
    State state;
    public Test(Follower follower, TelemetryManager telemetry){
        this.follower = follower;
        this.telemetry = telemetry;
        state = State.StartShoot;
        buildPaths();
    }

    public void update(){
        follower.update();
        switch (state){
            case StartShoot:
                follower.followPath(StartShootPos);
                state = State.ShootBall2;
                telemetry.addLine("Sigma 1");
                break;
            case ShootBall2:
                telemetry.addLine("Sigma 2");
                if (!follower.isBusy()){
                    telemetry.addLine("Sigma done");
                }
                break;
        }
    }
}
