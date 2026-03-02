package org.firstinspires.ftc.teamcode.Trajectories;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class CloseBlueTrajectory {

    public static Pose startPose = new Pose(32.000, 135.000, Math.toRadians(270));
    private static Pose shootPose = new Pose(55.000, 85.000, Math.toRadians(180));
    private static Pose  ballPose1 = new Pose(20.000, 60.000, Math.toRadians(180));
    private static Pose  ballPose2 = new Pose(20.000, 85.000, Math.toRadians(180));
    private static Pose  gatePose = new Pose(14.000, 60.000, Math.toRadians(150));

    public static Pose control1 = new Pose(67.000,63.000);
    public static Pose control2 = new Pose(44.000,66.000);

    public static PathChain StartShootPos, ShootBall1Pos, Ball1ShootPos,ShootBall2Pos ,Ball2ShootPos, ShootGatePos, GateShootPos;
    public static Follower follower;
    public static void buildPaths() {
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
}
