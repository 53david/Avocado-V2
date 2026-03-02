package org.firstinspires.ftc.teamcode.Trajectories;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class FarRedTrajectory {
    private static Pose startPose = new Pose(88.000,8.000,Math.toRadians(90));
    private static Pose shootPose = new Pose(79.36037079953651,23.65237543453071,Math.toRadians(62));
    private static Pose station = new Pose (133.53650057937426,9.522595596755522,Math.toRadians(0));
    private static Pose transition = new Pose(79.36037079953651,36.867902665121655,Math.toRadians(0));
    private static Pose ball = new Pose(125.47856315179607,35.37427578215528,Math.toRadians(180));
    public static PathChain StartShootPos,ShootStationPos,StationShootPos,ShootTransitionPos,TransitionBallPos,BallShootPos;
    public static Follower follower;
    public static void buildPaths(){
        StartShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();
        ShootStationPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,station))
                .setLinearHeadingInterpolation(shootPose.getHeading(), station.getHeading())
                .build();
        StationShootPos =follower.pathBuilder()
                .addPath(new BezierLine(station,shootPose))
                .setLinearHeadingInterpolation(station.getHeading(),shootPose.getHeading())
                .build();
        ShootTransitionPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,transition))
                .setLinearHeadingInterpolation(shootPose.getHeading(), transition.getHeading())
                .build();
        TransitionBallPos = follower.pathBuilder()
                .addPath(new BezierLine(transition,ball))
                .setLinearHeadingInterpolation(transition.getHeading(),ball.getHeading())
                .build();
        BallShootPos = follower.pathBuilder()
                .addPath(new BezierLine(ball,shootPose))
                .setLinearHeadingInterpolation(ball.getHeading(),shootPose.getHeading())
                .build();
    }
}

