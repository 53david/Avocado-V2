package org.firstinspires.ftc.teamcode.Trajectories;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class FarBlueTrajectory {
    public static Pose startPose = new Pose(56.000, 8.000, Math.toRadians(90));
    private static Pose shootPose = new Pose(59.670915411355736, 22.484356894553883, Math.toRadians(120));
    private static Pose station = new Pose(8.10776361529548, 8.996523754345302, Math.toRadians(200));
    private static Pose transition = new Pose(60.088064889918876, 36.867902665121655, Math.toRadians(180));
    private static Pose ball = new Pose(17.86210892236384, 35.77636152954809, Math.toRadians(180));
    public static PathChain StartShootPos, ShootStationPos, StationShootPos, ShootTransitionPos, TransitionBallPos, BallShootPos;
    public static Follower follower;
    public static void buildPaths() {
        StartShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        ShootStationPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, station))
                .setLinearHeadingInterpolation(shootPose.getHeading(), station.getHeading(), 0.6)
                .build();
        StationShootPos = follower.pathBuilder()
                .addPath(new BezierLine(station, shootPose))
                .setLinearHeadingInterpolation(station.getHeading(), shootPose.getHeading())
                .build();
        ShootTransitionPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, transition))
                .setLinearHeadingInterpolation(shootPose.getHeading(), transition.getHeading())
                .build();
        TransitionBallPos = follower.pathBuilder()
                .addPath(new BezierLine(transition, ball))
                .setLinearHeadingInterpolation(transition.getHeading(), ball.getHeading())
                .build();
        BallShootPos = follower.pathBuilder()
                .addPath(new BezierLine(ball, shootPose))
                .setLinearHeadingInterpolation(ball.getHeading(), shootPose.getHeading())
                .build();

    }
}
