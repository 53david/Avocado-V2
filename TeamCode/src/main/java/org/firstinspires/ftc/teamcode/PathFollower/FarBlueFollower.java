package org.firstinspires.ftc.teamcode.PathFollower;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Trajectories.FarBlueTrajectory;

public class FarBlueFollower {
    public Follower follower;
    public enum State{
        Start_ShootPos,
        Shoot_Station,
        Station_Shoot,
        Shoot_Transition,
        Transition_Ball,
        Ball_Shoot,
    };
    State state;
    public FarBlueFollower(){
        state = State.Start_ShootPos;
        FarBlueTrajectory.follower = follower;
        FarBlueTrajectory.buildPaths();
    }
    public void update(){
        switch (state){
            case Start_ShootPos:
                follower.followPath(FarBlueTrajectory.StartShootPos);
                break;
            case Shoot_Transition:
                follower.followPath(FarBlueTrajectory.ShootTransitionPos);
                break;
            case Transition_Ball:
                follower.followPath(FarBlueTrajectory.TransitionBallPos);
                break;
            case Ball_Shoot:
                follower.followPath(FarBlueTrajectory.BallShootPos);
                break;
            case Shoot_Station:
                follower.followPath(FarBlueTrajectory.ShootStationPos);
            case Station_Shoot:
                follower.followPath(FarBlueTrajectory.StationShootPos);
                break;
        }
    }
}
