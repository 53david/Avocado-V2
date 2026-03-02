package org.firstinspires.ftc.teamcode.PathFollower;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Trajectories.FarRedTrajectory;

public class FarRedFollower {
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
    public FarRedFollower(){
        state = State.Start_ShootPos;
        FarRedTrajectory.follower = follower;
        FarRedTrajectory.buildPaths();
    }
    public void update(){
        switch (state){
            case Start_ShootPos:
                follower.followPath(FarRedTrajectory.StartShootPos);
                break;
            case Shoot_Transition:
                follower.followPath(FarRedTrajectory.ShootTransitionPos);
                break;
            case Transition_Ball:
                follower.followPath(FarRedTrajectory.TransitionBallPos);
                break;
            case Ball_Shoot:
                follower.followPath(FarRedTrajectory.BallShootPos);
                break;
            case Shoot_Station:
                follower.followPath(FarRedTrajectory.ShootStationPos);
            case Station_Shoot:
                follower.followPath(FarRedTrajectory.StationShootPos);
                break;
        }
    }
}
