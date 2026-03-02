package org.firstinspires.ftc.teamcode.PathFollower;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Trajectories.CloseBlueTrajectory;

public class CloseBlueFollow {
    public Follower follower;
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
    public CloseBlueFollow(){
        state = State.StartShoot;
        CloseBlueTrajectory.follower = follower;
        CloseBlueTrajectory.buildPaths();
    }
    public void update(){
        switch (state){
            case StartShoot:
                follower.followPath(CloseBlueTrajectory.StartShootPos);
                if (!follower.isBusy())
                    state = State.ShootBall1;
            case ShootBall1:
                follower.followPath(CloseBlueTrajectory.ShootBall1Pos);
                if (!follower.isBusy())
                    state = State.Ball1Shoot;
                break;
            case Ball1Shoot:
                follower.followPath(CloseBlueTrajectory.Ball1ShootPos);
                if (!follower.isBusy())
                    state = State.ShootBall2;
                break;
            case ShootBall2:
                follower.followPath(CloseBlueTrajectory.ShootBall2Pos);
                if (!follower.isBusy())
                    state = State.Ball2Shoot;
                break;
            case Ball2Shoot:
                follower.followPath(CloseBlueTrajectory.Ball2ShootPos);
                if (!follower.isBusy())
                    state = State.ShootGate;
                break;
            case ShootGate:
                follower.followPath(CloseBlueTrajectory.ShootGatePos);
                if (!follower.isBusy())
                    state = State.GateShoot;
                break;
            case GateShoot:
                follower.followPath(CloseBlueTrajectory.GateShootPos);
                if (!follower.isBusy())
                    state = State.ShootGate;
                break;
        }
    }

}
