package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Trajectories.CloseBlueTrajectory;


@Autonomous
public class TestAuton extends LinearOpMode {
    Follower follower;
    @Override
    public void runOpMode(){
        follower = Constants.createFollower(hardwareMap);
        CloseBlueTrajectory trajectory = new CloseBlueTrajectory(follower);

        waitForStart();
        while (opModeIsActive()){
            follower.update();
            trajectory.update();
        }
    }
}
