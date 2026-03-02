package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PathFollower.CloseBlueFollow;
import org.firstinspires.ftc.teamcode.Trajectories.FarBlueTrajectory;


@Autonomous
public class TestAuton extends LinearOpMode {
    CloseBlueFollow follower = new CloseBlueFollow();
    @Override
    public void runOpMode(){
        waitForStart();
        while (opModeIsActive()){
            follower.update();
        }
    }
}
