package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PathFollower.CloseBlueFollower;


@Autonomous
public class TestAuton extends LinearOpMode {
    CloseBlueFollower follower = new CloseBlueFollower();
    @Override
    public void runOpMode(){
        waitForStart();
        while (opModeIsActive()){
            follower.update();
        }
    }
}
