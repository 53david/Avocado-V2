package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PathFollower.CloseBlueFollower;
import org.firstinspires.ftc.teamcode.RobotLogic.Robot;


@Autonomous
public class TestAuton extends LinearOpMode {
    CloseBlueFollower follower = new CloseBlueFollower();
    Robot robot = new Robot();
    @Override
    public void runOpMode(){
        waitForStart();
        while (opModeIsActive()){
            follower.update();
            robot.update();
        }
    }
}
