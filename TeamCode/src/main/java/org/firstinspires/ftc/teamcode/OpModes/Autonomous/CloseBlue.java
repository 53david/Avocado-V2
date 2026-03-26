package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Odo;
import org.firstinspires.ftc.teamcode.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Trajectories.CloseBlueTrajectory;
import org.firstinspires.ftc.teamcode.Wrappers.Initializer;
@Autonomous
public class CloseBlue extends LinearOpMode {
    Follower follower;
    CloseBlueTrajectory trajectory;
    Odo odo;
    public void runOpMode(){
        Initializer.start(hardwareMap);
        odo = new Odo();
        follower = Constants.createFollower(hardwareMap);
        trajectory = new CloseBlueTrajectory(follower);
        follower.setStartingPose(CloseBlueTrajectory.startPose);
        waitForStart();
        while(opModeIsActive()){
            follower.update();
            trajectory.update();
        }
    }
}
