package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Odo;
import org.firstinspires.ftc.teamcode.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Trajectories.CloseBlueTrajectory;
import org.firstinspires.ftc.teamcode.Trajectories.Test;


@Autonomous
public class TestAuton extends LinearOpMode {
    Follower follower;
    TelemetryManager telemetry;
    GoBildaPinpointDriver pp;
    @Override
    public void runOpMode(){
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        Test test = new Test(follower,telemetry);
        follower.setStartingPose(Test.startPose);
        Odo odo = new Odo(pp);
        waitForStart();
        while (opModeIsActive()){
            follower.update();
            odo.update();
            test.update();
        }
    }
}
