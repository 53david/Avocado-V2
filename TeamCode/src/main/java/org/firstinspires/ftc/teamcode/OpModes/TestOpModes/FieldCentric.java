package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Components.DriveTrain;

@TeleOp
public class FieldCentric extends LinearOpMode {
    public static IMU imu;
    @Override
    public void runOpMode(){
        DriveTrain drive = new DriveTrain();
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));
        imu.resetYaw();
        waitForStart();
        while (opModeIsActive()){
            drive.fieldDrive();
        }
    }
}
