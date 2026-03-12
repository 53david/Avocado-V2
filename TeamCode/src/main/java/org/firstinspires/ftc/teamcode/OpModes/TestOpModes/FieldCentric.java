package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Components.DriveTrain;
import org.firstinspires.ftc.teamcode.Wrappers.Initializer;

@TeleOp
public class FieldCentric extends LinearOpMode {
    public static IMU imu;
    DriveTrain drive;
    @Override
    public void runOpMode(){
        Initializer.start(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));
        imu.resetYaw();
        drive = new DriveTrain();
        waitForStart();
        while (opModeIsActive()){
            drive.fieldDrive(gamepad1);
        }
    }
}
