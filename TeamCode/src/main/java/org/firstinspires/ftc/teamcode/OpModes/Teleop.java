package org.firstinspires.ftc.teamcode.OpModes;


import static org.firstinspires.ftc.teamcode.Pedro.Constants.res;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.DriveTrain;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Outake;

import org.firstinspires.ftc.teamcode.Components.Turret;

@TeleOp(name = "Vlad e the goat")
public class Teleop extends LinearOpMode {
    public static IMU imu;
    private DriveTrain chassis; private Intake intake; private Turret turret;
    private Outake outake; Servo transfer;
    GoBildaPinpointDriver gobilda;
    WebcamName webcam;
    DcMotorEx intakeMotor,rotate,leftFront,leftBack,rightBack,rightFront,shoot1,shoot2;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    public static Gamepad prevgm1,prevgm2; int nr = 0;
    public static Gamepad gm1,gm2;
    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        prevgm1 = new Gamepad();
        prevgm2 = new Gamepad();
        gm1 = new Gamepad();
        gm2 = new Gamepad();
        waitForStart();
        while (opModeIsActive()) {
            gm1.copy(gamepad1);
            gm2.copy(gamepad2);
            intake.update();
            turret.update();
            chassis.update();
            prevgm1.copy(gm1);
            prevgm2.copy(gm2);

        }

    }
    private void initializeHardware() {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));
        imu.resetYaw();
        webcam = hardwareMap.get(WebcamName.class,"Webcam 1");
        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
        gobilda = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        gobilda.resetPosAndIMU();
        shoot1 = hardwareMap.get(DcMotorEx.class,"shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class,"shoot2");
        rotate = hardwareMap.get(DcMotorEx.class,"rotate");
        transfer = hardwareMap.get(Servo.class,"transfer");
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intake");
        chassis = new DriveTrain(leftFront,rightFront,leftBack,rightBack);
        intake = new Intake(intakeMotor,transfer);
        turret = new Turret(rotate,shoot1,shoot2,telemetryM,webcam,gobilda);
        gobilda.setEncoderResolution(res, DistanceUnit.MM);
        Turret.Voltage = 12.0/hardwareMap.getAll(VoltageSensor.class).get(0).getVoltage();
        DriveTrain.Voltage = 12.0/hardwareMap.getAll(VoltageSensor.class).get(0).getVoltage();
    }
}