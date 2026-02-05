package org.firstinspires.ftc.teamcode.OpModes;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Components.DriveTrain;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Outake;
import org.firstinspires.ftc.teamcode.Components.Pivot;
import org.firstinspires.ftc.teamcode.Components.Storage;
import org.firstinspires.ftc.teamcode.Components.Turret;

@TeleOp(name = "Vlad e the goat")
public class Teleop extends LinearOpMode {
    private DriveTrain chassis; private Intake intake; private Turret turret;
    private Outake outake; Servo transfer;
    private Pivot pivot;
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
        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
        MotorConfigurationType m= leftFront.getMotorType();
        m.setAchieveableMaxRPMFraction(1);
        leftFront.setMotorType(m);
        rightFront.setMotorType(m);
        leftBack.setMotorType(m);
        rightBack.setMotorType(m);
        shoot1 = hardwareMap.get(DcMotorEx.class,"shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class,"shoot2");
        rotate = hardwareMap.get(DcMotorEx.class,"rotate");
        transfer = hardwareMap.get(Servo.class,"transfer");
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intake");
        chassis = new DriveTrain(leftFront,rightFront,leftBack,rightBack);
        intake = new Intake(intakeMotor,transfer);
        turret = new Turret(rotate,shoot1,shoot2,webcam,telemetryM);

    }
}