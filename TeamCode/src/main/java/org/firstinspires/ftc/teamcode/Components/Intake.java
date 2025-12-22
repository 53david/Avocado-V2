package org.firstinspires.ftc.teamcode.Components;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.prevgm1;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.sql.Time;

public class Intake {
    Telemetry telemetry;
    private DcMotorEx intakeMotor;
    private CRServo servo;
    public Intake(DcMotorEx intakeMotor) {
        this.intakeMotor = intakeMotor;
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void update() {
        if (gm1.right_bumper==prevgm1.right_bumper && gm1.right_bumper)
            intakeMotor.setPower(-1);
        else if (gm1.left_bumper==prevgm1.left_bumper && gm1.left_bumper) {
            intakeMotor.setPower(1);
        }
        else {
            intakeMotor.setPower(-0.4);
        }

    }
    public void run(){
        intakeMotor.setPower(1);
    }

}