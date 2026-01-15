package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;


public class DriveTrain {
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    boolean ok;

    public DriveTrain(DcMotorEx leftFront, DcMotorEx rightFront, DcMotorEx leftBack, DcMotorEx rightBack) {

        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }

    public void test() {
        if (gm1.square) {
            leftFront.setPower(1);
        } else leftFront.setPower(0);
        if (gm1.triangle) {
            rightFront.setPower(1);
            //left back
        } else rightFront.setPower(0);
        if (gm1.circle) {
            leftBack.setPower(1);
            //right back
        } else leftBack.setPower(0);
        if (gm1.cross) {
            rightBack.setPower(1);
            //right front
        } else rightBack.setPower(0);
    }

    public void update() {

        double y = gm1.left_stick_y;
        double x = -gm1.left_stick_x * 1.1;
        double rx = gm1.left_trigger - gm1.right_trigger;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }

    public void StrafeRight() {
        leftFront.setPower(-1);
        leftBack.setPower(1);
        rightFront.setPower(1);
        rightBack.setPower(-1);
    }

    public void StrafeLeft() {
        leftFront.setPower(1);
        leftBack.setPower(-1);
        rightFront.setPower(-1);
        rightBack.setPower(1);
    }

    public void BackWards() {
        leftFront.setPower(-1);
        leftBack.setPower(-1);
        rightFront.setPower(-1);
        rightBack.setPower(-1);
    }
}
