package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


public class DriveTrain {
    private final DcMotorEx leftFront, rightFront, leftBack, rightBack;
    Servo transfer; ElapsedTime timer1 = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    boolean ok;
    public DriveTrain(DcMotorEx leftFront, DcMotorEx rightFront, DcMotorEx leftBack, DcMotorEx rightBack) {

        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack=leftBack;
        this.rightBack = rightBack;
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }

    public void test(Gamepad Gamepad1){
        boolean x = Gamepad1.square;
        boolean x1 = Gamepad1.circle;
        boolean x2 = Gamepad1.cross;
        boolean x3 = Gamepad1.triangle;

        int y = x? 1:0;
        int y1= x1? 1:0;
        int y2 = x2? 1:0;
        int y3= x3? 1:0;

        leftFront.setPower(y);
        leftBack.setPower(y1);
        rightFront.setPower(y2);
        rightBack.setPower(y3);
    }
    public void scale(double x){
        if (x>=0.3){
            x=1;
        }
        else return;
    }
    public void drive() {

        double y = -gm1.left_stick_y;
        double x = gm1.left_stick_x*1.1;
        double rx = gm1.right_trigger-gm1.left_trigger;
        scale(x);scale(y);scale(rx);
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx)/ denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x- rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }
    public void Strafe (){
        leftFront.setPower(-1);
        leftBack.setPower(1);
        rightFront.setPower(1);
        rightBack.setPower(-1);
    }
    public void BackWards(){
        leftFront.setPower(-1);
        leftBack.setPower(-1);
        rightFront.setPower(-1);
        rightBack.setPower(-1);
    }


}