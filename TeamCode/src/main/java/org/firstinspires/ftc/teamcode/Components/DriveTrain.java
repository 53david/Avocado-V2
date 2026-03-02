package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Stuff.PIDController;

@Configurable
public class DriveTrain {
    public static double Voltage = 0;
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    public static double ks1 = 0, ks2 = 0.07;
    boolean ok; private static double multiplier = 1, multi = 1.2;
    private PIDController tuner = new PIDController(0,0,0);
    public DriveTrain(DcMotorEx leftFront, DcMotorEx rightFront, DcMotorEx leftBack, DcMotorEx rightBack) {

        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        MotorConfigurationType m= leftFront.getMotorType();
        m.setAchieveableMaxRPMFraction(1);
        leftFront.setMotorType(m);
        rightFront.setMotorType(m);
        leftBack.setMotorType(m);
        rightBack.setMotorType(m);

    }

    public void update() {

        double y = gm1.left_stick_y;
        double x = -gm1.left_stick_x * multi;
        double rx = (gm1.left_trigger - gm1.right_trigger) * multiplier;

        double frontLeftPower = (y + x + rx);
        double backLeftPower = (y - x + rx);
        double frontRightPower = (y - x - rx);
        double backRightPower = (y + x - rx);

        double denominator = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
        denominator = Math.max(denominator, Math.abs(frontRightPower));
        denominator = Math.max(denominator, Math.abs(backRightPower));
        denominator = Math.max(denominator, 1.0);

        leftFront.setPower(((frontLeftPower / denominator)+ ks1));
        leftBack.setPower(((backLeftPower / denominator) + ks2));
        rightFront.setPower(((frontRightPower / denominator) + ks2));
        rightBack.setPower(((backRightPower / denominator) + ks1));


    }


}
