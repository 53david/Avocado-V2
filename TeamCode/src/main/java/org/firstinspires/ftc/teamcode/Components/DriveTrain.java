package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;
import static org.firstinspires.ftc.teamcode.OpModes.TestOpModes.FieldCentric.imu;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.leftBack;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.leftFront;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.rightBack;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.rightFront;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Wrappers.PIDController;

@Configurable
public class DriveTrain {
    public static double Voltage = 0;
    public static double ks1 = 0, ks2 = 0.07;
    boolean ok; private static double multiplier = 1, multi = 1.2;
    private PIDController tuner = new PIDController(0,0,0);
    public DriveTrain() {

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

        leftFront.setPower(((frontLeftPower / denominator / Voltage)+ ks2));
        leftBack.setPower(((backLeftPower / denominator / Voltage) + ks1));
        rightFront.setPower(((frontRightPower / denominator /Voltage) + ks2));
        rightBack.setPower(((backRightPower / denominator / Voltage) + ks1));


    }
    public void fieldDrive(){
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.left_trigger - gamepad1.right_trigger;


        double orientation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double YFieldOriented = y * Math.cos(orientation) - x * Math.sin(orientation);
        double XFieldOriented = y * Math.sin(orientation) + x * Math.cos(orientation);

        double frontLeftPower = (YFieldOriented + XFieldOriented + rx);
        double frontRightPower = (YFieldOriented - XFieldOriented - rx);
        double backLeftPower = (YFieldOriented - XFieldOriented + rx);
        double backRightPower = (YFieldOriented + XFieldOriented - rx);

        double denominator = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
        denominator = Math.max(denominator, Math.abs(frontRightPower));
        denominator = Math.max(denominator, Math.abs(backRightPower));
        denominator = Math.max(denominator, 1.0);

        leftFront.setPower(frontLeftPower/denominator + ks2);
        rightFront.setPower(frontRightPower/denominator +ks2);
        leftBack.setPower(backLeftPower/denominator + ks1);
        rightBack.setPower(backRightPower/denominator + ks2);
    }


}
