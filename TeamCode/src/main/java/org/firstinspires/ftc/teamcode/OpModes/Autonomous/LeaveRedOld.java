package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static org.firstinspires.ftc.teamcode.Stuff.Sigma.Kd;
import static org.firstinspires.ftc.teamcode.Stuff.Sigma.Ki;
import static org.firstinspires.ftc.teamcode.Stuff.Sigma.Kp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class LeaveRedOld extends LinearOpMode {

    public static double Voltage = 0;
    public static double x =0;
    double vel1 =0;
    public static double multi;
    Servo transfer;
    public double rpm = 1450;
    PIDController controller = new PIDController(0.00007,0,0);
    org.firstinspires.ftc.teamcode.Stuff.PIDController pid = new org.firstinspires.ftc.teamcode.Stuff.PIDController(1,0,0.05);
    DcMotorEx shoot1, shoot2, rotate,intake;
    ElapsedTime time = new ElapsedTime();
    DcMotorEx m1,m2,m3,m4; //apple silicon 67
    @Override
    public void runOpMode(){
        hardwinit();
        waitForStart();
        while (opModeIsActive()){
            controller = new PIDController(Kp,Ki,Kd);
            vel1 = controller.calculate(shoot2.getVelocity(),rpm);
            vel1 += 0.0004 * rpm + 0.15;
            vel1 *= Voltage;
            shoot1.setPower(vel1);
            shoot2.setPower(vel1);
            rotate.setPower(pid.calculatePower(rotate.getCurrentPosition()/(384.5 * (130.0/34.0)) * Math.PI * 2.0));
            pid.setTargetPosition(Math.PI/180 * (-x));
            if (Math.abs(shoot2.getVelocity())>1350){
                intake.setPower(1);
                transfer.setPosition(0.05);
            }
            else {
                intake.setPower(0);
                transfer.setPosition(0.3);
            }
            if (time.milliseconds()>15000){
                rpm = 0;
            }
            if (time.milliseconds()>27500 && time.milliseconds()<29000) {
                m1.setPower(1);
                m2.setPower(-1);
                m3.setPower(-1);
                m4.setPower(1);
            }
            else {
                m1.setPower(0);
                m2.setPower(0);
                m3.setPower(0);
                m4.setPower(0);
            }
        }
    }
    public void hardwinit(){
        time.reset();
        transfer = hardwareMap.get(Servo.class,"transfer");
        m1 = hardwareMap.get(DcMotorEx.class,"rightFront");
        m2 = hardwareMap.get(DcMotorEx.class,"leftFront");
        m3 = hardwareMap.get(DcMotorEx.class,"rightBack");
        m4 = hardwareMap.get(DcMotorEx.class,"leftBack");
        m2.setDirection(DcMotorSimple.Direction.REVERSE);
        m4.setDirection(DcMotorSimple.Direction.REVERSE);
        rotate = hardwareMap.get(DcMotorEx.class,"rotate");
        rotate = hardwareMap.get(DcMotorEx.class,"rotate");
        shoot1 = hardwareMap.get(DcMotorEx.class,"shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class,"shoot2");
        rotate = hardwareMap.get(DcMotorEx.class,"rotate");
        shoot2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        Voltage = 12.0/hardwareMap.getAll(VoltageSensor.class).get(0).getVoltage();
        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
