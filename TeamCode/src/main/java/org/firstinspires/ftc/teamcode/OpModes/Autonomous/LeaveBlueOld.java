package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class LeaveBlueOld extends LinearOpMode {
    ElapsedTime time = new ElapsedTime();
    public static double x =0;

    public static double Voltage = 0;
    double vel1 =0;
    public double rpm = 1400;
    PIDController controller = new PIDController(0.004,0,0);
    TelemetryManager telemetryM;
    org.firstinspires.ftc.teamcode.Stuff.PIDController pid = new org.firstinspires.ftc.teamcode.Stuff.PIDController(1,0,0.05);
    DcMotorEx shoot1, shoot2, rotate,intake;
    Servo transfer;
    DcMotorEx m1,m2,m3,m4; //apple silicon 67
    @Override
        public void runOpMode(){
            hardwinit();
            waitForStart();
            while (opModeIsActive()){
                vel1 = controller.calculate(shoot2.getVelocity(),rpm);
                vel1 += 0.0004 * rpm + 0.15;
                vel1 *= Voltage;
                shoot1.setPower(vel1);
                shoot2.setPower(vel1);
                rotate.setPower(pid.calculatePower(rotate.getCurrentPosition()/(384.5 * (130.0/34.0)) * Math.PI * 2.0));
                telemetryM.addData("Vel",shoot2.getVelocity());
                telemetryM.update();
                pid.setTargetPosition(Math.PI/180*(x));

                if (Math.abs(shoot2.getVelocity())>1340){
                    intake.setPower(1);
                    transfer.setPosition(0.05);
                }
                else {
                    intake.setPower(0);
                    transfer.setPosition(0.3);
                }
                if (time.milliseconds()>8000 && time.milliseconds()<9000) {
                    m1.setPower(-1);
                    m2.setPower(1);
                    m3.setPower(1);
                    m4.setPower(-1);
                }
                else {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                }
                if (time.milliseconds()>12000){
                    terminateOpModeNow();
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
            shoot1 = hardwareMap.get(DcMotorEx.class,"shoot1");
            shoot2 = hardwareMap.get(DcMotorEx.class,"shoot2");
            shoot2.setDirection(DcMotorSimple.Direction.REVERSE);
            intake = hardwareMap.get(DcMotorEx.class,"intake");
            Voltage = 12.0/hardwareMap.getAll(VoltageSensor.class).get(0).getVoltage();
            rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        }
}
