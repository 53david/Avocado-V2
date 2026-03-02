package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.prevgm1;
import static org.firstinspires.ftc.teamcode.OpModes.TestOpModes.BetterFormula.rpm;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
@Configurable
public class Intake {
    public static DcMotorEx intakeMotor;
    public static Servo transfer;
    public static double rpm = 0;
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Ks = 0;
    public static double Kv = 0;
    public PIDController pid = new PIDController(Kp,Ki,Kd);
    public Intake() {

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MotorConfigurationType m= intakeMotor.getMotorType();
        m.setAchieveableMaxRPMFraction(1);
        intakeMotor.setMotorType(m);

    }

    public void update() {
        if(gm1.right_bumper && gm1.right_bumper == prevgm1.right_bumper){
            intakeMotor.setPower(1);
        }
        else if (gm1.left_bumper && gm1.left_bumper == prevgm1.left_bumper){
            transfer.setPosition(0.05);
            intakeMotor.setPower(-1);
        }
        else {
            transfer.setPosition(0.3);
            intakeMotor.setPower(0);
        }
        if (gm1.cross && gm1.cross == prevgm1.cross){
            transfer.setPosition(0.05);
            if (Turret.getVelo()>1100){
                intakeMotor.setPower(1);
            }

        }

}
    public void test(){
        pid = new PIDController(Kp,Ki,Kd);
        double vel1 = pid.calculate(intakeMotor.getVelocity(),rpm);
        vel1 += Kv * rpm + Ks;
        intakeMotor.setPower(vel1);
    }
}