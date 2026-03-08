package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.Turret.Voltage;

import static org.firstinspires.ftc.teamcode.OpModes.Teleop.telemetryM;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.shoot1;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.shoot2;
import static org.firstinspires.ftc.teamcode.Wrappers.ShooterConstants.Kd;
import static org.firstinspires.ftc.teamcode.Wrappers.ShooterConstants.Ki;
import static org.firstinspires.ftc.teamcode.Wrappers.ShooterConstants.Kp;
import static org.firstinspires.ftc.teamcode.Wrappers.ShooterConstants.Ks;
import static org.firstinspires.ftc.teamcode.Wrappers.ShooterConstants.Kv;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FlyWheel {
    PIDController controller = new PIDController(Kp,Ki,Kd);
    public static double rpm = 0;
    Vision vision = new Vision();
    public FlyWheel(){
        shoot1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot2.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void update(){

        controller = new PIDController(Kp,Ki,Kd);
        double vel1 = controller.calculate(-shoot2.getVelocity(),rpm);
        vel1 += Kv * rpm + Ks;
        vel1 *= Voltage;
        shoot1.setPower(vel1);
        shoot2.setPower(vel1);
        telemetryM.addData("Velocity",Math.abs(shoot2.getVelocity()));
        telemetryM.addData("Goal dist",vision.DistanceOffset());
        telemetryM.addData("Goal dist",Odo.distance());
        telemetryM.update();

    }
}
