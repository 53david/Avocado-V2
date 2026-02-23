package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;
import static org.firstinspires.ftc.teamcode.Stuff.TurretPID.Kd;
import static org.firstinspires.ftc.teamcode.Stuff.TurretPID.Ki;
import static org.firstinspires.ftc.teamcode.Stuff.TurretPID.Kp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Stuff.PIDController;

@Configurable
public class Pivot {

    private static double y=0.33,x = 0.25;
    DcMotorEx Pivot;
    public static double target = 0;
    int pos = 0; Servo transfer;
    GoBildaPinpointDriver pp;
    Odo odo;
    public double gearRatio = 130.0 / 24.0;
    PIDController pid = new PIDController(Kp,Ki,Kd);

    public Pivot (DcMotorEx Pivot){
        this.Pivot = Pivot;
        Pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void update (){
        odo = new Odo(pp);
        odo.update();
        Pivot.setPower(pid.calculatePower(Pivot.getCurrentPosition()/(384.5 * gearRatio) * Math.PI * 2.0));
        pid.setTargetPosition(Math.PI/180 * target);
    }
}
