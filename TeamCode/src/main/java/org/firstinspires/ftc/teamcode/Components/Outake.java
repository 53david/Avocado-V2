package org.firstinspires.ftc.teamcode.Components;


import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm2;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.prevgm1;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.D;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.F;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.I;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.P;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.vel1;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Stuff.ShooterConstants;
@Configurable
public class Outake {
    private Telemetry telemetry;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    double power1,power2;
    double velocity;
    boolean ok = true; int nr = 0;
    private final DcMotorEx shoot1;
    private final DcMotorEx shoot2;
    private DcMotorEx rotate; CRServo p;
    PIDFController controller = new PIDFController(0.02,0,0,0.00025);
    private Servo servo;

    public Outake(DcMotorEx shoot1, DcMotorEx shoot2) {
        this.shoot1 = shoot1;
        this.shoot2 = shoot2;
        shoot1.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void update() {
        power1 += controller.calculate(shoot1.getVelocity(),900);
        power1 = Math.min(power1,1); power2 = Math.min(power2,1);
        power1 = Math.max(power1,-1); power2 = Math.max(power2,-1);
        shoot1.setPower(power1);
        shoot2.setPower(power1);

    }

}


