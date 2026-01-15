package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
@Configurable
public class Tune extends LinearOpMode {
    public static final double P = 0,F = 0;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    private DcMotorEx shoot1,shoot2;
    int x,vel1 = 4000,vel2 = 8000;
    @Override
    public void runOpMode() throws InterruptedException{
        hardwinit();
        waitForStart();
        while (opModeIsActive()){
            shoot1.setVelocity(x);
            shoot2.setVelocity(x);
            telemetryM.addData("P",P);
            telemetryM.addData("F",F);
            telemetryM.update();
        }

    }
    public void hardwinit(){
        x = vel1;
        PIDFCoefficients pidfcoef = new PIDFCoefficients(P,0,0,F);
        shoot1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfcoef);
        shoot1 = hardwareMap.get(DcMotorEx.class,"shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class,"shoot2");
        shoot1.setDirection(DcMotorSimple.Direction.REVERSE);

    }
}
