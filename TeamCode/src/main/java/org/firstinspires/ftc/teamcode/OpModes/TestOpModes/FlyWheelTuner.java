package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.F;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.I;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.P;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.D;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.vel1;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.vel2;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Iustinel")
@Configurable
public class FlyWheelTuner extends LinearOpMode {
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    private DcMotorEx shoot1,shoot2;
    @Override
    public void runOpMode() throws InterruptedException{
        hardwinit();
        waitForStart();
        while (opModeIsActive()){
            shoot1.setVelocity(vel1);
            shoot2.setVelocity(vel2);
            telemetryM.addData("P",P);
            telemetryM.addData("F",F);
            telemetryM.addData("Error 1",shoot2.getVelocity());
            telemetryM.update();
            PIDFCoefficients pidfcoef = new PIDFCoefficients(P,I,D,F);
            shoot1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfcoef);
            shoot2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfcoef);

        }

    }
    public void hardwinit(){
        shoot1 = hardwareMap.get(DcMotorEx.class,"shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class,"shoot2");
        PIDFCoefficients pidfcoef = new PIDFCoefficients(P,I,D,F);
        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfcoef);
        shoot2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfcoef);
        shoot1.setDirection(DcMotorSimple.Direction.REVERSE);

    }
}
