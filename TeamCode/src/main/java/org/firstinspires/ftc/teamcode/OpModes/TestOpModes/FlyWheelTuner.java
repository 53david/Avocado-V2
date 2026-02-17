package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;


import com.arcrobotics.ftclib.controller.PIDFController;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.P;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.I;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.D;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.F;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.vel1;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "FlyWheel Tuner")
@Configurable
public class FlyWheelTuner extends LinearOpMode {

    DcMotorEx m,m1,m2;
    double power = 0;
    public static double x = 0;
    PIDFController controller = new PIDFController(P,I,D,F);
    TelemetryManager telemetry1;
    @Override
    public void runOpMode() throws InterruptedException{
        hardwinit();
        waitForStart();
        while (opModeIsActive()){

            controller = new PIDFController(P,I,D,F);
            power +=  controller.calculate(m.getVelocity(),vel1);
            power = Math.min(power,1);
            power = Math.max(power,-1);
            m.setPower(power);
            m1.setPower(power);
            m2.setPower(x);
            telemetry1.addData("Error",m.getVelocity());
            telemetry1.update();


        }

    }
    public void hardwinit(){
        telemetry1 = PanelsTelemetry.INSTANCE.getTelemetry();
        m1 = hardwareMap.get(DcMotorEx.class,"shoot1");
       m = hardwareMap.get(DcMotorEx.class,"shoot2");
        m.setDirection(DcMotorSimple.Direction.REVERSE);
       m2 = hardwareMap.get(DcMotorEx.class,"intakeMotor");

    }
}
