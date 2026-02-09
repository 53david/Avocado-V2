package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;


import com.arcrobotics.ftclib.controller.PIDFController;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.P;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.I;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.D;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.F;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.vel1;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name = "FlyWheel Tuner")
@Configurable
public class FlyWheelTuner extends LinearOpMode {

    DcMotorEx m;
    PIDFController controller = new PIDFController(P,I,D,F);

    @Override
    public void runOpMode() throws InterruptedException{
        hardwinit();
        waitForStart();
        while (opModeIsActive()){

            controller = new PIDFController(P,I,D,F);
            m.setPower(controller.calculate(m.getVelocity(),vel1));

        }

    }
    public void hardwinit(){

       m = hardwareMap.get(DcMotorEx.class,"shoot2");

    }
}
