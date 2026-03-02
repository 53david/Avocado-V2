package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Stuff.Initializer;

@TeleOp
public class IntakeTuner extends LinearOpMode {
    Intake intake = new Intake();
    @Override
    public void runOpMode(){
        Initializer.start(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            intake.test();
        }
    }
}
