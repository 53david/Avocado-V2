package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Wrappers.Initializer;

@TeleOp
public class IntakeTuner extends LinearOpMode {
    Intake intake;
    @Override
    public void runOpMode(){
        Initializer.start(hardwareMap);
        intake = new Intake();
        waitForStart();
        while (opModeIsActive()) {
            intake.test();
        }
    }
}
