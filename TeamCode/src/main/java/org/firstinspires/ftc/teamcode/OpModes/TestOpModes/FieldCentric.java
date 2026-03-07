package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.DriveTrain;

@TeleOp
public class FieldCentric extends LinearOpMode {

    @Override
    public void runOpMode(){
        DriveTrain drive = new DriveTrain();
        waitForStart();
        while (opModeIsActive()){
            drive.fieldDrive();
        }
    }
}
