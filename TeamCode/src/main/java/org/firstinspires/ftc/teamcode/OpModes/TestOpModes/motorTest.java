package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import java.util.ArrayList;

@TeleOp(name = "TestM")
@Configurable
@Disabled
public class motorTest extends LinearOpMode {
    DcMotorEx motor;
    public static int id = 0;
    public static int port = 0;
    public static double power = 0;
    @Override
    public void runOpMode(){
        hardwinit();
        waitForStart();

        while (opModeIsActive()){
            motor = new DcMotorImplEx(hardwareMap.getAll(DcMotorController.class).get(id), port);
            motor.setPower(power);
        }
    }
    public void hardwinit(){

    }
}
