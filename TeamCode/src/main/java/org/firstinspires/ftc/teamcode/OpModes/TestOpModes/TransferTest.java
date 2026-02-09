package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Transfer Test")
@Configurable
@Disabled
public class TransferTest extends LinearOpMode {
    public static double x = 0.33,power = 0;
    Servo transfer;
    DcMotorEx intake;
    @Override
    public void runOpMode(){
        hardwinit();
        waitForStart();
        while (opModeIsActive()) {
            transfer.setPosition(x);
            intake.setPower(power);
        }
    }
    public void hardwinit(){
        transfer = hardwareMap.get(Servo.class,"transfer");
        intake = hardwareMap.get(DcMotorEx.class,"intake");
    }

}
