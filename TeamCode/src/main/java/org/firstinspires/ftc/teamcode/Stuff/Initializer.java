package org.firstinspires.ftc.teamcode.Stuff;

import static org.firstinspires.ftc.teamcode.Components.Intake.intakeMotor;
import static org.firstinspires.ftc.teamcode.Components.Intake.transfer;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Initializer {
    public static void start(HardwareMap hwMap){
        intakeMotor = hwMap.get(DcMotorEx.class,"intake");
        transfer = hwMap.get(Servo.class,"transfer");
    }
}
