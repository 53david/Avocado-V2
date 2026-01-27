package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.prevgm1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake {
    private DcMotorEx intakeMotor;
    public Intake(DcMotorEx intakeMotor) {
        this.intakeMotor = intakeMotor;
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void update() {
        if (gm1.right_bumper==prevgm1.right_bumper && gm1.right_bumper)
            intakeMotor.setPower(1);
        else if (gm1.left_bumper==prevgm1.left_bumper && gm1.left_bumper) {
            intakeMotor.setPower(-1);
        }
        else {
            intakeMotor.setPower(0.4);
        }

    }

}