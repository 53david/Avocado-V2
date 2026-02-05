package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.prevgm1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class Intake {
    DcMotorEx intakeMotor;
    Servo transfer;
    public enum State{
        INTAKE,
        TRANSFER,
        SPIT,
        IDLE,

    };
    State state = State.IDLE;
    public Intake(DcMotorEx intakeMotor, Servo transfer) {
        this.intakeMotor = intakeMotor;
        this.transfer = transfer;
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        MotorConfigurationType m= intakeMotor.getMotorType();
        m.setAchieveableMaxRPMFraction(1);
        intakeMotor.setMotorType(m);

    }

    public void update() {
        switch (state){
            case IDLE:
                intakeMotor.setPower(0);
                transfer.setPosition();
                break;
            case INTAKE:
                intakeMotor.setPower(1);
                transfer.setPosition();
                break;
            case SPIT:
                intakeMotor.setPower(-1);
                transfer.setPosition();
                break;
            case TRANSFER:
                intakeMotor.setPower(1);
                intakeMotor.setPower();
        }
    }

}