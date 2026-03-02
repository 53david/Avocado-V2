package org.firstinspires.ftc.teamcode.RobotLogic;

import static org.firstinspires.ftc.teamcode.Components.Intake.intakeMotor;

public class IntakeLogic {
    public enum State {
        Idle,
        Intake,
    }
    State state;
    public IntakeLogic(){
        state = State.Idle;
    }
    public void update(){
        switch (state){
            case Idle:
                intakeMotor.setPower(0);
                break;
            case Intake:
                intakeMotor.setPower(1);
                break;
        }
    }
}
