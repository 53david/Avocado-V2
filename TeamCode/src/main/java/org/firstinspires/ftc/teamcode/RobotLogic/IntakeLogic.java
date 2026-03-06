package org.firstinspires.ftc.teamcode.RobotLogic;

import static org.firstinspires.ftc.teamcode.Components.Intake.intakeMotor;

public class IntakeLogic {
    public enum State {
        IDLE,
        INTAKE,
    }
    State state;
    public IntakeLogic(){
        state = State.IDLE;
    }
    public void update(){
        switch (state){
            case IDLE:
                intakeMotor.setPower(0);
                break;
            case INTAKE:
                intakeMotor.setPower(1);
                break;
        }
    }
}
