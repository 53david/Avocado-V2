package org.firstinspires.ftc.teamcode.RobotLogic;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.transfer;

public class TransferLogic {
    public enum State{
        IDLE,
        TRANSFER,
    };
    State state;
    public TransferLogic(){
        state = State.IDLE;
    }
    public void update(){
        switch (state){
            case IDLE :
                transfer.setPosition(0.05);
                break;
            case TRANSFER:
                transfer.setPosition(0.3);
                break;
        }
    }
}
