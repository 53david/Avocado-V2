package org.firstinspires.ftc.teamcode.RobotLogic;

public class Robot {

    IntakeLogic intakeLogic;
    ShooterLogic shooterLogic;
    TransferLogic transferLogic;
    public Robot() {
        intakeLogic = new IntakeLogic();
        shooterLogic = new ShooterLogic();
        transferLogic = new TransferLogic();
    }
    public void update(){
        intakeLogic.update();
        shooterLogic.update();
        transferLogic.update();
    }
}
