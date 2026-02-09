package org.firstinspires.ftc.teamcode.Components;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Odo {
    public static double RedGoalX = 0,RedGoalY = 0,BlueGoalX = 0, BlueGoalY=0;
    TelemetryManager telemetry;
    GoBildaPinpointDriver pp;
    public Odo(GoBildaPinpointDriver pp, TelemetryManager telemetry){
        this.pp=pp;
        this.telemetry = telemetry;
        pp.setOffsets(48.366/2.54,48.366/2.54, DistanceUnit.INCH);
        pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pp.recalibrateIMU();
    }
    public double positionRED (){
        return Math.sqrt(Math.pow((pp.getPosX(DistanceUnit.INCH)-RedGoalX),2)+Math.pow((pp.getPosY(DistanceUnit.INCH)-RedGoalY),2));

    }
    public double positionBLUE(){
        return Math.sqrt(Math.pow((pp.getPosX(DistanceUnit.INCH)-BlueGoalX),2)+Math.pow((pp.getPosY(DistanceUnit.INCH)-BlueGoalY),2));
    }
    public void update(){
        pp.update();
    }
    public double thetaRed(){
        return Math.toDegrees(Math.acos(Math.sqrt(Math.pow((pp.getPosX(DistanceUnit.INCH)-RedGoalX),2)+Math.pow((pp.getPosY(DistanceUnit.INCH)-RedGoalY),2)))/pp.getPosX(DistanceUnit.INCH));
    }
    public double thetaBlue(){
        return Math.toDegrees(Math.acos(Math.sqrt(Math.pow((pp.getPosX(DistanceUnit.INCH)-BlueGoalX),2)+Math.pow((pp.getPosY(DistanceUnit.INCH)-BlueGoalY),2)))/pp.getPosX(DistanceUnit.INCH));
    }
    public void reset(){
        pp.resetPosAndIMU();
        pp.recalibrateIMU();
    }

}
