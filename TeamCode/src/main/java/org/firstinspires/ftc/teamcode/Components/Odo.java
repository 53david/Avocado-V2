package org.firstinspires.ftc.teamcode.Components;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class Odo {
    public static double RedGoalX = 143,RedGoalY = 143,BlueGoalX = 143, BlueGoalY = 7.5;
    TelemetryManager telemetry;
    GoBildaPinpointDriver pp;
    public Odo(GoBildaPinpointDriver pp){
        this.pp=pp;
        pp.setOffsets(48.366*0.0394,48.366*0.0394, DistanceUnit.INCH);
        pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pp.setHeading(90,AngleUnit.DEGREES);
    }
    public double deltaRED(){
        return Math.sqrt(Math.pow((pp.getPosX(DistanceUnit.INCH)-RedGoalX),2)+Math.pow((pp.getPosY(DistanceUnit.INCH)-RedGoalY),2));
    }
    public double deltaBLUE(){
        return Math.sqrt(Math.pow((pp.getPosX(DistanceUnit.INCH)-BlueGoalX),2)+Math.pow((pp.getPosY(DistanceUnit.INCH)-BlueGoalY),2));
    }
    public void update(){
        pp.update();
    }
    public double thetaRed(){
        return pp.getHeading(AngleUnit.DEGREES)- Math.toDegrees(Math.atan(RedGoalY-pp.getPosY(DistanceUnit.INCH) /RedGoalX -pp.getPosX(DistanceUnit.INCH)));
    }
    public double thetaBlue(){
        return Math.toDegrees(Math.atan(BlueGoalY-pp.getPosY(DistanceUnit.INCH)/BlueGoalX-pp.getPosX(DistanceUnit.INCH))) - pp.getHeading(AngleUnit.DEGREES);
    }
    public double podX(){
        return pp.getPosX(DistanceUnit.INCH);
    }
    public double podY(){
        return pp.getPosY(DistanceUnit.INCH);
    }
    public double niggerX(){
        return pp.getEncoderX();
    }
    public double niggerY(){
        return pp.getEncoderY();
    }
    public void resetBlue(){
        pp.resetPosAndIMU();
        pp.recalibrateIMU();
        pp.setPosX(39.647740440324455,DistanceUnit.INCH);
        pp.setPosY(136.31517960602548,DistanceUnit.INCH);
        pp.setHeading(90,AngleUnit.DEGREES);
    }

}
