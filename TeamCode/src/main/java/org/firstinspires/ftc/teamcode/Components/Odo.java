package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class Odo {
    double res = 2000/(Math.PI * 31);
    public static double RedGoalX = 144,RedGoalY = 144,BlueGoalX = 144, BlueGoalY = 0;
    TelemetryManager telemetry;
    static GoBildaPinpointDriver pp;
    public Odo(GoBildaPinpointDriver pp){
        this.pp=pp;
        pp.setOffsets(48.366 * 0.0394,48.366 * 0.0394, DistanceUnit.INCH);
        pp.setEncoderResolution(res,
                DistanceUnit.MM);
        pp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
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
        return pp.getHeading(AngleUnit.DEGREES)- Math.toDegrees(Math.atan(RedGoalX -pp.getPosX(DistanceUnit.INCH)/(RedGoalY-pp.getPosY(DistanceUnit.INCH))));
    }
    public double thetaBlue(){
        return pp.getHeading(AngleUnit.DEGREES)- Math.toDegrees(Math.atan(BlueGoalX -pp.getPosX(DistanceUnit.INCH)/(BlueGoalY-pp.getPosY(DistanceUnit.INCH))));
    }
    public double podX(){
        return pp.getPosX(DistanceUnit.MM);
    }
    public double podY(){
        return pp.getPosY(DistanceUnit.MM);
    }
    public void reset(){
        pp.resetPosAndIMU();
    }

    public void setPosition(SparkFunOTOS.Pose2D pose){
        pp.setPosition(new Pose2D(DistanceUnit.MM,pose.x,pose.y, RADIANS, pose.h));
    }
    public static double cwDistance(double from, double to) {
        from = normalizeRadians(from);
        to = normalizeRadians(to);

        double dist = to - from;
        if (dist < 0) dist += 2 * Math.PI;
        return dist;
    }
    public static SparkFunOTOS.Pose2D getCurrentPosition(){
        double h = pp.getHeading(RADIANS);
        h = normalizeRadians(h);
        return new SparkFunOTOS.Pose2D(pp.getPosX(DistanceUnit.MM),pp.getPosY(DistanceUnit.MM), h);
    }
    public static double distance(){
        getCurrentPosition();
            return Math.sqrt(
                    (getCurrentPosition().x - Turret.goalPositionX) * (getCurrentPosition().x - Turret.goalPositionX) +
                            (getCurrentPosition().y - Turret.goalPositionY) * (getCurrentPosition().y - Turret.goalPositionY)
            );
    }


}
