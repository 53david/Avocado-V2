package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.OpModes.Teleop.telemetryM;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.prevgm1;
import static org.firstinspires.ftc.teamcode.Wrappers.TurretPID.Kp;
import static org.firstinspires.ftc.teamcode.Wrappers.TurretPID.Ki;
import static org.firstinspires.ftc.teamcode.Wrappers.TurretPID.Kd;
import com.arcrobotics.ftclib.controller.PIDController;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpModes.Teleop;
import org.firstinspires.ftc.teamcode.Wrappers.ShooterConstants;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.rotate;
@Configurable
public class Turret {
    public static double KP = 0.006;
    public static double KD = 0.0035;
    public static double P = 0;
    public static double D = 0;
    public static double Voltage = 0;
    double target =0;

    public static double ManualOffset =0;
    public static double FeedForwardTurret = 0;
    public static double x = 20;
    GoBildaPinpointDriver pp;
    public double globalError;

    PIDController pid = new PIDController(KP,0,KD);
    org.firstinspires.ftc.teamcode.Wrappers.PIDController turretController = new org.firstinspires.ftc.teamcode.Wrappers.PIDController(Kp,Ki,Kd);
    org.firstinspires.ftc.teamcode.Wrappers.PIDController pd = new org.firstinspires.ftc.teamcode.Wrappers.PIDController(P,0,D);
    public Vision vision = new Vision();
    public enum AllienceState {
        RED,
        BLUE,
    }
    public enum State{
        Manual,
        Auto,
    }
    AllienceState state = AllienceState.BLUE;
    State bstate = State.Manual;
    public static double goalPositionX = 825, goalPositionY = -300;
    public Turret(GoBildaPinpointDriver pp) {

        this.pp = pp;
        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
    private double fromEncoderToRads() {
        double ticks = rotate.getCurrentPosition() + angleOffset;
        double ticksPerRev = 384.5 * (130.0 / 34.0);
        return ticks * 2.0 * Math.PI / ticksPerRev;
    }
    public static double maxContinuosLimit = Math.PI * 2;
    public static double angleOffset = 450;
    public static double offset = 1.9018;

    private double LiniarizedTargetAngle(double targetAngle,double currContinuosAngle){
        double bestdifference = 1e9, bestcontinuosAngle = 1e9;
        double distNormalizedContinuos = Odo.cwDistance(2*Math.PI - offset,targetAngle);
        while(distNormalizedContinuos<=maxContinuosLimit) {
            if(bestdifference > Math.abs(distNormalizedContinuos - currContinuosAngle)) {
                bestdifference = Math.abs(distNormalizedContinuos - currContinuosAngle);
                bestcontinuosAngle = distNormalizedContinuos;
            }
            distNormalizedContinuos+=2*Math.PI;
        }
        if(bestcontinuosAngle == 1e9)
            return 1e9;
        return currContinuosAngle - bestcontinuosAngle;
    }
    public void updateFacingDirection(SparkFunOTOS.Pose2D robotPose) {
        if (vision.CameraOffset() != 1e9) {
                telemetryM.addLine("Esti prost");
                telemetryM.update();
        }
        else {
            robotPose = new SparkFunOTOS.Pose2D(-robotPose.y, robotPose.x, robotPose.h);
            double robotHeading = robotPose.h;
            double dx = goalPositionX - robotPose.x;
            double dy = goalPositionY - robotPose.y;
            double targetGlobalHeading = Math.atan2(dy, dx);
            double ShouldHaveTurretHeading = targetGlobalHeading - robotHeading + Math.toRadians(ManualOffset);
            double currentTurretRel = fromEncoderToRads();
            telemetryM.addData("currentTurretRel", currentTurretRel);
            double error = LiniarizedTargetAngle(ShouldHaveTurretHeading, currentTurretRel);
            globalError = error;
            telemetryM.addData("error", error);
            telemetryM.addData("currticks", rotate.getCurrentPosition());
            if (error == 1e9)
                rotate.setPower(0);
            else
                rotate.setPower(turretController.calculatePower(error) + Math.signum(error) * FeedForwardTurret);
            telemetryM.addData("X", pp.getPosX(DistanceUnit.MM));
            telemetryM.update();
        }
    }

    public void update() {
        PIDCoefficients coef = new PIDCoefficients(Ki,Ki,Kd);
        turretController.setPidCoefficients(coef);
        TurretUpdate();
        AllienceUpdate();

    }
    public void AllienceUpdate(){
        switch (state){
            case BLUE:
                goalPositionX = 850;goalPositionY = -275;
                if (gm1.square && gm1.square!=prevgm1.square)
                    state = AllienceState.RED;
                break;
            case RED:
                goalPositionX = -2600; goalPositionY = -250;
                if (gm1.square && gm1.square!=prevgm1.square)
                    state = AllienceState.BLUE;
                break;
        }
    }
    public void TurretUpdate(){
        Odo odo =new Odo(pp);
        odo.update();
        switch (bstate){
            case Manual:
                target +=gm1.right_stick_x * 40;
                rotate.setPower(pid.calculate(rotate.getCurrentPosition(),target));
                if (gm1.circle && prevgm1.circle!=gm1.circle) {
                    bstate = State.Auto;
                    odo.setPosition(new SparkFunOTOS.Pose2D(0,0, Teleop.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));

                }
                if (gm1.triangle && prevgm1.triangle!=gm1.triangle){
                    rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    target = 0;
                }
                telemetryM.addLine("Manual");
                break;
            case Auto:
                SparkFunOTOS.Pose2D pos = Odo.getCurrentPosition();
                pos.h = Teleop.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                updateFacingDirection(pos);
                odo.setPosition(new SparkFunOTOS.Pose2D(Odo.getCurrentPosition().x, Odo.getCurrentPosition().y, pos.h));
                if (gm1.circle && prevgm1.circle!=gm1.circle) {
                    bstate = State.Manual;
                }
                telemetryM.addLine("Auto");
                break;
        }

    }
    public void test(){
        PIDCoefficients coef = new PIDCoefficients(P,0,D);
        pd.setPidCoefficients(coef);
        if (vision.CameraOffset()!=1e9) {
            rotate.setPower(pd.calculatePower(vision.CameraOffset()));
        }
    }

}

