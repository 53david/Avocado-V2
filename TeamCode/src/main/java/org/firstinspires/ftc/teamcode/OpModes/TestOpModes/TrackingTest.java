package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;
import static org.firstinspires.ftc.teamcode.Pedro.Constants.res;
import static org.firstinspires.ftc.teamcode.Stuff.TurretPID.Kp;
import static org.firstinspires.ftc.teamcode.Stuff.TurretPID.Ki;
import static org.firstinspires.ftc.teamcode.Stuff.TurretPID.Kd;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Stuff.PIDController;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Components.Odo;
import org.firstinspires.ftc.teamcode.Stuff.ShooterConstants;

@TeleOp
@Configurable
public class TrackingTest extends LinearOpMode {
    DcMotorEx rotate;
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    public static double powerOffset = 0.7;
    public static double target = 0;
    public double gearRatio = 130.0 / 34.0;
    TelemetryManager telemetryM;
    GoBildaPinpointDriver pp;
    Odo odo;
    PIDController pid = new PIDController(Kp,Ki,Kd);
    @Override
    public void runOpMode(){
        hardwinit();
        Gamepad gm1 = new Gamepad();
        waitForStart();
        while (opModeIsActive()){
            gm1.copy(gamepad1);
            PIDCoefficients coef = new PIDCoefficients(Kp,Ki,Kd);
            pid.setPidCoefficients(coef);
            telemetryM.addData("delta Blue",odo.deltaBLUE());
            telemetryM.addData("delta Red",odo.deltaRED());
            telemetryM.addData("Theta Blue",odo.thetaBlue());
            telemetryM.addData("Theta Red", odo.thetaRed());
            telemetryM.addData("pos x",odo.podX());
            telemetryM.addData("pos y",odo.podY());
            telemetryM.addData("heading",Odo.getCurrentPosition().h);
            telemetryM.addData("distance",Odo.distance()*0.0394);
            telemetryM.addData("Velocity Blue", ShooterConstants.fwVel(Odo.distance()*0.0394));
            telemetryM.addData("Encoder X",pp.getEncoderX());
            telemetryM.addData("Encoder Y",pp.getEncoderY());
            telemetryM.update();

            odo.update();
            rotate.setPower(pid.calculatePower(rotate.getCurrentPosition()/(384.5 * gearRatio) * Math.PI * 2.0));
            pid.setTargetPosition(Math.PI/180 * target);
            if (gm1.circleWasPressed()){
                odo.reset();
            }
        }
    }
    public void hardwinit(){

        rotate = hardwareMap.get(DcMotorEx.class,"rotate");
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pp =hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo = new Odo(pp);
        pp.setEncoderResolution(res, DistanceUnit.MM);
        rotate.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rotate.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}
