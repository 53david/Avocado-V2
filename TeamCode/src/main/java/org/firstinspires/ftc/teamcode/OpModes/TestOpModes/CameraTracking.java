package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Components.Turret;

@TeleOp
public class CameraTracking extends LinearOpMode {
    DcMotorEx rotate,shoot1,shoot2;
    WebcamName webcam;
    GoBildaPinpointDriver pp;
    Turret turret;
    TelemetryManager telemetry1;
    @Override
    public void runOpMode(){
        Init();
        waitForStart();
        while (opModeIsActive()){
            turret.test();
        }
    }
    public void Init(){
        webcam = hardwareMap.get(WebcamName.class,"Webcam 1");
        shoot1 = hardwareMap.get(DcMotorEx.class,"shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class,"shoot2");
        rotate = hardwareMap.get(DcMotorEx.class,"rotate");
        pp = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        telemetry1 = PanelsTelemetry.INSTANCE.getTelemetry();
        turret = new Turret(rotate,shoot1,shoot2,telemetry1,webcam,pp);

    }
}
