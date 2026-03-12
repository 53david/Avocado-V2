package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Components.Turret;
import org.firstinspires.ftc.teamcode.Wrappers.Initializer;

@TeleOp
public class CameraTracking extends LinearOpMode {
    DcMotorEx rotate,shoot1,shoot2;
    WebcamName webcam;
    GoBildaPinpointDriver pp;
    Turret turret;
    @Override
    public void runOpMode(){
        Init();
        waitForStart();
        while (opModeIsActive()){
            turret.test();
        }
    }
    public void Init(){
        Initializer.start(hardwareMap);
        pp = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        turret = new Turret(pp);

    }
}
