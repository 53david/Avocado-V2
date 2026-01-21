package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import android.util.Size;

import static org.firstinspires.ftc.teamcode.Stuff.TurretPID.P;
import static org.firstinspires.ftc.teamcode.Stuff.TurretPID.I;
import static org.firstinspires.ftc.teamcode.Stuff.TurretPID.D;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Configurable
@TeleOp(name="Turret Tuner")
public class TurretTuner extends LinearOpMode {
    public DcMotorEx rotate;
    public AprilTagProcessor tagProcessor;
    public VisionPortal visionPortal;
    WebcamName webcam;
    TelemetryManager telemetryM;
    double fx=752.848, fy=752.848, cx=314.441, cy=219.647;
    private PIDCoefficients coef = new PIDCoefficients(P,I,D);
    @Override
    public void runOpMode()throws InterruptedException{
        hardwinit();
        waitForStart();
        while (opModeIsActive()){

        }
    }
    public void hardwinit(){

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setLensIntrinsics(fx,fy,cx,cy)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .build();

        webcam = hardwareMap.get(WebcamName.class,"webcam1");
        rotate = hardwareMap.get(DcMotorEx.class,"rotate");

    }
}
