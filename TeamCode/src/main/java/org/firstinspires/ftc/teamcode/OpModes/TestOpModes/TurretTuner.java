package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import android.util.Size;

import static org.firstinspires.ftc.teamcode.Stuff.TurretPID.P;
import static org.firstinspires.ftc.teamcode.Stuff.TurretPID.I;
import static org.firstinspires.ftc.teamcode.Stuff.TurretPID.D;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Stuff.PIDController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Configurable
@TeleOp(name="Turret Tuner")
public class TurretTuner extends LinearOpMode {
    public DcMotorEx rotate;
    public AprilTagProcessor tagProcessor;
    public VisionPortal visionPortal;
    WebcamName webcam;
    public PIDController tuner = new PIDController(P,I,D);
    private static final double n = 300.0/48.0;
    double a = 0,target = 0;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    double fx=752.848, fy=752.848, cx=314.441, cy=219.647;
    public PIDCoefficients coef = new PIDCoefficients(P,I,D);
    @Override
    public void runOpMode()throws InterruptedException{
        hardwinit();
        waitForStart();
        while (opModeIsActive()){
            ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
            if (!detections.isEmpty()) {
                AprilTagDetection tag = detections.get(0);
                telemetryM.addData("ID",tag.id);
                telemetryM.addData("Pitch",tag.ftcPose.pitch);
                telemetryM.addData("Yaw",tag.ftcPose.yaw);
                telemetryM.addData("Bearing",tag.ftcPose.bearing);
                a = tag.ftcPose.bearing;
            }
            tuner.setPidCoefficients(coef);
            turretTrack();
            tUpdate();
            telemetryM.update();
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
    public void tUpdate(){
            rotate.setPower(tuner.calculatePower(FromTicksToDegrees()));

    }
    public void turretTrack(){
        target += a;
        tuner.setTargetPosition(target);
    }
    public double FromTicksToDegrees(){
        return rotate.getCurrentPosition() /384.5 * Math.PI * 2.0;
    }
}
