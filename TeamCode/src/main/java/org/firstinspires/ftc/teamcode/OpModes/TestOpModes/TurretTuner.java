package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import android.util.Size;

import static org.firstinspires.ftc.teamcode.Stuff.TurretPID.P;
import static org.firstinspires.ftc.teamcode.Stuff.TurretPID.I;
import static org.firstinspires.ftc.teamcode.Stuff.TurretPID.D;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
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

    private static final double n = 540/90.0;
    private static double x= 0;
    double a = 0,target = 0;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    double fx=807.567, fy=807.567, cx=345.549 , cy=267.084;
    public PIDController pid = new PIDController(P,I,D);
    public PIDCoefficients coef = new PIDCoefficients(P,I,D);
    @Override
    public void runOpMode()throws InterruptedException{
        hardwinit();
        waitForStart();

        while (opModeIsActive()){
            ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
            boolean IsTagSeen = false;
            for(AprilTagDetection tag:tagProcessor.getDetections()){
                if (tag.id == 20) {
                    telemetryM.addData("ID", tag.id);
                    telemetryM.addData("Pitch", tag.ftcPose.pitch);
                    telemetryM.addData("Yaw", tag.ftcPose.yaw);
                    telemetryM.addData("Bearing", tag.ftcPose.bearing);
                    rotate.setPower(pid.calculatePower(tag.ftcPose.bearing));
                    coef = new PIDCoefficients(P, I, D);
                    pid.setPidCoefficients(coef);
                    IsTagSeen = true;
                }
            }
            if(!IsTagSeen){
                rotate.setPower(0);
            }

            telemetryM.update();
        }
    }
    public void hardwinit(){
        pid.setTargetPosition(0);

        webcam = hardwareMap.get(WebcamName.class,"Webcam 1");
        rotate = hardwareMap.get(DcMotorEx.class,"rotate");
        rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        PanelsCameraStream.INSTANCE.startStream(visionPortal,30);
    }

}
