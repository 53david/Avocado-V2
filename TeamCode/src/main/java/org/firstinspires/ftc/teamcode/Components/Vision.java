package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.prevgm1;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.telemetryM;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.webcam;

import android.util.Size;

import com.bylazar.camerastream.PanelsCameraStream;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class Vision {

    public  AprilTagProcessor tagProcessor;
    public  VisionPortal visionPortal;
    double fx = 807.567, fy = 807.567, cx = 345.549, cy = 267.084;
    public static double allianceID = 20;

    public static long exposure = 2;
    public static int gain = 200;
    public static int temp = 40000;
    public enum AllienceState {
        RED,
        BLUE,
    }
    AllienceState state = AllienceState.BLUE;
    public Vision(){

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setLensIntrinsics(fx, fy, cx, cy)
                .build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(webcam)
                .setCameraResolution(new Size(800, 600))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setLiveViewContainerId(0)
                .build();

        PanelsCameraStream.INSTANCE.startStream(visionPortal, 10);

    }
    public void update(){
        if (visionPortal.getCameraState()==VisionPortal.CameraState.STREAMING) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            WhiteBalanceControl whiteBalanceControl= visionPortal.getCameraControl(WhiteBalanceControl.class);
            exposureControl.setMode(ExposureControl.Mode.Manual);
            whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
            exposureControl.setExposure(exposure, TimeUnit.MILLISECONDS);
            whiteBalanceControl.setWhiteBalanceTemperature(temp);
            gainControl.setGain(gain);

        }
        AllienceUpdate();
    }
    public  double CameraOffset(){

        for(AprilTagDetection tag:tagProcessor.getDetections()){
            if (tag.id == allianceID) {
                return Math.toRadians(tag.ftcPose.bearing);
            }
        }
        return 1e9;
    }
    public double DistanceOffset(){

        for(AprilTagDetection tag:tagProcessor.getDetections()){
            if (tag.id == allianceID) {
                return tag.ftcPose.range;
            }
        }
        return 1e9;
    }
    public void AllienceUpdate(){
        switch (state){
            case BLUE:
                allianceID = 20;
                if (gm1.square && gm1.square!=prevgm1.square)
                    state = AllienceState.RED;
                break;
            case RED:
                allianceID = 24;
                if (gm1.square && gm1.square!=prevgm1.square)
                    state = AllienceState.BLUE;

                break;
        }
    }
}
