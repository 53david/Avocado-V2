package org.firstinspires.ftc.teamcode.Components;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class Vision {
    public DcMotorEx rotate;
    public AprilTagProcessor tagProcessor;
    public VisionPortal visionPortal;
    double fx=752.848, fy=752.848, cx=314.441, cy=219.647;
    private WebcamName webcam;

    public Vision(DcMotorEx rotate,WebcamName webcam){
        this.rotate=rotate;
        this.webcam=webcam;
        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

    }

    public void update() {
        double n = 10.0 / 3.0;
        ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
        if (detections.isEmpty()) {
            return;
        } else {
            AprilTagDetection tag = detections.get(0);
            if (tag.ftcPose == null) {
                return;
            }
            if (tag.id == 20) {
                double a = tag.ftcPose.bearing;
                int target = (int) (a * n);
                rotate.setTargetPosition(target);
                rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotate.setPower(0.25);


            }
        }

    }




}