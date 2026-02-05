package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.F;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.I;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.P;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.D;


import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.pos;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.vel1;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.vel2;

import android.util.Size;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Stuff.PIDController;
import org.firstinspires.ftc.teamcode.Stuff.ShooterConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@TeleOp(name = "FlyWheel Tuner")
@Configurable
@Disabled
public class FlyWheelTuner extends LinearOpMode {
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    TelemetryManager telemetryC = PanelsTelemetry.INSTANCE.getTelemetry();
    double fx=752.848, fy=752.848, cx=314.441, cy=219.647;
    double delta  = 0.0;
    public static double multiplier = 1;
    private DcMotorEx shoot1,shoot2; private WebcamName webcam;
    public AprilTagProcessor tagProcessor;
    public VisionPortal visionPortal;
    private Servo transfer;
    @Override
    public void runOpMode() throws InterruptedException{
        hardwinit();
        waitForStart();
        while (opModeIsActive()){
            transfer.setPosition(pos);
            PIDFCoefficients pidfcoef = new PIDFCoefficients(P,I,D,F);
            telemetryM.addData("Error",shoot1.getVelocity());
            shoot1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfcoef);
            shoot2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfcoef);
            ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
            if (!detections.isEmpty()) {
                AprilTagDetection tag = detections.get(0);
                telemetryC.addData("ID",tag.id);
                telemetryC.addData("Bearing",tag.ftcPose.bearing);
                telemetryC.addData("Y",tag.ftcPose.y);
                telemetryC.addData("X",tag.ftcPose.x);
                telemetryC.addData("Z",tag.ftcPose.z);
                delta = tag.ftcPose.range;
                telemetryC.update();

            }

            telemetryM.update();
        }

    }
    public void hardwinit(){
        webcam = hardwareMap.get(WebcamName.class,"webcam1");
        shoot1 = hardwareMap.get(DcMotorEx.class,"shoot1");
        transfer = hardwareMap.get(Servo.class,"transfer");
        shoot2 = hardwareMap.get(DcMotorEx.class,"shoot2");
        PIDFCoefficients pidfcoef = new PIDFCoefficients(P,I,D,F);
        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfcoef);
        shoot2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfcoef);
        shoot1.setDirection(DcMotorSimple.Direction.REVERSE);

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
        PanelsCameraStream.INSTANCE.startStream(visionPortal,10);

    }
}
