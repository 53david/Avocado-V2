package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;
import static org.firstinspires.ftc.teamcode.Stuff.Sigma.Kd;
import static org.firstinspires.ftc.teamcode.Stuff.Sigma.Ki;
import static org.firstinspires.ftc.teamcode.Stuff.Sigma.Kp;
import static org.firstinspires.ftc.teamcode.Stuff.Sigma.Ks;
import static org.firstinspires.ftc.teamcode.Stuff.Sigma.Kv;

import android.util.Size;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Stuff.PIDController;
import org.firstinspires.ftc.teamcode.Stuff.ShooterConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Configurable
@TeleOp (name="Formula")
public class BetterFormula extends LinearOpMode {
    DcMotorEx shoot1,shoot2;
    double delta = 0;
    double vel1 = 0;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    TelemetryManager telemetryC = PanelsTelemetry.INSTANCE.getTelemetry();
    double fx=752.848, fy=752.848, cx=314.441, cy=219.647;
    WebcamName webcam;
    public AprilTagProcessor tagProcessor;
    public VisionPortal visionPortal;

    PIDController controller = new PIDController(Kp,Ki,Kd);
    PIDCoefficients coef = new PIDCoefficients(Kp,Ki,Kd);
    @Override
    public void runOpMode()throws InterruptedException{
        hardwinit();
        waitForStart();
        while (opModeIsActive()){

            controller.setPidCoefficients(coef);
            vel1 = controller.calculatePower(shoot1.getVelocity());
            vel1+= Kv * ShooterConstants.fwVel(delta) + Ks;
            shoot1.setPower(vel1);
            shoot2.setPower(vel1);

            telemetryM.addData("Error",vel1-shoot1.getVelocity());
            telemetryM.update();

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
        }
    }
    public void hardwinit(){
        shoot2 = hardwareMap.get(DcMotorEx.class,"shoot2");
        shoot1 = hardwareMap.get(DcMotorEx.class,"shoot1");
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
