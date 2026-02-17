package org.firstinspires.ftc.teamcode.Components;


import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.prevgm1;
import static org.firstinspires.ftc.teamcode.OpModes.TestOpModes.BetterFormula.rpm;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.D;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.F;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.I;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.P;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.vel1;
import static org.firstinspires.ftc.teamcode.Stuff.Sigma.Kd;
import static org.firstinspires.ftc.teamcode.Stuff.Sigma.Ki;
import static org.firstinspires.ftc.teamcode.Stuff.Sigma.Kp;
import static org.firstinspires.ftc.teamcode.Stuff.Sigma.Ks;
import static org.firstinspires.ftc.teamcode.Stuff.Sigma.Kv;
import android.util.Size;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Stuff.ShooterConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@Configurable
public class Turret {
    double power1 = 0, power2 = 0;
    double allianceID = 20;
    public static long exposure = 2;
    public static int gain = 200;
    WebcamName webcam;
    double fx = 807.567, fy = 807.567, cx = 345.549, cy = 267.084;
    GoBildaPinpointDriver pp;
    public AprilTagProcessor tagProcessor;
    public VisionPortal visionPortal;
    public DcMotorEx rotate, shoot1, shoot2;
    public TelemetryManager telemetryM;
    PIDController pid = new PIDController(0.01,0,0.0002);
    PIDController controller = new PIDController(Kp,Ki,Kd);

    public enum AllienceState {
        RED,
        BLUE,
    }

    AllienceState state = AllienceState.BLUE;


    public Turret(DcMotorEx rotate, DcMotorEx shoot1, DcMotorEx shoot2, TelemetryManager telemetryM, WebcamName webcam,GoBildaPinpointDriver pp) {
        this.rotate = rotate;
        this.telemetryM = telemetryM;
        this.shoot2 = shoot2;
        this.shoot1 = shoot1;
        this.webcam = webcam;
        this.pp = pp;
        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot2.setDirection(DcMotorSimple.Direction.REVERSE);

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
                .enableLiveView(true)
                .build();

        PanelsCameraStream.INSTANCE.startStream(visionPortal, 30);
        visionPortal.stopStreaming();

    }

    public void update() {
        Odo odo = new Odo(pp);
        controller = new PIDController(Kp,Ki,Kd);
        vel1 = controller.calculate(shoot2.getVelocity(),ShooterConstants.fwVel(odo.deltaBLUE()));
        telemetryM.addData("Delta",odo.deltaBLUE());
        telemetryM.addData("target",ShooterConstants.fwVel(odo.deltaBLUE()));
        telemetryM.update();
        vel1 += Kv * rpm + Ks;
        shoot1.setPower(vel1);
        shoot2.setPower(vel1);
        rotate.setPower(pid.calculate(rotate.getCurrentPosition(),0) * 0.7);
        if (gm1.circleWasPressed()){
            odo.resetBlue();
        }
    }
    public void allienceUpdate() {
        switch (state) {
            case RED:
                allianceID = 24;
                telemetryM.addLine("RED");

                break;
            case BLUE:
                allianceID = 20;
                telemetryM.addLine("BLUE");
                break;
        }
        if (gm1.dpadRightWasPressed()) {
            state = AllienceState.BLUE;
        }
        if (gm1.dpadDownWasPressed()) {
            state = AllienceState.RED;
        }
    }
}

