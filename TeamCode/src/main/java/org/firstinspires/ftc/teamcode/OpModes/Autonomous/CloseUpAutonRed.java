package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Components.DriveTrain;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Outake;
import org.firstinspires.ftc.teamcode.Components.Storage;
import org.firstinspires.ftc.teamcode.Components.Turret;
import org.firstinspires.ftc.teamcode.Localizer.Constants;

@Autonomous(name = "Mascul Fioros RED")
public class CloseUpAutonRed extends LinearOpMode {
    private Follower follower;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    private DriveTrain chassis; private Intake intake; private CRServo servo; public static Turret turret;
    private Outake outake; Servo transfer; private Storage storage; ElapsedTime robotTimer,pathTimer; private ColorRangeSensor colorSensor;
    Timer timer2;
    public static PIDCoefficients coefs = new PIDCoefficients(0.4 ,0, 0.002);
    DcMotorEx intakeMotor,rotate,leftFront,leftBack,rightBack,rightFront,shoot1,shoot2;
    WebcamName webcam1;
    double velocity = 200, power = 0.5;
    int xball = 0;
    public enum PathState{
        Start_ShootPos,
        ShootPos_Ball1,
        Ball1_ShootPos,
        ShootPos_Gate,
        Gate_ShootPos,
    }
    public enum RobotState{
        INTAKE,
        SHOOT,
        IDLE,
    }
    private PathState pathState;
    private RobotState robotState;
    private final Pose startPose = new Pose(121.94392523364486,124.41121495327104,Math.toRadians(37));
    private final Pose shootPose = new Pose(82.01869158878506,85.57009345794393,Math.toRadians(37));
    private final Pose ballPose = new Pose (125.41121495327104,83.21495327102804,Math.toRadians(0));
    private final Pose gatePose = new Pose(131.84112149532712,64.82242990654204,Math.toRadians(15));
    private final Pose parkPose = new Pose(30.72897196261682,93.53271028037383,Math.toRadians(143));

    private PathChain StartShootPos,ShootBallPos,BallShootPos,ShootGatePos,GateShootPos;
    public void buildPaths(){
        StartShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();
        ShootBallPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,ballPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(),ballPose.getHeading())
                .build();
        BallShootPos = follower.pathBuilder()
                .addPath(new BezierLine(ballPose,shootPose))
                .setLinearHeadingInterpolation(ballPose.getHeading(),shootPose.getHeading())
                .build();
        ShootGatePos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,gatePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(),gatePose.getHeading())
                .build();
        GateShootPos = follower.pathBuilder()
                .addPath(new BezierLine(gatePose,shootPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(),shootPose.getHeading())
                .build();
    }
    public void pathUpdate (){
        switch (pathState){
            case Start_ShootPos:
                follower.followPath(StartShootPos,1,true);
                if (!follower.isBusy()) {
                    pathState = PathState.ShootPos_Ball1;
                }
                break;
            case ShootPos_Ball1:
                follower.followPath(ShootBallPos,1,true);
                if (!follower.isBusy()) {
                    pathState = PathState.Ball1_ShootPos;
                }
                break;
            case Ball1_ShootPos:
                follower.followPath(BallShootPos,1,true);
                if (!follower.isBusy()) {
                        pathState = PathState.ShootPos_Gate;
                }
                break;
            case ShootPos_Gate:
                follower.followPath(ShootGatePos,1,true);
                if (!follower.isBusy()) {
                    pathState = PathState.Gate_ShootPos;
                }
                break;
            case Gate_ShootPos:
                follower.followPath(GateShootPos,1,true);
                if (!follower.isBusy()) {
                        pathState = PathState.Gate_ShootPos;
                }
                break;
        }

    }

    @Override
    public void runOpMode(){
        waitForStart();
        hardwinit();
        while (opModeIsActive()){
            follower.update();
            pathUpdate();
        }
    }
    public void hardwinit(){
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        pathState = PathState.Start_ShootPos;
        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
        MotorConfigurationType m= leftFront.getMotorType();
        m.setAchieveableMaxRPMFraction(1);
        leftFront.setMotorType(m);
        rightFront.setMotorType(m);
        leftBack.setMotorType(m);
        rightBack.setMotorType(m);
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intake");
        shoot1 = hardwareMap.get(DcMotorEx.class,"shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class,"shoot2");
        rotate = hardwareMap.get(DcMotorEx.class,"rotate");
        transfer = hardwareMap.get(Servo.class,"transfer");
        servo = hardwareMap.get(CRServo.class,"servo");
        colorSensor = hardwareMap.get(ColorRangeSensor.class,"colorSensor");
        webcam1 = hardwareMap.get(WebcamName.class,"webcam1");
        rotate = hardwareMap.get(DcMotorEx.class,"rotate");
        chassis = new DriveTrain(leftFront,rightFront,leftBack,rightBack);
        intake = new Intake(intakeMotor,transfer);
        outake = new Outake(shoot1,shoot2);
        storage = new Storage(transfer,servo,intakeMotor,colorSensor,telemetry);
        turret = new Turret(rotate,shoot1,shoot2,webcam1,telemetryM);
        storage.turner.setPidCoefficients(coefs);
    }
}
