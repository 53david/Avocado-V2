package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Components.DriveTrain;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Outake;
import org.firstinspires.ftc.teamcode.Components.Storage;
import org.firstinspires.ftc.teamcode.Components.Vision;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous (name = "Mascul Fioros")
public class Auto extends LinearOpMode {
    private Follower follower;
    private Timer pTimer,opTimer;
    private ElapsedTime timer;
    private DriveTrain chassis; private Intake intake; private CRServo servo; public static Vision vision;
    private Outake outake; Servo transfer; private Storage storage;
    private NormalizedColorSensor colorSensor;
    private VisionPortal visionPortal; double nr = 0;
    double velocity = 0;
    private AprilTagProcessor tagProcessor;
    public static PIDCoefficients coefs = new PIDCoefficients(0.4 ,0, 0.002);
    DcMotorEx intakeMotor,rotate,leftFront,leftBack,rightBack,rightFront,shoot1,shoot2;
    WebcamName webcam1;
    RevColorSensorV3 colorSensor1,colorSensor2;



    public enum PathState{
        Start_ShootPos,
        ShootPos_Ball1,
        Ball1_ShootPos,
        ShootPos_Gate,
        Gate_ShootPos,
    }
    public enum RobotState{
        IDLE,
        INTAKE,
        SHOOT,
        INDEX,
    }
    RobotState robotState;
    PathState pathState;
    private final Pose startPose = new Pose(21.53271028037383,122.69158878504673,Math.toRadians(143));
    private final Pose shootPose = new Pose(62.80373831775701,84.11214953271028,Math.toRadians(143));
    private final Pose ballPose = new Pose (13.906542056074766,83.21495327102804,Math.toRadians(182));
    private final Pose gatePose = new Pose(13.457943925233645,64.82242990654204,Math.toRadians(200));
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
    public void stateUpdate (){
        switch (pathState){
            case Start_ShootPos:
                follower.followPath(StartShootPos,true);
                if (!follower.isBusy()) {
                    robotState = robotState.SHOOT;
                }
                else {
                    robotState = robotState.IDLE;
                }
                break;
            case ShootPos_Ball1:
                follower.followPath(ShootBallPos,true);
                robotState = robotState.INTAKE;
                if (!follower.isBusy()) {
                    pathState = pathState.Ball1_ShootPos;
                }
                break;
            case Ball1_ShootPos:
                follower.followPath(BallShootPos,true);
                if (!follower.isBusy()) {
                    robotState = robotState.SHOOT;
                }
                else robotState = robotState.IDLE;
                break;
            case ShootPos_Gate:
                follower.followPath(ShootGatePos);
                robotState = robotState.INTAKE;
                break;
            case Gate_ShootPos:
                follower.followPath(GateShootPos);
                if (!follower.isBusy()){
                    robotState = robotState.SHOOT;
                }
                else {
                    robotState = robotState.IDLE;
                }
                break;
        }
        switch (robotState){
            case IDLE:
                velocity = 200;
                intakeMotor.setPower(0);
                shoot1.setVelocity(velocity);
                shoot2.setVelocity(velocity);
                nr = 0;
                timer.reset();
                break;
            case INDEX:
                if (nr<1){
                    storage.Turn120();
                }
                nr++;
                break;
            case SHOOT:
                velocity = 1600;
                if (timer.milliseconds()>500)
                    outake.activate();
                shoot1.setVelocity(velocity);
                shoot2.setVelocity(velocity);

        }
    }

    @Override
    public void runOpMode(){
        waitForStart();
        hardwinit();
        while (opModeIsActive()){
            stateUpdate();
            follower.update();
        }

    }
    public void hardwinit(){
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        pathState = pathState.Start_ShootPos;
        servo = hardwareMap.get(CRServo.class,"servo");
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intake");
        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
        MotorConfigurationType m= leftFront.getMotorType();
        m.setAchieveableMaxRPMFraction(1);
        leftFront.setMotorType(m);
        rightFront.setMotorType(m);
        leftBack.setMotorType(m);
        rightFront.setMotorType(m);
        transfer = hardwareMap.get(Servo.class,"transfer");
        shoot1 = hardwareMap.get(DcMotorEx.class,"shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class,"shoot2");
        rotate = hardwareMap.get(DcMotorEx.class,"rotate");
        webcam1 = hardwareMap.get(WebcamName.class,"webcam1");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class,"colorSensor");
        chassis = new DriveTrain(leftFront,rightFront,leftBack,rightBack);
        intake = new Intake(intakeMotor);
        outake = new Outake(shoot1,shoot2,rotate,transfer,telemetry);
        storage = new Storage(servo,intakeMotor,intakeMotor,colorSensor,telemetry);
        vision = new Vision(rotate,webcam1,telemetry);
        storage.turner.setPidCoefficients(coefs);
    }
}
