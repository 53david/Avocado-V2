package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
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

@Autonomous (name = "Mascul Inflacarat BLUE")
public class FarAutonBlue extends LinearOpMode {
    private Follower follower;
    private DriveTrain chassis; private Intake intake; private CRServo servo; public static Turret turret;
    private Outake outake; Servo transfer; private Storage storage;
    ElapsedTime pathTimer,robotTimer;
    private WebcamName webcam1;
    private ColorRangeSensor colorSensor;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    double velocity = 200,power = 0.5; int xball = 0;
    public static PIDCoefficients coefs = new PIDCoefficients(0.4 ,0, 0.002);
    DcMotorEx intakeMotor,rotate,leftFront,leftBack,rightBack,rightFront,shoot1,shoot2;
    private final Pose startPose = new Pose(62.30204081632653,9.11020408163265,Math.toRadians(90));
    private final Pose shootPose = new Pose(62.30204081632653,24.97959183673469,Math.toRadians(115));
    private final Pose ballPose = new Pose(22.628571428571426,35.85306122448979,Math.toRadians(180));
    private final Pose playerPose = new Pose(9.69795918367347,10.873469387755105,Math.toRadians(200));
    private PathChain StartShootPos,ShootBallPos,BallShootPos,ShootPlayerPos,PlayerShootPos;
    public enum PathState{
        Start_Shoot,
        Shoot_Ball,
        Ball_Shoot,
        Shoot_Player,
        Player_Shoot,
    }
    public enum RobotState{
        INTAKE,
        SHOOT,
        IDLE,
    }
    private RobotState robotState;
    private PathState pathState;
    public void buildPaths(){
        StartShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();
        ShootBallPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,ballPose))
                .addPath(new BezierCurve())
                .setLinearHeadingInterpolation(shootPose.getHeading(),ballPose.getHeading())
                .build();
        BallShootPos = follower.pathBuilder()
                .addPath(new BezierLine(ballPose,shootPose))
                .setLinearHeadingInterpolation(ballPose.getHeading(),shootPose.getHeading())
                .build();
        ShootPlayerPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,playerPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(),playerPose.getHeading())
                .build();
        PlayerShootPos = follower.pathBuilder()
                .addPath(new BezierLine(playerPose,shootPose))
                .setLinearHeadingInterpolation(playerPose.getHeading(),shootPose.getHeading())
                .build();
    }
    public void pathUpdate(){
        switch (pathState){
            case Start_Shoot:
                follower.followPath(StartShootPos,1,true);
                if (!follower.isBusy()){
                    robotState = RobotState.SHOOT;
                    if (pathTimer.milliseconds()>5000){
                        pathState = PathState.Shoot_Ball;
                    }
                }
                else {
                    robotState = RobotState.IDLE;
                    pathTimer.reset();
                }
                break;
            case Shoot_Ball:
                follower.followPath(ShootBallPos,1,true);
                if (!follower.isBusy()){
                    pathState = PathState.Ball_Shoot;
                    pathTimer.reset();
                }
                break;
            case Ball_Shoot:
                follower.followPath(BallShootPos,1,true);
                if (!follower.isBusy()){
                    robotState = RobotState.SHOOT;
                    if (pathTimer.milliseconds()>5000) {
                        pathState = PathState.Shoot_Player;
                    }
                }
                else {
                    pathTimer.reset();
                    robotState = RobotState.IDLE;
                }
                break;
            case Shoot_Player:
                follower.followPath(ShootPlayerPos,1,true);
                robotState = RobotState.INTAKE;
                if (!follower.isBusy()){
                    pathState = PathState.Player_Shoot;
                    pathTimer.reset();
                }
                break;
            case Player_Shoot:
                follower.followPath(PlayerShootPos,1,true);
                if (!follower.isBusy()){
                    robotState = RobotState.SHOOT;
                    if (pathTimer.milliseconds()>5000 ) {
                        pathState = PathState.Shoot_Player;
                    }
                }
                else {
                    pathTimer.reset();
                    robotState = RobotState.IDLE;
                }
                break;
        }
    }

    @Override
    public void runOpMode(){
        waitForStart();
        hardwinit();
        while (opModeIsActive()){
            pathUpdate();
            follower.update();
        }
    }
    public void hardwinit(){

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        pathState = PathState.Start_Shoot;
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
