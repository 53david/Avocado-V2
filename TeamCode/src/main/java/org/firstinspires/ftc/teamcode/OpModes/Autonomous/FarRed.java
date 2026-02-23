package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static org.firstinspires.ftc.teamcode.Localizer.Constants.res;
import static org.firstinspires.ftc.teamcode.Stuff.Sigma.Ks;
import static org.firstinspires.ftc.teamcode.Stuff.Sigma.Kv;
import static org.firstinspires.ftc.teamcode.Stuff.Sigma.Kp;
import static org.firstinspires.ftc.teamcode.Stuff.Sigma.Ki;
import static org.firstinspires.ftc.teamcode.Stuff.Sigma.Kd;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.DriveTrain;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Outake;
import org.firstinspires.ftc.teamcode.Components.Storage;
import org.firstinspires.ftc.teamcode.Components.Turret;
import org.firstinspires.ftc.teamcode.Localizer.Constants;

@Autonomous (name = "Mascul Fioros Red")
public class FarRed extends LinearOpMode {
    private Follower follower;
    double vel1 =0;
    GoBildaPinpointDriver gobilda;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    public static double rpm = 1450;
    private DriveTrain chassis; private Intake intake; private CRServo servo; public static Turret turret;
    private Outake outake; Servo transfer; private Storage storage; ElapsedTime robotTimer = new ElapsedTime(),pathTimer = new ElapsedTime(); private ColorRangeSensor colorSensor;
    public static PIDCoefficients coefs = new PIDCoefficients(0.38247891 ,0, 0.03728480);
    DcMotorEx intakeMotor,rotate,leftFront,leftBack,rightBack,rightFront,shoot1,shoot2;
    WebcamName webcam1;
    double velocity = 200, power = 0.5;
    int xball = 0;
    public enum PathState{
        Start_ShootPos,
        Shoot_Station,
        Station_Shoot,
        Shoot_Transition,
        Transition_Ball,
        Ball_Shoot,
        Shoot_Station1,
        Station1_Shoot,
    };
    public enum RobotState{
        IDLE,
        INTAKE,
        SHOOT,
    };
    PIDController controller = new PIDController(Kp,Ki,Kd);
    org.firstinspires.ftc.teamcode.Stuff.PIDController pid = new org.firstinspires.ftc.teamcode.Stuff.PIDController(1,0,0.02);

    private PathState pathState;
    private RobotState robotState;
    private final Pose startPose = new Pose(88.000,8.000,Math.toRadians(90));
    private final Pose shootPose = new Pose(79.36037079953651,23.65237543453071,Math.toRadians(62));
    private final Pose station = new Pose (133.53650057937426,9.522595596755522,Math.toRadians(0));
    private final Pose transition = new Pose(79.36037079953651,36.867902665121655,Math.toRadians(0));
    private final Pose ball = new Pose(125.47856315179607,35.37427578215528,Math.toRadians(180));
    private PathChain StartShootPos,ShootStationPos,StationShootPos,ShootTransitionPos,TransitionBallPos,BallShootPos,ShootStationPos1,Station1ShootPos;
    public void buildPaths(){
        StartShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();
        ShootStationPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,station))
                .setLinearHeadingInterpolation(shootPose.getHeading(), station.getHeading())
                .build();
        StationShootPos =follower.pathBuilder()
                .addPath(new BezierLine(station,shootPose))
                .setLinearHeadingInterpolation(station.getHeading(),shootPose.getHeading())
                .build();
        ShootTransitionPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,transition))
                .setLinearHeadingInterpolation(shootPose.getHeading(), transition.getHeading())
                .build();
        TransitionBallPos = follower.pathBuilder()
                .addPath(new BezierLine(transition,ball))
                .setLinearHeadingInterpolation(transition.getHeading(),ball.getHeading())
                .build();
        BallShootPos = follower.pathBuilder()
                .addPath(new BezierLine(ball,shootPose))
                .setLinearHeadingInterpolation(ball.getHeading(),shootPose.getHeading())
                .build();
        ShootStationPos1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,station))
                .setLinearHeadingInterpolation(shootPose.getHeading(), station.getHeading())
                .build();
        Station1ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(station,shootPose))
                .setLinearHeadingInterpolation(station.getHeading(),shootPose.getHeading())
                .build();
    }
    public void pathUpdate () {
        telemetryM.addData("TIME", pathTimer.milliseconds());
        telemetryM.update();
        switch (pathState) {
            case Start_ShootPos:
                follower.followPath(StartShootPos);
                if (pathTimer.milliseconds()>1200) {
                    robotState = RobotState.SHOOT;
                    if (pathTimer.milliseconds()>3000) {
                        pathState = PathState.Shoot_Station;
                        robotState = RobotState.IDLE;
                        pathTimer.reset();
                    }
                }
                break;
            case Shoot_Station:
                follower.followPath(ShootStationPos);
                robotState = RobotState.INTAKE;
                if (pathTimer.milliseconds()>3000) {
                    pathState = PathState.Station_Shoot;
                    robotState = RobotState.IDLE;
                    pathTimer.reset();
                }
                break;
            case Station_Shoot:
                follower.followPath(StationShootPos);
                if (pathTimer.milliseconds()>1200) {
                    robotState = RobotState.SHOOT;
                    if (pathTimer.milliseconds()>3000) {
                        pathState = PathState.Shoot_Transition;
                        robotState = RobotState.IDLE;
                        pathTimer.reset();
                    }
                }
                break;
            case Shoot_Transition:
                follower.followPath(ShootTransitionPos);
                if (pathTimer.milliseconds()>1200) {
                    pathTimer.reset();
                    pathState = PathState.Transition_Ball;
                }
                break;
            case Transition_Ball:
                robotState = RobotState.INTAKE;
                follower.followPath(TransitionBallPos);
                if (pathTimer.milliseconds()>3000) {
                    pathState = PathState.Ball_Shoot;
                    robotState = RobotState.IDLE;
                    pathTimer.reset();
                }
                break;
            case Ball_Shoot:
                follower.followPath(BallShootPos);
                if (pathTimer.milliseconds()>1200) {
                    robotState = RobotState.SHOOT;
                    if (pathTimer.milliseconds()>3000) {
                        pathState = PathState.Shoot_Station1;
                        robotState = RobotState.IDLE;
                        pathTimer.reset();
                    }
                }
                break;
            case Shoot_Station1:
                robotState = RobotState.INTAKE;
                follower.followPath(ShootStationPos1);
                if (pathTimer.milliseconds()>3000) {
                    pathState = PathState.Station1_Shoot;
                    robotState = RobotState.IDLE;
                    pathTimer.reset();
                }
                break;
            case Station1_Shoot:
                follower.followPath(Station1ShootPos);
                if (pathTimer.milliseconds()>1200) {
                    robotState = RobotState.SHOOT;
                    if (pathTimer.milliseconds()>3000) {
                        pathState = PathState.Shoot_Station1;
                        robotState = RobotState.IDLE;
                        pathTimer.reset();
                    }
                }
                break;
        }
    }
    public void robotUpdate(){
        vel1 = controller.calculate(shoot2.getVelocity(),rpm);
        vel1 += Kv * rpm + Ks;
        shoot1.setPower(vel1);
        shoot2.setPower(vel1);
        rotate.setPower(pid.calculatePower(rotate.getCurrentPosition()/(384.5 * (130.0/34.0)) * Math.PI * 2.0));
        pid.setTargetPosition(Math.PI/180*0);
        switch (robotState){
            case IDLE:
                intakeMotor.setPower(0);
                transfer.setPosition(0.3);
                break;
            case SHOOT:
                intakeMotor.setPower(1);
                transfer.setPosition(0.05);
                break;

            case INTAKE:
                intakeMotor.setPower(1);
                transfer.setPosition(0.3);
                break;
        }
    }

    @Override
    public void runOpMode (){
        hardwinit();
        waitForStart();
        while (opModeIsActive()){
            pathUpdate();
            follower.update();
            robotUpdate();
            telemetryM.addData("Heading",gobilda.getHeading(AngleUnit.DEGREES));
            telemetryM.addData("X",gobilda.getPosX(DistanceUnit.INCH));
            telemetryM.addData("Y",gobilda.getPosY(DistanceUnit.INCH));
            telemetryM.update();
        }
    }
    public void hardwinit(){
        shoot1 = hardwareMap.get(DcMotorEx.class,"shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class,"shoot2");
        transfer = hardwareMap.get(Servo.class,"transfer");
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intake");
        Timer pathTimer = new Timer();
        gobilda = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        pathState = PathState.Start_ShootPos;
        gobilda.setHeading(90, AngleUnit.DEGREES);
        gobilda.setPosX(56.000, DistanceUnit.INCH);
        gobilda.setPosY(8.000,DistanceUnit.INCH);
        gobilda.setEncoderResolution(res, DistanceUnit.MM);

    }
}
