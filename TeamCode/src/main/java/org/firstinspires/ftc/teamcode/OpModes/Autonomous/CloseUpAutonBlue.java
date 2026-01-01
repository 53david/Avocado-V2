package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Components.DriveTrain;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Outake;
import org.firstinspires.ftc.teamcode.Components.Storage;
import org.firstinspires.ftc.teamcode.Components.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (name = "Mascul Fioros BLUE")
public class CloseUpAutonBlue extends LinearOpMode {
    private Follower follower;
    private DriveTrain chassis; private Intake intake; private CRServo servo; public static Turret turret;
    private Outake outake; Servo transfer; private Storage storage; ElapsedTime robotTimer,pathTimer; private NormalizedColorSensor colorSensor;
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
    };
    private PathState pathState;
    private RobotState robotState;
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
    public void pathUpdate (){
        switch (pathState){
            case Start_ShootPos:
                follower.followPath(StartShootPos,true);
                if (!follower.isBusy()) {
                    robotState = RobotState.SHOOT;
                    if (pathTimer.milliseconds()>5000) {
                        pathState = PathState.ShootPos_Ball1;
                    }
                }
                else {
                    robotState = RobotState.IDLE;
                    pathTimer.reset();
                }
                break;
            case ShootPos_Ball1:
                follower.followPath(ShootBallPos,true);
                robotState = RobotState.INTAKE;
                if (!follower.isBusy()) {
                    pathState = PathState.Ball1_ShootPos;
                    pathTimer.reset();
                }

                break;
            case Ball1_ShootPos:
                follower.followPath(BallShootPos,true);
                if (!follower.isBusy()) {
                    robotState = RobotState.SHOOT;
                    if (pathTimer.milliseconds()>5000) {
                        pathState = PathState.ShootPos_Gate;
                    }
                }
                else {
                    robotState = RobotState.IDLE;
                    pathTimer.reset();
                }
                break;
            case ShootPos_Gate:
                follower.followPath(ShootGatePos);
                robotState = RobotState.INTAKE;
                if (!follower.isBusy()) {
                    pathState = PathState.Gate_ShootPos;
                    pathTimer.reset();
                }
                break;
            case Gate_ShootPos:
                follower.followPath(GateShootPos);
                if (!follower.isBusy()) {
                    robotState = RobotState.SHOOT;
                    if (pathTimer.milliseconds()>5000) {
                        pathState = PathState.Gate_ShootPos;
                    }
                }
                else {
                    robotState = RobotState.IDLE;
                    pathTimer.reset();
                }
                break;
        }

    }
    public void robotUpdate(){

        shoot1.setVelocity(velocity);
        shoot2.setVelocity(velocity);
        intakeMotor.setPower(power);

        switch (robotState){
            case IDLE:
                velocity = 200;
                power = 0.5;
                robotTimer.reset();
                xball = 0;
                break;
            case SHOOT:
                velocity = 1600;
                power = 0.5;
                if (robotTimer.milliseconds()>1000 && xball<3) {
                    outake.activate(); xball++;
                    robotTimer.reset();
                }
                else if (xball == 3){
                    robotState = RobotState.IDLE;
                }
                break;
            case INTAKE:
                velocity = 200;
                power = 1;
                robotTimer.reset();
                xball = 0;
                break;
        }
    }

    @Override
    public void runOpMode (){
        hardwinit();
        waitForStart();
        while (opModeIsActive()){
            pathUpdate();
            robotUpdate();
            follower.update();
        }

    }
    public void hardwinit(){

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        pathState = PathState.Start_ShootPos;
        transfer = hardwareMap.get(Servo.class,"transfer");
        shoot1 = hardwareMap.get(DcMotorEx.class,"shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class,"shoot2");
        rotate = hardwareMap.get(DcMotorEx.class,"rotate");
        webcam1 = hardwareMap.get(WebcamName.class,"webcam1");
        chassis = new DriveTrain(leftFront,rightFront,leftBack,rightBack);
        chassis.init(hardwareMap);
        intake = new Intake(intakeMotor);
        outake = new Outake(shoot1,shoot2,rotate,transfer,telemetry);
        storage = new Storage(servo,intakeMotor,colorSensor,telemetry);
        storage.init(hardwareMap);
        turret = new Turret(rotate,webcam1,telemetry);
        storage.turner.setPidCoefficients(coefs);

    }
}
