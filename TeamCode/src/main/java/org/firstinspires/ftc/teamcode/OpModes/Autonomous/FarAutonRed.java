package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
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

@Autonomous (name = "Mascul Inflacarat RED")
public class FarAutonRed extends LinearOpMode {
    private Follower follower; private WebcamName webcam1;
    private DriveTrain chassis; private Intake intake; private CRServo servo; public static Turret turret;
    private Outake outake; Servo transfer; private Storage storage;
    ElapsedTime pathTimer,robotTimer;
    private NormalizedColorSensor colorSensor;
    double velocity = 200,power = 0.5; int xball = 0;
    public static PIDCoefficients coefs = new PIDCoefficients(0.4 ,0, 0.002);
    DcMotorEx intakeMotor,rotate,leftFront,leftBack,rightBack,rightFront,shoot1,shoot2;
    private final Pose startPose = new Pose(82.9158878504673,6.654205607476635,Math.toRadians(90));
    private final Pose shootPose = new Pose(83.14018691588785,24.97959183673469,Math.toRadians(75));
    private final Pose ballPose = new Pose(123.21495327102804,36.54205607476635,Math.toRadians(15));
    private final Pose playerPose = new Pose(136.25233644859813,8.130841121495326,Math.toRadians(0));
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
                follower.followPath(StartShootPos,true);
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
                follower.followPath(ShootBallPos,true);
                robotState = RobotState.INTAKE;
                if (!follower.isBusy()){
                    pathState = PathState.Ball_Shoot;
                    pathTimer.reset();
                }
                break;
            case Ball_Shoot:
                follower.followPath(BallShootPos,true);
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
                follower.followPath(ShootPlayerPos,true);
                robotState = RobotState.INTAKE;
                if (!follower.isBusy()){
                    pathState = PathState.Player_Shoot;
                    pathTimer.reset();
                }
                break;
            case Player_Shoot:
                follower.followPath(PlayerShootPos,true);
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
    public void runOpMode(){
        waitForStart();
        hardwinit();
        while (opModeIsActive()){
            follower.update();
            pathUpdate();
            robotUpdate();
        }
    }
    public void hardwinit(){
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        pathState = PathState.Start_Shoot;
        robotState = RobotState.IDLE;
        chassis = new DriveTrain(leftFront,rightFront,leftBack,rightBack);
        intake = new Intake(intakeMotor);
        outake = new Outake(shoot1,shoot2,rotate,transfer,telemetry);
        storage = new Storage(servo,intakeMotor,colorSensor,telemetry);
        turret = new Turret(rotate,webcam1,telemetry);
        chassis.init(hardwareMap);
        intake.init(hardwareMap);
        outake.init(hardwareMap);
        storage.init(hardwareMap);
        storage.turner.setPidCoefficients(coefs);
        storage.turner.setPidCoefficients(coefs);
    }
}
