package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
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

@Autonomous(name = "Mascul Puturos RED")
public class NoLocalizerAutonRed extends LinearOpMode {
    private Follower follower; private WebcamName webcam1;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    private DriveTrain chassis; private Intake intake; private CRServo servo; public static Turret turret;
    private Outake outake; Servo transfer; private Storage storage; boolean ok1 = true,ok2 = true,ok3 = true;
    ElapsedTime timer = new ElapsedTime();
    private ColorRangeSensor colorSensor;
    double velocity = 200,power = 0.5; int xball = 0;
    public static PIDCoefficients coefs = new PIDCoefficients(0.4 ,0, 0.002);
    DcMotorEx intakeMotor,rotate,leftFront,leftBack,rightBack,rightFront,shoot1,shoot2;
    @Override
    public void runOpMode(){
        waitForStart();
        hardwinit();
        while (opModeIsActive()){
            intakeMotor.setPower(0.5);

        }
    }
    public void hardwinit(){
        timer.reset();
        power = 0.6;
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
