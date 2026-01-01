package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.pedropathing.follower.Follower;
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

@Autonomous
public class RetardedAutonomous extends LinearOpMode {
    private Follower follower;
    private DriveTrain chassis; private Intake intake; private CRServo servo; public static Vision vision;
    private Outake outake; Servo transfer; private Storage storage; boolean ok1 = true,ok2 = true,ok3 = true;
    ElapsedTime timer;
    private NormalizedColorSensor colorSensor;
    double velocity = 200,power = 0.5; int xball = 0;
    public static PIDCoefficients coefs = new PIDCoefficients(0.4 ,0, 0.002);
    DcMotorEx intakeMotor,rotate,leftFront,leftBack,rightBack,rightFront,shoot1,shoot2;
    @Override
    public void runOpMode(){
        waitForStart();
        hardwinit();
        while (opModeIsActive()){
            shoot1.setPower(power);
            shoot2.setPower(power);
            if (timer.milliseconds()<350) {
                chassis.BackWards();
            }
            if(timer.milliseconds()<1000 && ok1){
                ok1 = false ;
                outake.activate();
            }
            if(timer.milliseconds()<1500 && ok2){
                ok2 = false ;
                outake.activate();
            }
            if (timer.milliseconds()<2000 && ok3){
                ok3 = false;
                outake.activate();
            }
            if (timer.milliseconds()<2350){
                chassis.Strafe();
                power = 0;

            }


        }
    }
    public void hardwinit(){
        timer.reset();
        power = 0.6;
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
        colorSensor = hardwareMap.get(NormalizedColorSensor.class,"colorSensor");
        chassis = new DriveTrain(leftFront,rightFront,leftBack,rightBack);
        intake = new Intake(intakeMotor);
        outake = new Outake(shoot1,shoot2,rotate,transfer,telemetry);
        storage = new Storage(servo,intakeMotor,intakeMotor,colorSensor,telemetry);
        storage.turner.setPidCoefficients(coefs);
    }
}
