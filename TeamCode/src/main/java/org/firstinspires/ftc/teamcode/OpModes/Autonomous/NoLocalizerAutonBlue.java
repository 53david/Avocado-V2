package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.pedropathing.follower.Follower;
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

@Autonomous(name = "Mascul Puturos BLUE")
public class NoLocalizerAutonBlue extends LinearOpMode {
    private Follower follower; private WebcamName webcam1;
    private DriveTrain chassis; private Intake intake; private CRServo servo; public static Turret turret;
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
            intakeMotor.setPower(0.5);
            if (timer.milliseconds()<350) {
                chassis.BackWards();
            }
            else if(timer.milliseconds()<1000 && ok1){
                ok1 = false ;
                storage.Turn120();
                outake.activate();
            }
            else if(timer.milliseconds()<1500 && ok2){
                ok2 = false ;
                storage.Turn120();
                outake.activate();
            }
            else if (timer.milliseconds()<2000 && ok3){
                ok3 = false;
                storage.Turn120();
                outake.activate();
            }
            else if (timer.milliseconds()<2350){
                chassis.StrafeRight();
                power = 0;
            }
        }
    }
    public void hardwinit(){
        timer.reset();
        power = 0.6;
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
    }
}
