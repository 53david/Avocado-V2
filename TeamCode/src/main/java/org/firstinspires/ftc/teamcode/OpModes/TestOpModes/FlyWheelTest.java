package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Stuff.PIDController;

@Configurable
@TeleOp(name = "New Test")
public class FlyWheelTest extends LinearOpMode {
    public static double vel = 0,Kp = 0, Ki = 0, Kd = 0 ;
    PIDController pid = new PIDController(Kp,Ki,Kd);
    DcMotorEx shoot1, shoot2;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void runOpMode(){
        hardwinit();
        waitForStart();
        while (opModeIsActive()){
            pid.setPidCoefficients(new PIDCoefficients(Kp,Ki,Kd));
            shoot1.setPower(pid.calculatePower(vel));
            shoot2.setPower(pid.calculatePower(vel));
            telemetryM.addData("Velocity",shoot1.getVelocity());
            telemetryM.update();
        }

    }
    public void hardwinit(){
        pid.setTargetPosition(1000);
        shoot1 = hardwareMap.get(DcMotorEx.class,"shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class,"shoot2");
        shoot2.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
