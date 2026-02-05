package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import static org.firstinspires.ftc.teamcode.Stuff.FuckassPID.P;
import static org.firstinspires.ftc.teamcode.Stuff.FuckassPID.I;
import static org.firstinspires.ftc.teamcode.Stuff.FuckassPID.D;
import static org.firstinspires.ftc.teamcode.Stuff.FuckassPID.vel;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Stuff.PIDController;
import org.firstinspires.ftc.teamcode.Stuff.ShooterConstants;


@Configurable
@TeleOp (name="Fuckass cod")
public class FuckassTuner extends LinearOpMode {
    double power = 0;
    DcMotorEx shoot1,shoot2;
    TelemetryManager telemetrym = PanelsTelemetry.INSTANCE.getTelemetry();
    PIDController cont = new PIDController(P,I,D);
    PIDCoefficients coef = new PIDCoefficients(P,I,D);
    @Override
        public void runOpMode(){
                hardwinit();
                waitForStart();
                while (opModeIsActive()){
                    cont.setPidCoefficients(coef);
                    power+=cont.calculatePower(shoot2.getVelocity(),vel);
                    power = Math.min(1,power);
                    power = Math.max(-1,power);
                    shoot1.setPower(power);
                    shoot2.setPower(power);
                    telemetrym.addData("Error",shoot1.getVelocity());
                    telemetrym.addData("Error",shoot2.getVelocity());
                    telemetrym.update();
                }
        }
        public void hardwinit(){

            shoot2 = hardwareMap.get(DcMotorEx.class,"shoot2");
            shoot1 = hardwareMap.get(DcMotorEx.class,"shoot1");
            shoot1.setDirection(DcMotorSimple.Direction.REVERSE);
        }

}
