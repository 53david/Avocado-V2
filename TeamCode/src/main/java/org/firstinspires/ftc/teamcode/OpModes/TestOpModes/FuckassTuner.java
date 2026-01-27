package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import static org.firstinspires.ftc.teamcode.Stuff.FuckassPID.P;
import static org.firstinspires.ftc.teamcode.Stuff.FuckassPID.I;
import static org.firstinspires.ftc.teamcode.Stuff.FuckassPID.D;
import static org.firstinspires.ftc.teamcode.Stuff.FuckassPID.vel;

import com.bylazar.configurables.annotations.Configurable;
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
    PIDController cont = new PIDController(P,I,D);
    PIDCoefficients coef = new PIDCoefficients(P,I,D);
    @Override
        public void runOpMode(){
                hardwinit();
                waitForStart();
                while (opModeIsActive()){
                    cont.setPidCoefficients(coef);
                    power+=cont.calculatePower(shoot1.getVelocity(),vel);
                    power = Math.max(1,power);
                    power = Math.max(-1,power);
                    shoot1.setPower(power);
                    shoot2.setPower(power);
                }
        }
        public void hardwinit(){

            shoot2 = hardwareMap.get(DcMotorEx.class,"shoot2");
            shoot1 = hardwareMap.get(DcMotorEx.class,"shoot1");
            shoot1.setDirection(DcMotorSimple.Direction.REVERSE);
        }

}
