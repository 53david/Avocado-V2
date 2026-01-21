package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;
import static org.firstinspires.ftc.teamcode.Stuff.MecanumPID.P;
import static org.firstinspires.ftc.teamcode.Stuff.MecanumPID.I;
import static org.firstinspires.ftc.teamcode.Stuff.MecanumPID.D;
import static org.firstinspires.ftc.teamcode.Stuff.MecanumPID.F;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@Configurable
@TeleOp(name = "Mecanum Tuner")
public class MecanumTuner extends LinearOpMode {
    DcMotorEx leftFront,leftBack,rightBack,rightFront;
    private PIDFCoefficients coef = new PIDFCoefficients(P,I,D,F);
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException{
        hardwinit();
        waitForStart();
        while (opModeIsActive()){
            double y = gm1.left_stick_y;
            double x = -gm1.left_stick_x * 1.1;
            double rx = gm1.left_trigger - gm1.right_trigger;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            leftFront.setVelocity(frontLeftPower*1500);
            leftBack.setVelocity(backLeftPower*1500);
            rightFront.setVelocity(frontRightPower*1500);
            rightBack.setVelocity(backRightPower*1500);
            telemetryM.addData("error",leftFront.getVelocity());
        }
    }
    public void hardwinit(){

        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
        MotorConfigurationType m= leftFront.getMotorType();
        m.setAchieveableMaxRPMFraction(1);
        leftFront.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,coef);
        rightFront.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,coef);
        leftBack.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,coef);
        rightBack.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,coef);
        leftFront.setMotorType(m);
        rightFront.setMotorType(m);
        leftBack.setMotorType(m);
        rightBack.setMotorType(m);

    }
}
