package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;
import static org.firstinspires.ftc.teamcode.Wrappers.ShooterConstants.Kd;
import static org.firstinspires.ftc.teamcode.Wrappers.ShooterConstants.Ki;
import static org.firstinspires.ftc.teamcode.Wrappers.ShooterConstants.Kp;
import static org.firstinspires.ftc.teamcode.Wrappers.ShooterConstants.Ks;
import static org.firstinspires.ftc.teamcode.Wrappers.ShooterConstants.Kv;


import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Configurable
@Disabled
@TeleOp (name="Formula")
public class BetterFormula extends LinearOpMode {
    DcMotorEx shoot1,shoot2,m2;
    public static double rpm = 1200, x = 0;
    double vel1 = 0;
    TelemetryManager telemetryM;
    PIDController controller = new PIDController(Kp, Ki, Kd);
    PIDCoefficients coef = new PIDCoefficients(Kp,Ki,Kd);
    @Override
    public void runOpMode()throws InterruptedException {
        hardwinit();
        waitForStart();
        while (opModeIsActive()) {
            controller = new PIDController(Kp,Ki,Kd);
            vel1 = controller.calculate(shoot2.getVelocity(),rpm);
            vel1 += Kv * rpm + Ks;
            shoot1.setPower(vel1);
            shoot2.setPower(vel1);
            telemetryM.addData("Error", shoot2.getVelocity());
            telemetryM.update();
            m2.setPower(x);


        }
    }
    public void hardwinit(){
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        shoot2 = hardwareMap.get(DcMotorEx.class,"shoot2");
        shoot1 = hardwareMap.get(DcMotorEx.class,"shoot1");
        shoot2.setDirection(DcMotorSimple.Direction.REVERSE);
        m2 = hardwareMap.get(DcMotorEx.class,"intake");


    }
}
