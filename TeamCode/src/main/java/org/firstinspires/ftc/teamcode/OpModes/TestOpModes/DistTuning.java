package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;


import static org.firstinspires.ftc.teamcode.Stuff.ShooterConstants.Kd;
import static org.firstinspires.ftc.teamcode.Stuff.ShooterConstants.Ki;
import static org.firstinspires.ftc.teamcode.Stuff.ShooterConstants.Kp;
import static org.firstinspires.ftc.teamcode.Stuff.ShooterConstants.Ks;
import static org.firstinspires.ftc.teamcode.Stuff.ShooterConstants.Kv;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Components.Odo;

@TeleOp
@Disabled
public class DistTuning extends LinearOpMode {
    public static double rpm = 0;
    public static IMU imu;
    TelemetryManager telemetryM;
    double Voltage;
    GoBildaPinpointDriver pp;
    Odo odo;
    PIDController controller = new PIDController(Kp,Ki,Kd);
    DcMotorEx shoot1,shoot2;
    @Override
    public void runOpMode(){
        hardwinit();
        waitForStart();
        while (opModeIsActive()){
            double vel1 = controller.calculate(-shoot2.getVelocity(),rpm);
            vel1 += Kv * rpm + Ks;
            vel1 *= Voltage;
            shoot1.setPower(vel1);
            shoot2.setPower(vel1);
            telemetryM.addData("X",Odo.getCurrentPosition().x);
            telemetryM.addData("Y",Odo.getCurrentPosition().y);
            telemetryM.addData("H",Odo.getCurrentPosition().h);
            telemetryM.addData("as",Odo.distance());
            telemetryM.update();
        }
    }
    public void hardwinit(){
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));
        odo = new Odo(pp);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Voltage = 12.0/hardwareMap.getAll(VoltageSensor.class).get(0).getVoltage();
        shoot1 = hardwareMap.get(DcMotorEx.class,"shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class,"shoot2");
        pp = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
    }
}
