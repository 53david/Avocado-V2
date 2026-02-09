package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Components.Odo;
import org.firstinspires.ftc.teamcode.Components.Turret;
@Configurable
@TeleOp(name="Test")
public class TurretTuning extends LinearOpMode {
    TelemetryManager telemetry1;
    Turret turret;
    Odo odo;
    DcMotorEx shoot1,shoot2,rotate;
    GoBildaPinpointDriver pp;
    @Override
    public void runOpMode(){
        hardwinit();
        waitForStart();
        while (opModeIsActive()){
            turret.update();
            telemetry1.addData("Error",shoot1.getVelocity());

        }
    }
    public void hardwinit(){

        pp = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        shoot1 = hardwareMap.get(DcMotorEx.class,"shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class,"shoot2");
        rotate = hardwareMap.get(DcMotorEx.class,"rotate");
        odo = new Odo(pp);
        turret = new Turret(rotate,shoot1,shoot2,telemetry1);

    }
}
