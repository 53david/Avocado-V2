package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Stuff.PIDController;
public class Storage {
    private final CRServo revolver; private RevColorSensorV3 v8;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime time1 = new ElapsedTime();ElapsedTime time2 = new ElapsedTime();
    ElapsedTime time3 = new ElapsedTime();
    public static double a = 8;
    ElapsedTime time = new ElapsedTime(); int nr = 0 , prevnr;
    double p = 1.0, t=2000; boolean ok = false,ok1 = true,ok2 = true,ok3=true; Gamepad gamepad2;
    boolean next = false, prevnext = false;
    DcMotorEx encoder; Telemetry telemetry; ColorRangeSensor colorSensor;
    float x = 3 ; Servo servo;
    public PIDController turner;
    public double Target = 0;

    int n=0;
    enum ColorState {
        IDLE,
        GREEN,
        PURPLE,
        SHOOT,
    }

    ColorState state;

    private DcMotorEx motor;ShootState shootState;

    public Storage (Servo servo,CRServo revolver, DcMotorEx encoder, ColorRangeSensor colorSensor, Telemetry telemetry){
        this.servo = servo;
        this.revolver = revolver;
        this.colorSensor=colorSensor;
        this.telemetry=telemetry;
        this.encoder = encoder;
        shootState = ShootState.IDLE;
        colorSensor.setGain(x);
        state = ColorState.IDLE;
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turner = new PIDController(0,0,0);
    }

    enum ShootState{
        IDLE,
        SHOOT,
    }
    public double FromTicksToDegrees(){
        return encoder.getCurrentPosition() / 8192.0 * Math.PI * 2.0;
    }

    public void Turn120(){
        Target += Math.PI * 2 / 3;
        turner.setTargetPosition(Target);
    }

    public void update()
    {
        revolver.setPower(turner.calculatePower(FromTicksToDegrees()));
    }
    public void Index(){
        control();
        getColor();
        switch (state){
            case IDLE:
                timer.reset();
                time1.reset();
                time2.reset();
                break;
            case GREEN:
                if (nr < 3) {
                    Turn120();
                    nr++;
                }
                state = ColorState.IDLE;

                break;
            case PURPLE:
                if (nr < 3) {
                    Turn120();
                    nr++;
                }
                state = ColorState.IDLE;
                break;
            case SHOOT:
                if (timer.milliseconds() < 900 && ok){
                    Turn120();
                    ok = false;
                }
                if (timer.milliseconds()>700 && timer.milliseconds()<1200){
                    servo.setPosition(0.3);
                }
                if (timer.milliseconds()>1200 && timer.milliseconds()<1700){
                    servo.setPosition(0.65);
                }
                if (timer.milliseconds()>1700){
                    ok=true; a++;
                    timer.reset();
                }
                if (a>3){
                    a=1;
                    state = ColorState.IDLE;
                }

                break;
        }
        telemetry.update();

    }
    public void control(){
        if (gm1.circleWasPressed()){
            state = ColorState.SHOOT;
        }
    }
    public void getColor(){
        float red,green,blue;
        NormalizedRGBA color = colorSensor.getNormalizedColors();
        red = color.red/color.alpha;
        green = color.green/color.alpha;
        blue = color.blue/color.alpha;


        if (state != ColorState.SHOOT && time.milliseconds()>500) {
            if (DetectColor() == 1 && !ok) {
                state = ColorState.GREEN;
                ok = true;

            } else if (DetectColor() == 2 && !ok) {
                state = ColorState.PURPLE;
                ok = true;

            } else {
                ok = false;
                state = ColorState.IDLE;
            }
            time.reset();
        }

    }
    public int DetectColor(){
        float red,green,blue;
        NormalizedRGBA color = colorSensor.getNormalizedColors();
        red = color.red/color.alpha;
        green = color.green/color.alpha;
        blue = color.blue/color.alpha;
        double distance = colorSensor.getDistance(DistanceUnit.CM);
        if (distance<a){
            return 1;
        }
        if (red<0.25 && green>0.45 && blue<0.35)
            return 1;
        else if (red<0.36 && green<0.32 && blue>0.34)
            return 2;
        return 0;
    }
    public void k1 (){
        if (gm1.dpad_left){
            Turn120();
        }
        if (gm1.dpad_right){
            shoot();
        }
        if (gm1.dpad_down){
            nr=0;
        }
    }
    public void shoot(){
        if (time3.milliseconds()<500){
            servo.setPosition(0.3);
        }
        if (time3.milliseconds()>500){
            servo.setPosition(0.65);
            time3.reset();
        }
    }

    public boolean IsStorageSpinning(){
        return Math.abs(Target-FromTicksToDegrees()) < Math.toRadians(5) && encoder.getVelocity() < 20;
    }

}