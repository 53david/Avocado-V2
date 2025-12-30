package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.prevgm1;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Stuff.PIDController;

public class Storage {
    private CRServo revolver;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime time = new ElapsedTime(); int nr = 0 , prevnr;
    double p = 1.0, t=2000; boolean ok = false; Gamepad gamepad2;
    boolean next = false, prevnext = false;
    DcMotorEx encoder; Telemetry telemetry; NormalizedColorSensor colorSensor;
    float x = 3 ;
    public PIDController turner;
    public double Target = 0;

    int n=0;
    enum ColorState {
        IDLE,
        GREEN,
        PURPLE,
        MANUAL,
    };
    ColorState state;
    public enum IntakeState{
        IDLE,
        ACTIVE,

    };
    public enum StorageState{
        BALL1,
        BALL2,
        BALL3,
    }
    StorageState state1 = StorageState.BALL1; private DcMotorEx motor;
    IntakeState state2 = IntakeState.IDLE;

    public Storage (CRServo revolver, DcMotorEx encoder, DcMotorEx motor, NormalizedColorSensor colorSensor, Telemetry telemetry){
        this.revolver = revolver;
        this.colorSensor=colorSensor;
        this.telemetry=telemetry;
        colorSensor.setGain(x);
        state = ColorState.IDLE;
        this.encoder = encoder;
        this.motor = motor;
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turner = new PIDController(0,0,0);
        revolver.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double FromTicksToDegrees(){
        return encoder.getCurrentPosition() / 8192.0 * Math.PI * 2.0;
    }

    public void Turn120(){
        Target -= Math.PI * 2 / 3;
        turner.setTargetPosition(Target);
    }

    public void update()
    {
        revolver.setPower(turner.calculatePower(FromTicksToDegrees()));
    }
    public void Index(){
        getColor();
        switch (state){
            case IDLE:
                    telemetry.addLine("IDLE");
                    nr=0;
                    timer.reset();
                break;
            case GREEN:
                    telemetry.addLine("GREEN");
                    Turn120();
                    state = state.IDLE;
                break;
            case PURPLE:
                    telemetry.addLine("PURPLE");
                    Turn120();
                    state = state.IDLE;
                break;
            case MANUAL:
                    if (timer.milliseconds()>500){
                        Turn120();
                        state = state.IDLE;
                    }
                break;
        }
        telemetry.update();

    }
    public void getColor(){
        float red,green,blue;
        NormalizedRGBA color = colorSensor.getNormalizedColors();
        red = color.red/color.alpha;
        green = color.green/color.alpha;
        blue = color.blue/color.alpha;

        telemetry.addData("Red",red);
        telemetry.addData("Green",green);
        telemetry.addData("Blue",blue);

        if (DetectColor()==1 && ok == false){
            state = state.GREEN; ok = true;
            telemetry.addLine("GREEN");

        }
        else if (DetectColor()==2 && ok == false){
            state = state.PURPLE; ok = true;
            telemetry.addLine("PURPLE");

        }
        else {
            ok=false;
            state = state.IDLE;
        }

    }
    public int DetectColor(){
        float red,green,blue;
        NormalizedRGBA color = colorSensor.getNormalizedColors();
        red = color.red/color.alpha;
        green = color.green/color.alpha;
        blue = color.blue/color.alpha;

        if (red<0.25 && green>0.45 && blue<0.35)
            return 1;
        else if (red<0.36 && green<0.32 && blue>0.34)
            return 2;
       return 0;
    }
    public boolean IsStorageSpinning(){
        return Math.abs(Target-FromTicksToDegrees()) < Math.toRadians(5) && encoder.getVelocity() < 20;
    }

}