package org.firstinspires.ftc.teamcode.Stuff;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
@Configurable
public class ShooterConstants {
    public static double flywheelOffset = 0;
    public static double hoodOffset = 0;
    public static double fwVel(double delta) {
        return MathFunctions.clamp(0.0204772 * Math.pow(delta, 2)+ 0.643162 * delta + 712.90909, 600,1400)
                +flywheelOffset;
    }
    public static double hoodAngle(double delta){
        return MathFunctions.clamp(-2.34831e-7 * Math.pow(delta,3) + 0.0000936893 * Math.pow(delta, 2) - 0.0165033
        *delta +1.25724, 0.11,0.904) + hoodOffset;
    }

}
