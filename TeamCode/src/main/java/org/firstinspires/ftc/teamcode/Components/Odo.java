package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Odo {
    GoBildaPinpointDriver pp;
    public Odo(GoBildaPinpointDriver pp){
        this.pp=pp;
        pp.resetPosAndIMU();
        pp.setOffsets(48.366/2.54,48.366/2.54, DistanceUnit.INCH);
    }
}
