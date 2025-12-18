package org.firstinspires.ftc.teamcode.mechanisms;

import com.arcrobotics.ftclib.hardware.ServoEx;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;


public class LaunchMechanism {

    double pushDegrees = 90;


    ServoEx Push;



    public void init () {

        Push = hardwareMap.get(ServoEx.class, "left_push");




        //Push.setInverted(true);
        //might need to remove, idk

    }

    public void shoot () {
        Push.rotateByAngle(pushDegrees);

    }


}
