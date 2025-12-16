package org.firstinspires.ftc.teamcode.mechanisms;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;


public class launchMechanism {

    double rotateDegrees = 90;


    ServoEx leftPush;
    ServoEx rightPush;


    public void init () {

        leftPush = hardwareMap.get(ServoEx.class, "left_push");
        rightPush = hardwareMap.get(ServoEx.class, "right_push");



        rightPush.setInverted(true);
        //might need to be left, idk

    }

    public void shoot () {
        leftPush.rotateByAngle(rotateDegrees);
        rightPush.setPosition(rotateDegrees);

    }


}
