package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretEncoder {

    double encResolution = 4000;

    double boundUpper = 1 / encResolution * 360;
    double boundLower = -1 / encResolution * 360;
    //Encoder bounds, measured in rotations of encoder

    double rotSpeed = 1;

    DcMotor rot;
    CRServo pan;

    public void init() {

        rot = hardwareMap.get(DcMotor.class, "fl");
        //Change deviceName based on wiring

        pan = hardwareMap.get(CRServo.class, "pan");

    }

    public void loop() {

    }

    public void RotToPos(Double degrees) {

        if (degrees > boundUpper || degrees < boundLower) return;

        while (degrees > toDeg(rot.getCurrentPosition())) pan.setPower(-rotSpeed);
        while (degrees < toDeg(rot.getCurrentPosition())) pan.setPower(rotSpeed);


    }

    public double toDeg(int ticks){
        return ticks / encResolution * 360;
    }
}
