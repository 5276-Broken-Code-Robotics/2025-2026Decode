package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.HoodedShooter;

@TeleOp(name = "ShooterTest")
public class ShooterTest extends OpMode {

    Servo tilt;

    @Override
    public void init() {

        tilt = hardwareMap.get(Servo.class, "tilt");

    }

    @Override
     public void loop() {
        //tilt.setPosition(0.5 * (double) gamepad1.right_trigger);
        tilt.setPosition(0);

    }
}
