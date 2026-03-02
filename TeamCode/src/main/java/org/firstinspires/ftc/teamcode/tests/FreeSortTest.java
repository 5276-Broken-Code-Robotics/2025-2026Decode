package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.FieldRelativeDrive;
import org.firstinspires.ftc.teamcode.mechanisms.FreeSortHSV;
import org.firstinspires.ftc.teamcode.mechanisms.NewFreeSortHSV;

@TeleOp(group = "Tests")
public class FreeSortTest extends OpMode {

    NewFreeSortHSV freesort = new NewFreeSortHSV();


    public void init(){

        freesort.init(hardwareMap);

    }

    public void loop(){

        freesort.updateColors(telemetry);


        //if (gamepad1.right_trigger > .25) freesort.shootAll();
        //if (gamepad1.right_bumper) freesort.shootPurple();
        //if (gamepad1.left_bumper) freesort.shootGreen();

        if (gamepad1.dpadDownWasPressed()) {freesort.shoot(1);}
        if (gamepad1.dpadRightWasPressed()) {freesort.shoot(2);}
        if (gamepad1.dpadLeftWasPressed()) {freesort.shoot(3);}

        freesort.loop();

    }


}
