package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.FieldRelativeDrive;
import org.firstinspires.ftc.teamcode.mechanisms.FreeSortHSV;
import org.firstinspires.ftc.teamcode.mechanisms.FreeSortRGB;

@TeleOp
public class FreeSortTest extends OpMode {

    FreeSortHSV freesort = new FreeSortHSV();
    FieldRelativeDrive drive = new FieldRelativeDrive();

    public void init(){
        freesort.init(hardwareMap);
        drive.init(hardwareMap);
    }

    public void loop(){
        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        freesort.updateColors(telemetry);
        freesort.loop();

        if (gamepad1.right_trigger > .5) freesort.shootAll();
        if (gamepad1.right_bumper) freesort.shootPurple();
        if (gamepad1.left_bumper) freesort.shootGreen();
    }


}
