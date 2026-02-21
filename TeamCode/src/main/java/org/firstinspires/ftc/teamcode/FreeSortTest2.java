package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.FreeSortHSV;

@TeleOp
public class FreeSortTest2 extends OpMode {

    Servo arm1;
    Servo arm2;
    Servo arm3;




    double val = 0f;

    ElapsedTime incrCD = new ElapsedTime();


    boolean begin = false;
    @Override

    public void init() {


        incrCD = new ElapsedTime();
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        arm3 = hardwareMap.get(Servo.class, "arm3");

        arm1.setPosition(0);
        arm2.setPosition(0);
        arm3.setPosition(0);

    }

    @Override
    public void loop() {

        /*
        arm1.setPosition((0.4 * (double) gamepad1.right_trigger));

        arm2.setPosition((.4 * (double) gamepad1.right_trigger));



        arm3.setPosition((.4 * (double) gamepad1.left_trigger));


         */

        if(gamepad1.right_stick_y > 0){
            begin = true;
        }


    if(begin){
        if(incrCD.seconds() > 0.3){
            incrCD.reset();
            val+=0.1;

            if(val >= 0.4){
                val = 0.4;
                begin = false;
            }
            arm2.setPosition(val);
        }
    }

        /*
        arm1.setPosition(.15 * gamepad2.right_trigger);
        arm2.setPosition((.15 * gamepad2.right_trigger));
        arm3.setPosition((.3 * gamepad2.right_trigger));
        */

    }

}
