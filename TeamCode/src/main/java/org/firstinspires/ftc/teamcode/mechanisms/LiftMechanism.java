package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;






public class LiftMechanism {

    double upSpeed = 1.0;
    double downSpeed = 1.0;
    double ticks = 537.7/2; // 312 rpm goBilda motor w/2:1 ratio
    double rotationCount = (19.5/2.54)/12.2;
    // 19.5 in (extension length) / 12.2cm (circumference of pulley)


    DcMotor leftLift;
    DcMotor rightLift;

    public void init(){
        leftLift = hardwareMap.get(DcMotor.class, "left_lift");
        rightLift = hardwareMap.get(DcMotor.class, "right_lift");

        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void lift(double direction) {
        //Ascent
        if (direction > 0) {

            leftLift.setTargetPosition((int) (rotationCount * ticks));
            rightLift.setTargetPosition((int) (rotationCount * ticks));

            leftLift.setPower(upSpeed * gamepad2.right_trigger);
            rightLift.setPower(upSpeed * gamepad2.right_trigger);

        }

        //Descent
        else if (direction < 0){

            leftLift.setTargetPosition(0);
            rightLift.setTargetPosition(0);

            leftLift.setPower(downSpeed * gamepad1.left_trigger);
            rightLift.setPower(downSpeed * gamepad1.left_trigger);

        }




    }


}
