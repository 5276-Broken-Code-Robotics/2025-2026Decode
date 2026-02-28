package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.FieldRelativeDrive;
import org.firstinspires.ftc.teamcode.mechanisms.FreeSortHSV;

@TeleOp(group = "Tests")
public class EliTestTeleop extends OpMode {

    FieldRelativeDrive drive = new FieldRelativeDrive();
    FreeSortHSV freesort = new FreeSortHSV();

    DcMotor intake;

    GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {

        intake = hardwareMap.get(DcMotor.class, "intake");
        //intake.setDirection(DcMotor.Direction.REVERSE);

        drive.init(hardwareMap, gamepad1, pinpoint);
        freesort.init(hardwareMap);


    }

    @Override
    public void loop() {

        drive.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.right_trigger > 0.25) {freesort.shootAll();}
        if (gamepad1.rightBumperWasPressed()) {freesort.shootGreen();}
        if(gamepad1.leftBumperWasPressed()) {freesort.shootPurple();}

        if (gamepad1.dpadDownWasPressed()) {freesort.shoot1();}
        if (gamepad1.dpadRightWasPressed()) {freesort.shoot2();}
        if (gamepad1.dpadLeftWasPressed()) {freesort.shoot3();}

        intake.setPower(1);

    }
}
