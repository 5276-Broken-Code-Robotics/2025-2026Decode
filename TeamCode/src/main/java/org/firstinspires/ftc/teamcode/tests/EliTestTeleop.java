package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.FieldRelativeDrive;
import org.firstinspires.ftc.teamcode.mechanisms.FreeSortHSV;
import org.firstinspires.ftc.teamcode.mechanisms.NewFreeSortHSV;
import org.firstinspires.ftc.teamcode.mechanisms.POVDrive;

@TeleOp(group = "Tests")
public class EliTestTeleop extends OpMode {

    POVDrive drive = new POVDrive();
    NewFreeSortHSV freesort = new NewFreeSortHSV();

    DcMotor intake;
    DcMotor flywheel1;
    DcMotor flywheel2;


    ElapsedTime timer = new ElapsedTime();

    boolean flywheelToggle = false;
    boolean intakeToggle = false;

    @Override
    public void init() {

        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel1 = hardwareMap.get(DcMotor.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotor.class, "flywheel2");

        flywheel1.setDirection(DcMotor.Direction.REVERSE);

        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        //intake.setDirection(DcMotor.Direction.REVERSE);

        drive.init(hardwareMap);
        freesort.init(hardwareMap);


    }

    @Override
    public void loop() {

        freesort.updateColors(telemetry);
        freesort.loop();

        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.dpadDownWasPressed()) {freesort.shoot(1);}
        if (gamepad1.dpadRightWasPressed()) {freesort.shoot(2);}
        if (gamepad1.dpadLeftWasPressed()) {freesort.shoot(3);}

        if (gamepad1.aWasPressed()) {
            if (flywheelToggle) {
                flywheelToggle = false;
            } else {
                flywheelToggle = true;
            }
        }

        if (gamepad1.bWasPressed()) {
            if (intakeToggle) {
                intakeToggle = false;
            } else {
                intakeToggle = true;
            }
        }


        if (flywheelToggle) {
            flywheel1.setPower(1);
            flywheel2.setPower(1);
        }
        else {
            flywheel1.setPower(0);
            flywheel2.setPower(0);
        }

        if (intakeToggle) {
            intake.setPower(1);
        }
        else {
            intake.setPower(0);
        }




    }
}
