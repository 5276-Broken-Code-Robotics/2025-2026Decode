package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.LaunchMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.LiftMechanism;

@TeleOp(name = "POVDrive", group = "Drive")
public class EliTeleopTest extends OpMode {

    LiftMechanism lift = new LiftMechanism();
    POVDriveOp drive = new POVDriveOp();

    LaunchMechanism launch = new LaunchMechanism();

    // Uncomment below line + comment out above line for Field Relative

//    FieldRelativeDrive drive = new FieldRelativeDrive();



    DcMotor intake;
    DcMotor flywheel;




    @Override
    public void init() {

        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

    }

    @Override
    public void loop() {

        //Lift Mechanism
        lift.lift(gamepad2.right_trigger-gamepad2.left_trigger);

        //Drivetrain
        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        //Intake and flywheel - running continuously
        intake.setPower(1);
        flywheel.setPower(1);

        if (gamepad1.right_trigger > 0) {
            launch.shoot();
        }

    }



}