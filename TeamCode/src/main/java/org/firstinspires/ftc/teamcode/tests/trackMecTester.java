package org.firstinspires.ftc.teamcode.tests;

import static java.lang.Math.PI;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.PanTracking;


@TeleOp(group = "Tests")
public class trackMecTester extends OpMode {
    DcMotor pan;



    double tAngle;
    float lastOrientedPos;


    PanTracking panTracking;


    float panZero = -410;

    float panForward = 0;

    float panMax = 410;

    DcMotor fl;

    DcMotor fr;

    boolean resetting = false;
    DcMotor bl;
    DcMotor br;
    GoBildaPinpointDriver pinpoint;



    Limelight3A limelight;


    public void init(){





        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        limelight = hardwareMap.get(Limelight3A.class,"limelight");


        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.RADIANS,0));


        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.

        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);

        panTracking = new PanTracking();


        limelight.start();

        panTracking.init(hardwareMap,gamepad1,pinpoint,telemetry, limelight);


    }




    int count = 0;

    int pos = 0;
    public void loop(){

        pinpoint.update();

        telemetry.update();

        panTracking.loop();

        drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);



    }


    public void start() {
        panTracking.start();
    }


    public void drive(double forward, double strafe, double rotate) {
        double flPower = forward + strafe + rotate;
        double frPower = forward - strafe - rotate;
        double blPower = forward - strafe + rotate;
        double brPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(flPower));
        maxPower = Math.max(maxPower, Math.abs(frPower));
        maxPower = Math.max(maxPower, Math.abs(blPower));
        maxPower = Math.max(maxPower, Math.abs(brPower));

        fl.setPower((maxSpeed * (flPower / maxPower)));
        fr.setPower((maxSpeed * (frPower / maxPower)));
        bl.setPower((maxSpeed * (blPower / maxPower)));
        br.setPower((maxSpeed * (brPower / maxPower)));
    }

}
