package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.FinalFreeSortHSV;


@TeleOp(group = "Tests")
public class StanleyFreeSortTest extends OpMode {

    FinalFreeSortHSV freeSort;


    private DcMotor flywheel1;

    private DcMotor flywheel2;



    GoBildaPinpointDriver pinpoint;
    public void init(){



        freeSort = new FinalFreeSortHSV();
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,96, 24, AngleUnit.RADIANS,0));


        flywheel1 = hardwareMap.get(DcMotor.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotor.class, "flywheel2");

        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        freeSort.init(hardwareMap, telemetry);



    }

    public void loop(){


        flywheel1.setPower(0.6);
        flywheel2.setPower(0.6);
        if(gamepad1.squareWasPressed()){
            freeSort.shoot(1);
        }
        if(gamepad1.triangleWasPressed()){
            freeSort.shoot(2);

        }
        if(gamepad1.circleWasPressed()){
            freeSort.shoot(3);

        }

        freeSort.loop();

    }

    public void start(){

    }

}
