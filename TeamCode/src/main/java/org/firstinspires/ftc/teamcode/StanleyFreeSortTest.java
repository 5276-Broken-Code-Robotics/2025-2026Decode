package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.HoodedShooter;
import org.firstinspires.ftc.teamcode.mechanisms.UseFreeSortHSV;


@TeleOp
public class StanleyFreeSortTest extends OpMode {

    UseFreeSortHSV freeSort;


    GoBildaPinpointDriver pinpoint;
    public void init(){

        freeSort = new UseFreeSortHSV();
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,96, 24, AngleUnit.RADIANS,0));

        freeSort.init(hardwareMap, telemetry);



    }

    public void loop(){

        if(gamepad1.squareWasPressed()){
            freeSort.shootnum = 1;
            freeSort.armsMoving = true;
        }
        if(gamepad1.triangleWasPressed()){
            freeSort.shootnum = 2;

            freeSort.armsMoving = true;
        }
        if(gamepad1.circleWasPressed()){
            freeSort.shootnum = 3;

            freeSort.armsMoving = true;

        }

        freeSort.loop();

    }

    public void start(){

    }

}
