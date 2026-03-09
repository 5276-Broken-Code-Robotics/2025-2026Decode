
package org.firstinspires.ftc.teamcode.mechanisms;

import static java.lang.Math.PI;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


public class PanTracking{
    DcMotor pan;
    double tAngle;
    float lastOrientedPos;



    float panZero = -410;

    float panForward = 0;

    float panMax = 410;

    boolean resetting = false;
    GoBildaPinpointDriver pinpoint;



    Gamepad gamepad1;

    Telemetry telemetry;
    public void init(HardwareMap hardwareMap, Gamepad gamepad1, GoBildaPinpointDriver pinpoint, Telemetry telemetry){
        pan = hardwareMap.get(DcMotorEx.class, "rot");
        pan.setPower(1);
        pan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.pinpoint = pinpoint;
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }




    int count = 0;

    int pos = 0;
    public void loop(){

        pinpoint.update();

        if(gamepad1.rightBumperWasPressed()){
            pos+=15;
            count++;
        }

        if(gamepad1.leftBumperWasPressed()){
            pos-=15;
            count++;
        }

        if(gamepad1.aWasPressed()){
            pan.setTargetPosition(0);

            resetting = true;
        }

        telemetry.addData("Heading :", pinpoint.getHeading(AngleUnit.DEGREES));




        double angleToAprilTag = Math.atan2((144 - pinpoint.getPosY(DistanceUnit.INCH)),(144 - pinpoint.getPosX(DistanceUnit.INCH)));


        double currAngle  = pinpoint.getHeading(AngleUnit.RADIANS);

        tAngle = turnAngle(currAngle,angleToAprilTag);

        double targetTicks = -  tAngle * (panMax-panZero)/(Math.PI);


        if(Math.abs(angleToAprilTag) <= PI + 0.1){
            telemetry.addData("Moving :", "True");
        }else{
            telemetry.addData("Moving :", "False");
        }


        if(Math.abs(angleToAprilTag) <= PI + 0.1 && Math.abs(lastOrientedPos - targetTicks) > 10){
            if(!resetting)pan.setTargetPosition((int)targetTicks);
            lastOrientedPos = (int)targetTicks;
        }





        telemetry.addData("Current position on pan", pan.getCurrentPosition());
        telemetry.addData("Target position on pan", pan.getTargetPosition());




        telemetry.addData("Angle : ", angleToAprilTag * 180 / PI);
        telemetry.addData("Turn Angle : ", tAngle * 180/ PI );



        telemetry.addData("Pos X :", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("Pos Y :", pinpoint.getPosY(DistanceUnit.INCH));

        telemetry.update();




    }


    public void start(){
        pan.setDirection(DcMotorSimple.Direction.FORWARD);
        pan.setTargetPosition((int)panForward);
        pan.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        resetting = false;
        pan.setPower(1);
    }

    public static double turnAngle(double currentHeading, double targetAngle) {
        double error = targetAngle - currentHeading;

        // Normalize to (-PI, PI]
        while (error > Math.PI)  error -= 2 * Math.PI;
        while (error <= -Math.PI) error += 2 * Math.PI;

        return error;
    }


}
