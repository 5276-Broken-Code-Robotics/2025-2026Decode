package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@Autonomous



public class AprilTagWebcamtest extends OpMode {



    private Servo pan;
    private Servo tilt;
    private DcMotor fr;


    float turretangle = 0f;
    boolean beganshot = false;
    private DcMotor fl;
    private DcMotor br;
    private DcMotor bl;
    private DcMotor flywheel;

    ElapsedTime timer;


    AprilTagWebcam aprilTagWebCam = new AprilTagWebcam();
    public void init(){
        aprilTagWebCam.init(hardwareMap, telemetry);

        fr  = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl  = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        tilt = hardwareMap.get(Servo.class, "tilt");
        pan = hardwareMap.get(Servo.class, "pan");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        timer.reset();
        beganshot = false;
    }








    public void loop(){
        aprilTagWebCam.update();
        AprilTagDetection id20 = aprilTagWebCam.getTagByID(20);


        if(!beganshot){
            if(id20 == null){
                pan.setPosition(pan.getPosition() + 10f);
            }else{

                //perform calculation to get needed angle
                if(Math.abs(id20.ftcPose.bearing) < 10){

                    beganshot = true;
                    timer.reset();
                    flywheel.setPower(0.7);


                    
                }else{
                    pan.setPosition(pan.getPosition() + id20.ftcPose.bearing);
                }
            }

        }else if(timer.milliseconds() > 1000){
            beganshot = false;


            flywheel.setPower(0);
            timer.reset();
        }






    }


}
