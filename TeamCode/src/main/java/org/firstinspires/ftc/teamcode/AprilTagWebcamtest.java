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

    double initpos = 0f;
    private DcMotor fl;
    private DcMotor br;
    private DcMotor bl;

    ElapsedTime elapsedTime;
    private DcMotor flywheel;


    boolean waiting = false;

    boolean seen = false;
    ElapsedTime timer;


    AprilTagWebcam aprilTagWebCam = new AprilTagWebcam();
    public void init() {
        aprilTagWebCam.init(hardwareMap, telemetry);
        timer = new ElapsedTime();
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        tilt = hardwareMap.get(Servo.class, "tilt");
        pan = hardwareMap.get(Servo.class, "pan");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        timer.reset();
        beganshot = false;
        elapsedTime = new ElapsedTime();

        elapsedTime.reset();

     }








    public void loop(){
        aprilTagWebCam.update();
        AprilTagDetection id24 = aprilTagWebCam.getTagByID(24);

        telemetry.update();
        if(aprilTagWebCam.getDetectedTags() != null){
            telemetry.addData("Num : ", aprilTagWebCam.getDetectedTags().size());
        }else{
            telemetry.addData("Save ", "Me");
        }

        telemetry.addData("Seen val :" , seen);



        if(elapsedTime.seconds() <= 1f){
            waiting = false;
        }else

        if(elapsedTime.seconds() >= 1f && elapsedTime.seconds() <= 2f){
            waiting = true;
        }else
        if(elapsedTime.seconds() >= 2f){
            waiting = false;
            elapsedTime.reset();

            initpos = pan.getPosition();
        }


            if(id24 == null && !seen){

                if(!waiting)pan.setPosition(initpos + 0.05f);

            }else{

                seen = true;

                telemetry.addData("Num : ", aprilTagWebCam.getDetectedTags().size());

                telemetry.addData("We did it", "john");

                //perform calculation to get needed angle

            if(id24 != null){
                if(Math.abs(id24.ftcPose.bearing) < 10){

                    beganshot = true;
                    timer.reset();
                    flywheel.setPower(0);



                }else{
                    pan.setPosition(pan.getPosition() + id24.ftcPose.bearing);
                }
            }

            }







    }


}
