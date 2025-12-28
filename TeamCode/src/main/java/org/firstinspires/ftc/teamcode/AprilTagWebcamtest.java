package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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

    float initposforseeingpreorient = 0f;
    boolean beganshot = false;

    double initpos = 0f;
    private DcMotor fl;
    private DcMotor br;
    private DcMotor bl;



    ElapsedTime elapsedTime;
    private DcMotor flywheel;

    private DcMotor intake;




    AprilTagDetection currOne;
    int state = 0;

    //0 : looking for
    //1 : orienting correctly

    boolean waiting = false;

    boolean seen = false;

    CRServo transfer;
    ElapsedTime timer;


    double positionnecessary = 0f;

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


        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(CRServo.class, "transfer");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        intake.setDirection(DcMotor.Direction.REVERSE);

        timer.reset();
        beganshot = false;
        elapsedTime = new ElapsedTime();

        elapsedTime.reset();



        state=  0;
     }








    public void loop(){


        intake.setPower(1);
        transfer.setPower(-1);




        if(aprilTagWebCam.getDetectedTags() != null){
            telemetry.addData("Num : ", aprilTagWebCam.getDetectedTags().size());
        }else{
            telemetry.addData("Save ", "Me");
        }

        telemetry.addData("Seen val :" , seen);


        if(state == 0) {


            if (elapsedTime.seconds() <= 1f) {
                waiting = false;
            } else if (elapsedTime.seconds() >= 1f && elapsedTime.seconds() <= 2f) {
                waiting = true;
            } else if (elapsedTime.seconds() >= 2f) {
                waiting = false;
                elapsedTime.reset();

                initpos = pan.getPosition();
            }


            if (aprilTagWebCam.getDetectedTags().isEmpty()) {

                if (!waiting) pan.setPosition(initpos + 0.05f);

            } else {

                seen = true;

                telemetry.addData("Num : ", aprilTagWebCam.getDetectedTags().size());

                telemetry.addData("We did it", "john");
                state = 1;

                currOne = aprilTagWebCam.getDetectedTags().get(0);
            }


        }

        if(state == 1){

            positionnecessary = pan.getPosition() + currOne.ftcPose.bearing * 0.4/180;

            initposforseeingpreorient = (float)pan.getPosition();
            telemetry.addData("position going to" ,  pan.getPosition() + currOne.ftcPose.bearing * 0.4/180);

            //tilt.setPosition(tilt.getPosition() + currOne.ftcPose.elevation);
            state = 2;
        }

        if(state == 2){

            telemetry.addData("curpos" , pan.getPosition());

            telemetry.addData("position going to" ,  positionnecessary);

            telemetry.addData("initpos" ,  initposforseeingpreorient);

            pan.setPosition(positionnecessary);


            telemetry.addData("bearing is this : ", currOne.ftcPose.bearing);


            if(pan.getPosition() == positionnecessary){
                state = 3;

                transfer.setPower(1);
                flywheel.setPower(0.75);
            }


            aprilTagWebCam.displayDetectionTelemetry(currOne);


        }



        telemetry.addData("state : ", state);


        if(state == 3){
            aprilTagWebCam.displayDetectionTelemetry(currOne);
        }

        aprilTagWebCam.update();

    }


}
