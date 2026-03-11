package org.firstinspires.ftc.teamcode.mechanisms;


import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class HoodedShooter {

    FinalFreeSortHSV freeSort;


    private DcMotor pan;
    private Servo tilt;

    ElapsedTime shootTimer;

    double initpos = 0f;

    double positionnecessary;


    int id = 0;
    String state;

    ElapsedTime elapsedTime;


    private DcMotor flywheel1;

    private DcMotor flywheel2;



    //0.75 for close sec 2

    double flywheelPower;


    Telemetry telemetry;

    public boolean shotbegan = false;

    boolean resetting = false;

    ElapsedTime waitrotate;

    double angleToAprilTag = 0;
    Pose2D aprilTagRed = new Pose2D(DistanceUnit.INCH,144,144, AngleUnit.RADIANS,0);
    Pose2D aprilTagBlue = new Pose2D(DistanceUnit.INCH,0,144, AngleUnit.RADIANS,0);
    boolean rotated = false;
    Pose2D currentAprilTagPos;


    int lastOrientedPos;
    boolean isAutoShot;

    Pose initPose;
    int currentID;
    Limelight3A limelight;

    float faceForwardPos = 105;

    float maxTurn = 525;
    GoBildaPinpointDriver pinpoint;

    public ElapsedTime pinpointUpdatePause;
    // (160 (vbig gear) / 18 (small gear)  * 20 (degrees of rotation))/300
    double maxTilt = 0.59;
    public void init(HardwareMap hardwareMap, Telemetry telemetry, GoBildaPinpointDriver pinpoint, int id) {

        shootTimer = new ElapsedTime();

        freeSort = new FinalFreeSortHSV();

        freeSort.init(hardwareMap, telemetry);

        shootTimer.reset();


        pinpointUpdatePause = new ElapsedTime();
        tilt = hardwareMap.get(Servo.class, "tilt");
        flywheel1 = hardwareMap.get(DcMotor.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotor.class, "flywheel2");
        limelight = hardwareMap.get(Limelight3A.class,"limelight");


        pan = hardwareMap.get(DcMotor.class,"rot");
        pan.setTargetPosition((int)faceForwardPos);
        pan.setPower(1);
        pan.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.id = id;


        this.pinpoint = pinpoint;


        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        waitrotate = new ElapsedTime();
        resetting = false;



        panTracking = new PanTracking();


        if(id == 24){
            currentAprilTagPos = aprilTagRed;
        }else{
            currentAprilTagPos = aprilTagBlue;
        }


        currentID = id;


        panTracking.init(hardwareMap,pinpoint,telemetry,limelight,id,currentAprilTagPos);
        limelight.start();

        this.telemetry = telemetry;




        elapsedTime = new ElapsedTime();
    }

    public void start(){
        pinpointUpdatePause.reset();
        panTracking.start();
    }

    public void loop()
    {
        freeSort.loop();

        panTracking.loop();

        Orienting();
        /*


        telemetry.addData("Flywheel power", flywheel1.getPower() + " " + flywheel2.getPower());
        telemetry.addData("Pan position : ", pan.getCurrentPosition());



        telemetry.addData("numchanges", numchanges);
        telemetry.addData("IsAutoShot : ", isAutoShot);
        telemetry.addData("Shot status : ", shotbegan);

        telemetry.addData("Num shots : ", numshots);

         */

        if(pinpointUpdatePause.seconds() > 1)pinpoint.update();

        LLResult Llresult = limelight.getLatestResult();

        if(Llresult!= null && Llresult.isValid()){

            Pose3D botpose = Llresult.getBotpose_MT2();


            /*
            telemetry.addData("PX : ",botpose.getPosition().x);

            telemetry.addData("PY :", botpose.getPosition().y);

            telemetry.addData("Orient : ", botpose.getOrientation());

             */

        }


        //Orienting();


        flywheel1.setPower(0.6);
        flywheel2.setPower(0.6);
        tilt.setPosition(0.075);

        //telemetry.update();
    }

    public void AutoBeginShot(boolean isRed, boolean isFar){

        isAutoShot = true;
        flywheelPower = 0.63; // Needs testing for accurate value
        tilt.setPosition(0); // Needs testing for accurate value
        elapsedTime.reset();



        waitrotate.reset();

        if(isRed &&!isFar){
            positionnecessary = 0.29/0.45 * 286/(0.45);
            flywheelPower = 0.63; // Needs testing for accurate value
            tilt.setPosition(0); // Needs testing for accurate value
        }

        if(!isRed && !isFar){
            positionnecessary = 0.11/0.45* 286/(0.45);
            flywheelPower = 0.63; // Needs testing for accurate value
            tilt.setPosition(0); // Needs testing for accurate value
        }

        if(isRed && isFar){
            positionnecessary = 0.38* 286/(0.45);
            flywheelPower = 0.8; // Needs testing for accurate value
            tilt.setPosition(0.275); // Needs testing for accurate value
        }

        if(!isRed && isFar){
            positionnecessary = (0.45-0.38)* 286/(0.45);
            flywheelPower = 0.8; // Needs testing for accurate value
            tilt.setPosition(0.275); // Needs testing for accurate value
        }

        pan.setTargetPosition((int)positionnecessary);
        shotbegan = true;



        shotbegan = false;
    }



    public void BeginShot(){


        isAutoShot = false;
        elapsedTime.reset();
        state = "chassis_orient_to_tag";
        currentID = id;
        if(id == 24){
            currentAprilTagPos = aprilTagRed;
        }

        if(id == 20){
            currentAprilTagPos = aprilTagBlue;
        }

        rotated = false;

        shotbegan = true;


        shotbegan = false;

    }



    boolean firingAllThree = false;
    public void fireThree(){
        firingAllThree = true;
    }


    public void FiringAllThree(){
        if(firingAllThree){
            //implement
        }
    }


    PanTracking panTracking;
    public void Orienting(){

    }



    public void Fire(int num){
        if(!freeSort.shooting)freeSort.shoot(num);
    }

    public static double turnAngle(double currentHeading, double targetAngle) {
        double error = targetAngle - currentHeading;

        // Normalize to (-PI, PI]
        while (error > Math.PI)  error -= 2 * Math.PI;
        while (error <= -Math.PI) error += 2 * Math.PI;

        return error;
    }

    public static double distTo(Pose tagPos, Pose ourPos){
        return Math.sqrt((ourPos.getY() - tagPos.getY())*(ourPos.getY() - tagPos.getY()) + (ourPos.getX() - tagPos.getX())*(ourPos.getX() - tagPos.getX()));
    }

    public static double AngleToTag(Pose tagPos, Pose ourPos){
        return Math.atan2(tagPos.getY() - ourPos.getY(), tagPos.getX() - ourPos.getX());
    }


}


