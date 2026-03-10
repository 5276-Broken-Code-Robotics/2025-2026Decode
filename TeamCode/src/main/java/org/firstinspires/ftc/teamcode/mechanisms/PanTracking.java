
package org.firstinspires.ftc.teamcode.mechanisms;

import static java.lang.Math.PI;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.List;


public class PanTracking{
    DcMotor pan;
    double tAngle;
    float lastOrientedPos;

    Pose2D goalPos;



    float panZero = -410;

    float panForward = 0;

    float panMax = 410;

    boolean resetting = false;
    GoBildaPinpointDriver pinpoint;




    Limelight3A limelight3A;

    Gamepad gamepad1;




    ElapsedTime spinCD;



    int id;
    Telemetry telemetry;
    public void init(HardwareMap hardwareMap, Gamepad gamepad1, GoBildaPinpointDriver pinpoint, Telemetry telemetry, Limelight3A limelight3A, int id, Pose2D goalPos){
        pan = hardwareMap.get(DcMotorEx.class, "rot");
        pan.setPower(1);
        pan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.pinpoint = pinpoint;
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
        this.limelight3A = limelight3A;

        this.id = id;

        spinCD = new ElapsedTime();

        spinCD.reset();

        this.goalPos = goalPos;


    }









    int count = 0;

    List<LLResultTypes.FiducialResult> results;


    int pos = 0;
    public void loop(){

        limelight3A.updateRobotOrientation(pinpoint.getHeading(AngleUnit.RADIANS) + pan.getCurrentPosition() / (panMax-panZero) * PI);

        telemetry.addData("Heading :", pinpoint.getHeading(AngleUnit.DEGREES));

        if(limelight3A.getLatestResult() != null){
            results = limelight3A.getLatestResult().getFiducialResults();
        }else{
            results = new ArrayList<>();
            telemetry.addData("LL is null unfortunately", " womp womp");
        }


        double angleToAprilTag = Math.atan2((goalPos.getY(DistanceUnit.INCH) - pinpoint.getPosY(DistanceUnit.INCH)),(goalPos.getX(DistanceUnit.INCH) - pinpoint.getPosX(DistanceUnit.INCH)));


        double currAngle  = pinpoint.getHeading(AngleUnit.RADIANS);

        tAngle = turnAngle(currAngle,angleToAprilTag);



        if(Math.abs(angleToAprilTag) <= PI + 0.1){
            telemetry.addData("Moving :", "True");
        }else{
            telemetry.addData("Moving :", "False");
        }


        double targetTicks = -  tAngle * (panMax-panZero)/(Math.PI);


        if(!results.isEmpty())telemetry.addData("Yaw Value : ", results.get(0).getTargetPoseCameraSpace().getOrientation().getYaw());



        if(pinpoint.getPosY(DistanceUnit.INCH) < 48 && spinCD.seconds() > 0.2){

            spinCD.reset();
            telemetry.addData("Using Limelight Data : ",  "Now");

            boolean foundOne = false;
            LLResultTypes.FiducialResult fidRes = null;
            for(int i =0; i < results.size(); i++){
                if(results.get(i).getFiducialId() == id ){
                    foundOne = true;
                    fidRes = results.get(i);
                }
            }

            telemetry.addData("Found One : ", foundOne);


            if(foundOne){
                double valCalced = pan.getCurrentPosition() - fidRes.getTargetPoseCameraSpace().getOrientation().getYaw(AngleUnit.RADIANS) * (panMax-panZero) / PI;
                if(Math.abs(Math.abs(valCalced) - Math.abs(targetTicks)) > 10){
                    targetTicks = valCalced;
                }
            }
        }




        if(Math.abs(Math.abs(lastOrientedPos) - Math.abs(targetTicks)) > 10 && Math.abs(angleToAprilTag) <= PI ){
            pan.setTargetPosition((int) targetTicks);
            lastOrientedPos = (int)targetTicks;
        }


        /*

        LLResult reOrientRes = limelight3A.getLatestResult();
        if(pinpoint.getPosY(DistanceUnit.INCH) > 72){
            if(reOrientRes != null){
                if(reOrientRes.isValid()){
                    Pose3D pose = reOrientRes.getBotpose_MT2();
                    telemetry.addData("Calced Pos X:", pose.getPosition().x);
                    telemetry.addData("Calced Pos Y : ", pose.getPosition().y);

                }
            }
        }

         */


        List<LLResultTypes.FiducialResult> Atagresults;

        if(limelight3A.getLatestResult() != null){
            Atagresults = limelight3A.getLatestResult().getFiducialResults();
        }else{
            Atagresults = new ArrayList<>();
        }


        telemetry.addData("ATAG Results size :", Atagresults.size());
        for(int i =0; i < Atagresults.size();i++){
            if(Atagresults.get(i).getFiducialId() == 24){
                LLResultTypes.FiducialResult tag = Atagresults.get(i);
                Pose3D camPose = tag.getRobotPoseTargetSpace();

                double x = tag.getRobotPoseFieldSpace().getPosition().x;
                double y = tag.getRobotPoseFieldSpace().getPosition().y;
                double z = tag.getRobotPoseFieldSpace().getPosition().z;

                telemetry.addData("Tag ID", tag.getFiducialId());
                telemetry.addData("FidEstim Xpos", x);
                telemetry.addData("FidEstim Ypos", y );
                telemetry.addData("FidEstim Zpos", z);
            }


        }





        telemetry.addData("Current position on pan", pan.getCurrentPosition());
        telemetry.addData("Target position on pan", pan.getTargetPosition());




        telemetry.addData("Angle : ", angleToAprilTag * 180 / PI);
        telemetry.addData("Turn Angle : ", tAngle * 180/ PI );



        telemetry.addData("Pos X :", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("Pos Y :", pinpoint.getPosY(DistanceUnit.INCH));





        /*
        List<LLResultTypes.FiducialResult> Atagresults = limelight3A.getLatestResult().getFiducialResults();

        for(int i =0; i < Atagresults.size();i++){
            if(Atagresults.get(i).getFiducialId() == 24){
                LLResultTypes.FiducialResult tag = Atagresults.get(i);
                Pose3D camPose = tag.getRobotPoseFieldSpace();

                double x = tag.getRobotPoseFieldSpace().getPosition().x;
                double y = tag.getRobotPoseFieldSpace().getPosition().y;
                double z = tag.getRobotPoseFieldSpace().getPosition().z;

                telemetry.addData("Tag ID", tag.getFiducialId());
                telemetry.addData("FidEstim Xpos", x);
                telemetry.addData("FidEstim Ypos", y );
                telemetry.addData("FidEstim Zpos", z);
            }
        }

         */

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
