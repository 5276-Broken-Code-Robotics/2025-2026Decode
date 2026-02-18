package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp
public class limelight3ATesting extends OpMode {




    int jklm = 0;
    private GoBildaPinpointDriver pinpoint;
    Limelight3A limelight3A;
    @Override
    public void init(){
        limelight3A = hardwareMap.get(Limelight3A.class,"limelight");



        limelight3A.pipelineSwitch(0);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();


    }
    @Override
    public void start(){

    }
    @Override
    public void loop(){

        pinpoint.update();
        double robotHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        limelight3A.updateRobotOrientation(robotHeading);

        LLResult Llresult = limelight3A.getLatestResult();


        if(Llresult!= null){

            if(Llresult.isValid()){
                Pose3D botpose = Llresult.getBotpose_MT2();

                telemetry.addData("PX : ",botpose.getPosition().x);

                telemetry.addData("PY :", botpose.getPosition().y);

                telemetry.addData("Orient : ", botpose.getOrientation());

                telemetry.update();
            }

        }

        LLResultTypes.FiducialResult tag = null;


        if(!limelight3A.getLatestResult().getFiducialResults().isEmpty()){
            tag =  limelight3A.getLatestResult().getFiducialResults().get(0);

        }



        if(tag!= null){

            Pose3D camPose = tag.getRobotPoseTargetSpace();

            double x = camPose.getPosition().x;   // meters (left/right)
            double z = camPose.getPosition().z;   // meters (forward)

            double yaw   = camPose.getOrientation().getYaw();

            telemetry.addData("Tag ID", tag.getFiducialId());
            telemetry.addData("X (m)", 144 + x);
            telemetry.addData("Z (m)", 144 + z);
            telemetry.addData("Yaw (deg)", yaw);
        }





        telemetry.addData("Why is this happening squad + ", jklm);

        jklm++;
    }
}
