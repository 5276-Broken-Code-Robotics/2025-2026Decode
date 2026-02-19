package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;




@TeleOp
public class limelight3ATesting extends OpMode {






    float range = 0;
    DcMotor pan;
    int jklm = 0;
    private GoBildaPinpointDriver pinpoint;
    Limelight3A limelight3A;
    @Override
    public void init(){
        limelight3A = hardwareMap.get(Limelight3A.class,"limelight");


        pan.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        limelight3A.pipelineSwitch(0);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();


        pan.setTargetPosition(0);

    }
    @Override
    public void start(){

    }
    @Override
    public void loop(){

        pinpoint.update();
        double robotHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        limelight3A.updateRobotOrientation(robotHeading + pan.getCurrentPosition() / range * PI);

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

        LLResultTypes.FiducialResult tag;


        if(!limelight3A.getLatestResult().getFiducialResults().isEmpty()) {
            tag = limelight3A.getLatestResult().getFiducialResults().get(0);





            Pose3D camPose = tag.getRobotPoseTargetSpace();

            double x = camPose.getPosition().x;   // meters (left/right)
            double z = camPose.getPosition().z;   // meters (forward)

            double yaw = camPose.getOrientation().getYaw();

            telemetry.addData("Tag ID", tag.getFiducialId());
            telemetry.addData("Xpos", 144 + x * 39.37);
            telemetry.addData("Zpos", 144 - z * 39.37);
            telemetry.addData("Yaw (deg)", yaw);

        }









        telemetry.addData("Why is this happening squad + ", jklm);

        jklm++;
    }
}
