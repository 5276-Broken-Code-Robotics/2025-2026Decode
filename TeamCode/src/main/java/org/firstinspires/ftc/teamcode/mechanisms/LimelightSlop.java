package org.firstinspires.ftc.teamcode.mechanisms;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class LimelightSlop {
    //the limelight needs to be set to the correct pipeline based on what
    // side of the field we are on otherwise we will track the wrong thing
    Limelight3A limelight=new Limelight3A();
    limelight.start();
}
