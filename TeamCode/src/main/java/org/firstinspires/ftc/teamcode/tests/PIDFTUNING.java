package org.firstinspires.ftc.teamcode.tests;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class PIDFTUNING {
    public DcMotorEx flywheel1;
    double closeRange=600;
    double midRange=1200;
    double longRange=1800;
    //the values used here are temporary or starting values depending on how you want to look at it
    double currentTarget;
    double F=0;
    double P=0;

    double[] stepSizes={10.0,1.0,0.1,0.01,0.001};

    @Override
    public void init(){
        flywheel1=hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients goonCoefficients= new PIDFCoefficients(P,0,0,F);
        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, goonCoefficients);
    }
    @Override
    public void loop(){

    }
}
