package org.firstinspires.ftc.teamcode.tests;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class PIDFTUNING {
    public DcMotorEx flywheel1;
    double closeRange=600;

    double longRange=1800;
    //the values used here are temporary or starting values depending on how you want to look at it
    double currentTarget;
    double F=0;
    double P=0;

    double[] stepSizes={10.0,1.0,0.1,0.01,0.001};
    int stepIndex=1;


    public void init(){
        flywheel1=hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients goonCoefficients= new PIDFCoefficients(P,0,0,F);
        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, goonCoefficients);
    }
    public void loop(){
        if (gamepad1.yWasPressed()){
            if (currentTarget==longRange){
                currentTarget=closeRange;
            }else{
                currentTarget=longRange;
            }
        }
        if (gamepad1.bWasPressed()){
            stepIndex=(stepIndex+1)%stepSizes.length;
        }
        if(gamepad1.dpadLeftWasPressed()){
            F+=stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()){
            F+=stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()){
            P+=stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()){
            P-=stepSizes[stepIndex];
        }
        flywheel1.setVelocity(currentTarget);
        double currentVelocity=flywheel1.getVelocity();
        double error=currentTarget-currentVelocity;
        telemetry.addData("Target Velocity", currentTarget);
        telemetry.addData("Current Velocity", "%.2f", currentVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad )", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
        telemetry.update();
    }
}
