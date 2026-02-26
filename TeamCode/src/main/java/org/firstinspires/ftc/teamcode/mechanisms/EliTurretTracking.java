package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class EliTurretTracking {

    Limelight3A limelight;

    DcMotorEx pan;

    private double kP = 0.0001;
    private double kD = 0.0000;
    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 0.2;
    private final double maxPower = 1;
    private double power = 0;
    private final ElapsedTime timer = new ElapsedTime();



    /*
    big pulley: 102t
    small pulley: 24t

    goBilda 312 rpm: 537.7 ticks per rot

    x rot(turret) * 24 / 102 * 537.7

    ALWAYS power up robot with turret facing straight ahead (Maybe sharpie mark for aligning)

     */


    private double minRot = -0.5 * 24 / 102 * 537.7; // max rot to the left
    private double maxRot = 0.5 * 24 / 102 * 537.7; // max rot to the right


    Servo tilt;

    DcMotorEx flywheel1;
    DcMotorEx flywheel2;

    public void init(HardwareMap hardwaremap, char alliance) {

        limelight = hardwaremap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        if (alliance == 'r') {limelight.pipelineSwitch(3);}
        //red pipeline: 3
        else if (alliance == 'b') {limelight.pipelineSwitch(1);}
        //blue pipeline: 1


        pan = hardwaremap.get(DcMotorEx.class, "rot");
        tilt = hardwaremap.get(Servo.class, "tilt");

        flywheel1 = hardwaremap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwaremap.get(DcMotorEx.class, "flywheel2");

        pan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pan.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pan.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void setKp (double newKP) {
        kP = newKP;
    }

    public double getKp () {
        return kP;
    }

    public void setKd (double newKD) {
        kD = newKD;
    }

    public double getkD () {
        return kD;
    }

    public void resetTimer() {
        timer.reset();
    }

    public void update(Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            telemetry.addData("result", result.getTx());
        }
        else{
            telemetry.addLine("no detections");
        }

        double deltaTime = timer.seconds();
        timer.reset();

        if (!result.isValid()) {
            pan.setPower(0);
            lastError = 0;
            return;
        }

        double error = goalX - result.getTx();
        double pTerm = error * kP;
        telemetry.addData("pTerm", pTerm);

        double dTerm = 0;
        telemetry.addData("dTerm", dTerm);

        if (deltaTime > 0) {
            dTerm = ((error - lastError) / deltaTime) * kD;
        }
        if (Math.abs(error) < angleTolerance) {
            power = 0;
        }
        else {
            power = Range.clip(pTerm + dTerm, -maxPower, maxPower);
        }

       // if (pan.getCurrentPosition() < maxRot && pan.getCurrentPosition() > minRot){
           // pan.setPower(power);
           // telemetry.addData("power", power);
        //}

        pan.setPower(power);
        telemetry.addData("power", power);

        //else {pan.setPower(0);}
        lastError = error;

    }

}
