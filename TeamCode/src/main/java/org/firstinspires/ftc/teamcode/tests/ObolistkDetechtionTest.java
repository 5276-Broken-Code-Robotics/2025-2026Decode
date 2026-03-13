package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;
import java.util.jar.Attributes;


@Autonomous(group = "Tests")
public class ObolistkDetechtionTest extends OpMode {
    Limelight3A limelight;
    int obeliskId = 0;

    DcMotor pan;

    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(2);

        pan = hardwareMap.get(DcMotor.class, "rot");
        pan.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER5);
    }

    public void loop() {
        LLResult result = limelight.getLatestResult();
        telemetry.addData("obeliskId", obeliskId);

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                obeliskId = fiducial.getFiducialId();
            }
            fiducials = limelight.getLatestResult().getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                obeliskId = fiducial.getFiducialId();
            }
            if (obeliskId != 0) {
                limelight.pipelineSwitch(2);
            }
            telemetry.addData("obeliskId", obeliskId);
            if (obeliskId == 21) {
                char[] pattern = {'g', 'p', 'p'};
            }
            if (obeliskId == 22) {
                char[] pattern = {'p', 'g', 'p'};
            }
            if (obeliskId == 23) {
                char[] pattern = {'p', 'p', 'g'};
            }

            telemetry.addData("Pos:", pan.getCurrentPosition());


    }
}

