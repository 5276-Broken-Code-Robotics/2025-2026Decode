package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;


public class ObolistkDetechtionTest {
    Limelight3A limelight;
    int obeliskId = 0;
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(2);
    }
    public void loop() {
        LLResult result = limelight.getLatestResult();
        if (obeliskId == 2) {
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
        }
        }
    public void start(){

    }
    }
