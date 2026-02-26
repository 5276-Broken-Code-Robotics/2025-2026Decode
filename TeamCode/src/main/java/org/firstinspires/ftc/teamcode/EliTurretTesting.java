package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.EliTurretTracking;

@TeleOp
public class EliTurretTesting extends OpMode {

    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001, 0.000001};

    int stepIndex = 2;

    private EliTurretTracking turret = new EliTurretTracking();

    @Override
    public void init() {

        turret.init(hardwareMap, 'r');

    }

    @Override
    public void start () {

        turret.resetTimer();

    }

    @Override
    public void loop () {

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            turret.setKp(turret.getKp() - stepSizes[stepIndex]);
        }

        if (gamepad1.dpadRightWasPressed()) {
            turret.setKp(turret.getKp() + stepSizes[stepIndex]);
        }

        if (gamepad1.dpadUpWasPressed()) {
            turret.setKd(turret.getkD() + stepSizes[stepIndex]);
        }

        if (gamepad1.dpadDownWasPressed()) {
            turret.setKd(turret.getkD() - stepSizes[stepIndex]);
        }

        telemetry.addLine("------------------------------------");
        telemetry.addData("Tuning P", "%.5f (D-pad L/R)", turret.getKp());
        telemetry.addData("Tuning D", "%.5f (D-pad U/D)", turret.getkD());
        telemetry.addData("Step Size", "%.6f (B Button)", stepSizes[stepIndex]);

        turret.update(telemetry);

    }

}
