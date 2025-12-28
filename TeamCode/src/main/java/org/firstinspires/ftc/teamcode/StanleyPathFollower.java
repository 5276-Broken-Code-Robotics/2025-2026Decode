package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.paths.StanleyAutoP1;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;



@Autonomous
public class StanleyPathFollower extends OpMode {

    StanleyAutoP1 path1;

    Follower follower;
    int dex = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        path1 = new StanleyAutoP1(follower);

        follower.setStartingPose(new Pose(56.000, 8.000, Math.toRadians(90)));

        dex = 0;
    }

    @Override
    public void loop() {


        telemetry.addData("pos : ", follower.getPose());
        follower.update();
        if(!follower.isBusy() && dex < path1.paths.size()){
            follower.followPath(path1.paths.get(dex));
            dex++;

        }

        telemetry.addData("Path Index", dex);
        telemetry.addData("Busy", follower.isBusy());
        telemetry.update();
    }

}
