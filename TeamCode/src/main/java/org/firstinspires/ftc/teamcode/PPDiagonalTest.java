package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.paths.StanleyAutoP1;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
public class PPDiagonalTest extends OpMode {

    PathChain path1;

    Follower follower;


    boolean gone = false;
    int dex = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(0, 0), new Pose(24, 24))
                )
                .setTangentHeadingInterpolation()
                .build();

        follower.setStartingPose(new Pose(0, 0, Math.toRadians(90)));

    }

    @Override
    public void loop() {

        if(!gone && !follower.isBusy()){

            gone = true;

            follower.followPath(path1);
        }
        telemetry.addData("pos : ", follower.getPose());

        telemetry.addData("Path Index", dex);
        telemetry.addData("Busy", follower.isBusy());
        telemetry.update();

        follower.update();
    }

}
