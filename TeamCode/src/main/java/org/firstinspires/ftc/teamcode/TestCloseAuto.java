package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.mechanisms.LaunchMechanism;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.nio.file.Path;

@Autonomous(name = "TestClose", group = "testing")
public class TestCloseAuto extends OpMode {


    private Follower follower;
    private Timer pathTimer, opModeTimer;

    LaunchMechanism shooter = new LaunchMechanism();

    public enum PathState {
        // Start POS --> End POS
        //

        driveStartShoot,
        shootPreload
    }

    PathState pathState;
    private final Pose startPos = new Pose(21.59, 123.07, Math.toRadians(144));
    private final Pose shootPos = new Pose(48, 95.34, Math.toRadians(144));

    private PathChain driveStartShoot;

    public void buildPaths() {

        driveStartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPos, shootPos))
                .setLinearHeadingInterpolation(startPos.getHeading(), shootPos.getHeading())
                .build();

    }

    public void statePathUpdate() {
        switch(pathState) {
            case driveStartShoot:
                follower.followPath(driveStartShoot, true);
                setPathState(PathState.shootPreload);
                break;
            case shootPreload:
                if (!follower.isBusy()) {
                    shooter.shoot();
                }
                break;
            default:
                telemetry.addLine("No State");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.driveStartShoot;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPos);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
    }
}
