package org.firstinspires.ftc.teamcode.auto.red;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.mechanisms.HoodedShooter;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.mechanisms.FreeSortHSV;

import java.util.List;

@Autonomous(group = "Red Auto")
public class AbeTestAutoLever extends OpMode {
    HoodedShooter shooter;
    int obeliskId = 0;
    private int pathState;
    FreeSortHSV freesort = new FreeSortHSV();
    private DcMotor intake;
    private Follower follower;
    Limelight3A limelight;
    private ElapsedTime opmodeTimer;

    // Only the starting pose is kept for initialization
    private final Pose startPose = new Pose(124.757, 119.827, Math.toRadians(36));

    private PathChain path1, path2, path3, path4, path5;

    GoBildaPinpointDriver pinpoint;

    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(124.757, 119.827), new Pose(96.000, 96.000)))
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(96.000, 96.000), new Pose(96.000, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(96.000, 84.000), new Pose(128.000, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(128.000, 84.000), new Pose(119.249, 71.130)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(119.249, 71.130), new Pose(127.865, 71.103)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move to Score Preload (Path 1)
                follower.followPath(path1, true);
                setPathState(1);
                break;

            case 1: // Wait for Path 1, then Shoot

                    setPathState(2);

                break;

            case 2: // Wait for Shot, then Rotate (Path 2)
                if (!follower.isBusy()) {
                    follower.followPath(path2, true);
                    setPathState(3);
                }
                break;

            case 3: // Move to Grab (Path 3)
                if (!follower.isBusy()) {
                    follower.followPath(path3, true);
                    setPathState(4);
                }
                break;

            case 4: // Move to Shoot (Path 4)
                if (!follower.isBusy()) {
                    follower.followPath(path4, true);
                    setPathState(5);
                }
                break;

            case 5: // Shoot then Final Rotation (Path 5)
                if (!follower.isBusy()) {
                        follower.followPath(path5, true);
                        setPathState(-1); // End of specified paths
                    }

                break;
        }
    }

    @Override
    public void init() {
        opmodeTimer = new ElapsedTime();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(2);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 124.757, 119.827, AngleUnit.RADIANS, Math.toRadians(36)));

        shooter = new HoodedShooter();
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);

        shooter.init(hardwareMap, telemetry, pinpoint, 24);
        intake.setPower(0);
    }

    @Override
    public void loop() {
        // Logic for Obelisk detection
        LLResult result = limelight.getLatestResult();
        if (obeliskId == 2) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                obeliskId = fiducial.getFiducialId();
            }
        }

        follower.update();
        shooter.loop();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.reset();
        setPathState(0);
        intake.setPower(1);
    }
}