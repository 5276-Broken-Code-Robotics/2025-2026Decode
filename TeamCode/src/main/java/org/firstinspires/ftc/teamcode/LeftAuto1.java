package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Timer;

@Autonomous
public class LeftAuto1 extends OpMode {
    private int pathState;
    private Follower follower;
    private ElapsedTime pathTimer, actionTimer, opmodeTimer;
    private final Pose startPose = new Pose(20.639209225700164, 122.41186161449752, Math.toRadians(136)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(61.680395387149915, 81.37067545304778, Math.toRadians(136)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(24, 83.98023064250413, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(24, 60.019769357495896, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(24, 34.87314662273476, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose rotation1Pose = new Pose(49.344316309719936, 83.98023064250413, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose rotation2Pose = new Pose(41.990115321252055, 60.019769357495896, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose rotation3Pose = new Pose(41.51565074135091, 34.87314662273476, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose finalPose = new Pose(55.98682042833608, 64.05271828665569, Math.toRadians(90)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private PathChain rotate1, grabPickup1, shootPickup1, rotate2, grabPickup2, shootPickup2, rotate3, grabPickup3, shootPickup3,finalPosition;
    private Path scorePreload;
    public void buildPaths(){
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        rotate1= follower.pathBuilder()
                .addPath(new BezierLine(scorePose, rotation1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), rotation1Pose.getHeading())
                .build();
        grabPickup1= follower.pathBuilder()
                .addPath(new BezierLine(rotation1Pose, pickup1Pose))
                .setLinearHeadingInterpolation(rotation1Pose.getHeading(), pickup1Pose.getHeading())
                .build();
        shootPickup1= follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();
        rotate2= follower.pathBuilder()
                .addPath(new BezierLine(scorePose, rotation2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), rotation2Pose.getHeading())
                .build();
        grabPickup2= follower.pathBuilder()
                .addPath(new BezierLine(rotation2Pose, pickup2Pose))
                .setLinearHeadingInterpolation(rotation2Pose.getHeading(), pickup2Pose.getHeading())
                .build();
        shootPickup2= follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();
        rotate3= follower.pathBuilder()
                .addPath(new BezierLine(scorePose, rotation3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), rotation3Pose.getHeading())
                .build();
        grabPickup3= follower.pathBuilder()
                .addPath(new BezierLine(rotation3Pose, pickup3Pose))
                .setLinearHeadingInterpolation(rotation3Pose.getHeading(), pickup3Pose.getHeading())
                .build();
        shootPickup3= follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
        finalPosition= follower.pathBuilder()
                .addPath(new BezierLine(scorePose, finalPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), finalPose.getHeading())
                .build();
    }
    public void setPathState(int pState) {
        pathState = pState;
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

                if (!follower.isBusy()) {

                    follower.followPath(rotate1, true);
                    setPathState(2);
                }
                break;
            case 2:

                if (!follower.isBusy()) {

                    follower.followPath(grabPickup1, true);
                    setPathState(3);
                }
                break;
            case 3:

                if (!follower.isBusy()) {

                    follower.followPath(shootPickup1, true);
                    setPathState(4);
                }
                break;
            case 4:

                if (!follower.isBusy()) {

                    follower.followPath(rotate2, true);
                    setPathState(5);
                }
                break;
            case 5:

                if (!follower.isBusy()) {

                    follower.followPath(grabPickup2, true);
                    setPathState(6);
                }
                break;
            case 6:

                if (!follower.isBusy()) {

                    follower.followPath(shootPickup2, true);
                    setPathState(7);
                }
                break;
            case 7:

                if (!follower.isBusy()) {

                    follower.followPath(rotate3, true);
                    setPathState(8);
                }
                break;
            case 8:

                if (!follower.isBusy()) {

                    follower.followPath(grabPickup3, true);
                    setPathState(9);
                }
                break;
            case 9:

                if (!follower.isBusy()) {

                    follower.followPath(shootPickup3, true);
                    setPathState(10);
                }
                break;
            case 10:

                if (!follower.isBusy()) {

                    follower.followPath(finalPosition, true);
                    setPathState(-1);
                }
                break;
        }
    }

    @Override
    public void init() {
        opmodeTimer = new ElapsedTime();
        opmodeTimer.reset();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    @Override
    public void start() {
        opmodeTimer.reset();
        setPathState(0);
    }
}