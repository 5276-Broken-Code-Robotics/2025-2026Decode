package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class OneDayAfterXMasAutoTest extends OpMode {
    private final Pose startPose = new Pose(20.639209225700164, 122.41186161449752, Math.toRadians(136)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(61.680395387149915, 81.37067545304778, Math.toRadians(136)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(13.28500823723229, 83.98023064250413, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(11.387149917627676, 60.019769357495896, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(11.387149917627676, 34.87314662273476, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose rotation1Pose = new Pose(49.344316309719936, 84.2174629324547, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose rotation2Pose = new Pose(41.990115321252055, 59.78253706754529, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose rotation3Pose = new Pose(41.51565074135091, 35.11037891268535, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose finalPose = new Pose(55.98682042833608, 64.05271828665569, Math.toRadians(90)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private PathChain rotate1, grabPickup1, shootPickup1, rotate2, grabPickup2, shootPickup2, rotate3, grabPickup3, shootPickup3;
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
    }
    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
