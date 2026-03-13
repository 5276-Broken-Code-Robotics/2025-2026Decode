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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.mechanisms.HoodedShooter;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.mechanisms.FreeSortHSV;

import java.util.List;

@Autonomous(group = "Red Auto")
public class AbeTestAutoLever extends OpMode {
    boolean hasShotThisState;
    HoodedShooter shooter;
    int obeliskId = 0;
    private int pathState;
    FreeSortHSV freesort = new FreeSortHSV();

    private DcMotor intake;

    private Follower follower;
    Limelight3A limelight;
    private ElapsedTime pathTimer, actionTimer, opmodeTimer, shotTimer;
    private final Pose startPose = new Pose(125.4, 119.3, Math.toRadians(36)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(96, 96, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pose4 = new Pose(128, 84, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pose3 = new Pose(96, 84, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pose5=new Pose(119,71, Math.toRadians(90));
    private final Pose finalPose = new Pose(127.8, 71, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private PathChain rotate1, grabPickup1, shootPickup1,finalPosition;
    private Path scorePreload;

    GoBildaPinpointDriver pinpoint;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        rotate1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pose3))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pose3.getHeading())
                .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pose3, pose4))
                .setLinearHeadingInterpolation(pose3.getHeading(), pose4.getHeading())
                .build();
        shootPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pose4, pose5))
                .setLinearHeadingInterpolation(pose5.getHeading(), pose5.getHeading())
                .build();
        finalPosition= follower.pathBuilder()
                .addPath(new BezierLine(pose5,finalPose))
                .setLinearHeadingInterpolation(pose5.getHeading(),finalPose.getHeading())
                .build();

    }

    public void setPathState(int pState) {
        pathState = pState;
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:

                follower.followPath(scorePreload, true);
                setPathState(1);
                break;


            case 1:

                if ( !follower.isBusy()) {

                    follower.followPath(rotate1, true);
                    setPathState(2);
                }
                break;
            case 2:

                if (!follower.isBusy()) {

                    follower.followPath(grabPickup1,  true);
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

                    follower.followPath(finalPosition, true);
                    setPathState(-1);
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
        opmodeTimer.reset();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);


        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();


        pinpoint.setPosition( new Pose2D(DistanceUnit.INCH,125.4, 119.3, AngleUnit.RADIANS, Math.toRadians(144)));

        shooter = new HoodedShooter();


        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);

        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");

        shooter.init(hardwareMap, telemetry, pinpoint, 24);
        intake.setPower(0);

    }


    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();
        if (obeliskId == 2) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                obeliskId = fiducial.getFiducialId();
            }
            int obeliskId = 0;
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

            follower.update();

            shooter.loop();


            autonomousPathUpdate();
            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
        }
    }
    @Override
    public void start () {
        opmodeTimer.reset();
        setPathState(0);

        intake.setPower(1);

    }
}
