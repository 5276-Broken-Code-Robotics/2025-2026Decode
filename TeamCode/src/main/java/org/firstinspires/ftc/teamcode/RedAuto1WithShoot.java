package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.mechanisms.HoodedShooter;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.mechanisms.FreeSort;
@Autonomous
public class RedAuto1WithShoot extends OpMode {
    boolean hasShotThisState;
    HoodedShooter shooter;

    private int pathState;

    private DcMotor intake;

    private Follower follower;
    Limelight3A limelight;
    private ElapsedTime pathTimer, actionTimer, opmodeTimer, shotTimer;
    private final Pose startPose = new Pose(125, 118.37891268533772, Math.toRadians(36)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(96, 99, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(125, 87, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(125, 60, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(130, 39, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose rotation1Pose = new Pose(96, 87, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose rotation2Pose = new Pose(96, 60, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose rotation3Pose = new Pose(96, 39, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose finalPose = new Pose(103, 68, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
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

                follower.followPath(scorePreload, true);
                setPathState(-10);
                break;

            case -10:
                if(!follower.isBusy()){
                    shooter.AutoBeginShot(true,false);
                    setPathState(1);
                }

                break;
            case 1:

                if (!shooter.shotbegan && !follower.isBusy()) {

                    follower.followPath(rotate1, true);
                    setPathState(2);
                }
                break;
            case 2:

                if (!follower.isBusy()) {

                    follower.followPath(grabPickup1, .8,false);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {

                    follower.followPath(shootPickup1, true);
                    setPathState(21);
                }
                break;


            case 21:
                if (!follower.isBusy()) {

                    if(!shooter.shotbegan){
                        shooter.AutoBeginShot(true,false);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if(shooter.shotbegan) return; // Wait until the shot is done



                if(!follower.isBusy()) {
                    follower.followPath(rotate2, true);
                    setPathState(5);
                }

                break;
            case 5:

                if (!follower.isBusy()) {

                    follower.followPath(grabPickup2,.8, false);
                    setPathState(6);
                }
                break;
            case 6:

                if (!follower.isBusy()) {

                    follower.followPath(shootPickup2, true);
                    setPathState(7);

                    shooter.AutoBeginShot(true,false);
                }

                break;
            case 7:
                if(shooter.shotbegan) return; // Wait until the shot is done

                if (!follower.isBusy()) {

                    follower.followPath(rotate3, true);
                    setPathState(8);
                }
                break;
            case 8:

                if (!follower.isBusy()) {

                    follower.followPath(grabPickup3,.8, false);
                    setPathState(9);
                }
                break;
            case 9:

                if (!follower.isBusy()) {

                    follower.followPath(shootPickup3, true);
                    setPathState(10);

                    shooter.AutoBeginShot(true,false);
                }

                break;
            case 10:
                if(shooter.shotbegan) return; // Wait until the shot is done

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



        opmodeTimer.reset();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        shooter = new HoodedShooter();




        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);

        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");

        shooter.init(hardwareMap, telemetry, follower, fl, fr, bl, br);
        intake.setPower(0);

    }


    @Override
    public void loop() {


        follower.update();

        shooter.loop();
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);

        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
    }
    @Override
    public void start() {
        opmodeTimer.reset();
        setPathState(0);

        intake.setPower(1);

    }
}