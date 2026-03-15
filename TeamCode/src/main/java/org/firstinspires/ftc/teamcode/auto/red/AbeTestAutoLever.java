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
import com.qualcomm.robotcore.hardware.HardwareMap;
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
import org.firstinspires.ftc.teamcode.mechanisms.FinalFreeSortHSV;

import java.util.List;

@Autonomous(group = "Red Auto")
public class AbeTestAutoLever extends OpMode {
    boolean hasShotThisState;
    HoodedShooter shooter;
    int obeliskId = 0;
    private int pathState;
    //FinalFreeSortHSV freesort = new FinalFreeSortHSV();

    private DcMotor intake;

    private Follower follower;
    //Limelight3A limelight;
    private ElapsedTime leverHoldTime1, opmodeTimer;
    private final Pose startPose = new Pose(125.4, 119.3, Math.toRadians(36)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(96, 96, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pose6 = new Pose(118, 84, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pose5 = new Pose(96, 84, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose leverPrep=new Pose(118,71, Math.toRadians(90));
    private final Pose pose3=new Pose(96,57, Math.toRadians(0));

    private final Pose pose4=new Pose(118,57, Math.toRadians(0));


    private final Pose pose7=new Pose(96,36, Math.toRadians(0));

    private final Pose pose8=new Pose(118,36, Math.toRadians(0));

    private final Pose leverHit = new Pose(120, 71, Math.toRadians(90));
    private final Pose finalPose = new Pose(110, 70, Math.toRadians(90));// Lowest (Third Set) of Artifacts from the Spike Mark.
    private PathChain scorePreload,pos1,pos2,pos3,pos4,pos5,pos6,pos7,pos8,pos9,pos10,pos11,pos12,pos13,pos14;

    GoBildaPinpointDriver pinpoint;

//poses  after 1, 6,9,12


    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        pos1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pose3))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pose3.getHeading())
                .build();
        pos2 = follower.pathBuilder()
                .addPath(new BezierLine(pose3, pose4))
                .setLinearHeadingInterpolation(pose3.getHeading(), pose4.getHeading())
                .build();
        pos3 = follower.pathBuilder()
                .addPath(new BezierLine(pose4, pose3))
                .setLinearHeadingInterpolation(pose4.getHeading(), pose3.getHeading())
                .build();
        pos4 = follower.pathBuilder()
                .addPath(new BezierLine(pose3, scorePose))
                .setLinearHeadingInterpolation(pose3.getHeading(), scorePose.getHeading())
                .build();
        pos7= follower.pathBuilder()
                .addPath(new BezierLine(pose6,leverPrep))
                .setLinearHeadingInterpolation(pose6.getHeading(),leverPrep.getHeading())
                .build();
        pos8= follower.pathBuilder()
                .addPath(new BezierLine(leverPrep,leverHit))
                .setLinearHeadingInterpolation(leverPrep.getHeading(),leverHit.getHeading())
                .build();
        pos5= follower.pathBuilder()
                .addPath(new BezierLine(scorePose,pose5))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pose5.getHeading())
                .build();
        pos6= follower.pathBuilder()
                .addPath(new BezierLine(pose5,pose6))
                .setLinearHeadingInterpolation(pose5.getHeading(),pose6.getHeading())
                .build();
        pos9= follower.pathBuilder()
                .addPath(new BezierLine(leverHit,scorePose))
                .setLinearHeadingInterpolation(leverHit.getHeading(),scorePose.getHeading())
                .build();
        pos10= follower.pathBuilder()
                .addPath(new BezierLine(scorePose,pose7))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pose7.getHeading())
                .build();
        pos11= follower.pathBuilder()
                .addPath(new BezierLine(pose7,pose8))
                .setLinearHeadingInterpolation(pose7.getHeading(),pose8.getHeading())
                .build();
        pos12= follower.pathBuilder()
                .addPath(new BezierLine(pose8,scorePose))
                .setLinearHeadingInterpolation(pose8.getHeading(),scorePose.getHeading())
                .build();
        pos13= follower.pathBuilder()
                .addPath(new BezierLine(scorePose,finalPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(),finalPose.getHeading())
                .build();

    }

    public void setPathState(int pState) {
        pathState = pState;
    }

    ElapsedTime fireCD;

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                shooter.panTracking.autoPausing = true;
                shooter.panTracking.autoPausingTimer.reset();
                follower.followPath(scorePreload, 1, false);
                setPathState(100);

                break;
            case 100:
                if (!follower.isBusy()) {
                    setPathState(1);
                    shooter.firePattern();
                    fireCD.reset();
                }

                break;
            case 1:
                if(fireCD.seconds() > 3) {
                    follower.followPath(pos1, false);
                    setPathState(101);
                }
                break;

            case 101:
                if (!follower.isBusy()){
                    setPathState(2);

                }
                break;
            case 2:

                    follower.followPath(pos2, false);
                    setPathState(102);



                break;
            case 102:
                if (!follower.isBusy()) setPathState(3);
                break;
            case 3:


                follower.followPath(pos3, false);
                setPathState(103);

                break;


            case 103:
                if (!follower.isBusy()) {

                    setPathState(67);

                }
                break;
            case 67:

//shoot
                if (!follower.isBusy()) {
                    follower.followPath(pos4, false);
                    setPathState(6767);
                }

                break;
            case 6767:
                if (!follower.isBusy()) {

                    setPathState(4);

                }
                break;
            case 4:


                if (!follower.isBusy()) {
                    follower.followPath(pos5, false);
                    setPathState(104);
                }

                break;
            case 104:
                if (!follower.isBusy()) {

                    setPathState(5);

                }
                break;
            case 5:


                if (!follower.isBusy()) {
                    leverHoldTime1.startTime();
                    follower.followPath(pos6, true);
                    setPathState(105);
                }

                break;
            case 105:
                if (!follower.isBusy()) {

                    setPathState(6);
                    fireCD.reset();
                    shooter.firePattern();
                }
                break;
            case 6:


                if (!follower.isBusy() && fireCD.seconds() > 3) {
                    follower.followPath(pos7, false);
                    setPathState(106);
                }

                break;
            case 106:
                if (!follower.isBusy()) {

                    setPathState(7);

                }
                break;
            case 7:


                if (!follower.isBusy()) {
                    follower.followPath(pos8, false);
                    setPathState(107);
                }

                break;
            case 107:
                if (!follower.isBusy()) {

                    setPathState(8);

                }
                break;
            case 8:


                if (!follower.isBusy()) {
                    follower.followPath(pos9, false);
                    setPathState(108);
                }

                break;
            case 108:
                if (!follower.isBusy()) {

                    setPathState(9);
                    fireCD.reset();
                    shooter.firePattern();

                }
                break;
            case 9:


                if (!follower.isBusy() && fireCD.seconds() > 3) {
                    follower.followPath(pos10, false);
                    setPathState(109);
                }

                break;
            case 109:
                if (!follower.isBusy()) {

                    setPathState(10);

                }
                break;
            case 10:


                if (!follower.isBusy()) {
                    follower.followPath(pos11, false);
                    setPathState(110);
                }

                break;
            case 110:
                if (!follower.isBusy()) {

                    setPathState(11);

                }
                break;
            case 11:


                if (!follower.isBusy()) {
                    follower.followPath(pos12, false);
                    setPathState(111);
                }

                break;
            case 111:
                if (!follower.isBusy()) {
                    shooter.firePattern();
                    fireCD.reset();
                    setPathState(12);

                }
                break;
            case 12:


                if (!follower.isBusy() && fireCD.seconds() > 3) {
                    follower.followPath(pos13, false);
                    setPathState(-1);
                }

                break;

        }
    }


    @Override
    public void init() {
        opmodeTimer = new ElapsedTime();
        leverHoldTime1 = new ElapsedTime();
        opmodeTimer = new ElapsedTime();
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //limelight.start();
        opmodeTimer.reset();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);



        fireCD = new ElapsedTime();
        //freesort = new FinalFreeSortHSV();

        //freesort.init(hardwareMap, telemetry);




        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);



        shooter = new HoodedShooter();


        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);

        shooter.init(hardwareMap, telemetry, pinpoint, 24);
        intake.setPower(0);

    }


    public char[] pattern = {'p','p','p'};

    @Override


    public void loop() {


        /*
            freesort.loop();

            LLResult result = limelight.getLatestResult();


            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();


            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                if(fiducial.getFiducialId() <= 23 && fiducial.getFiducialId() >= 21) obeliskId  = fiducial.getFiducialId();
            }

         */


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
            //telemetry.addData("Detections empty : ", limelight.getLatestResult().getFiducialResults().isEmpty());

            telemetry.update();
        }

    @Override
    public void start () {
        opmodeTimer.reset();
        setPathState(0);
        pinpoint.setPosition( new Pose2D(DistanceUnit.INCH,125.4, 119.3, AngleUnit.RADIANS, Math.toRadians(36)));
        intake.setPower(-1);

        shooter.start();

    }
}
