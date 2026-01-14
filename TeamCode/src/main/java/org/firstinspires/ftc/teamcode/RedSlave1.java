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

@Autonomous
public class RedSlave1 extends OpMode {
    boolean hasShotThisState;
    HoodedShooter shooter;

    private int pathState;

    private DcMotor intake;

    private Follower follower;
    private ElapsedTime pathTimer, actionTimer, opmodeTimer, shotTimer;
    private final Pose startPose = new Pose(80, 0, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose finalPose = new Pose(96, 0, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private PathChain finalPosition;
    private Path scorePreload;
    public void buildPaths(){
        scorePreload = new Path(new BezierLine(startPose, finalPose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), finalPose.getHeading());
    }

    public void setPathState(int pState) {
        pathState = pState;
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.AutoBeginShot(true,true);
                setPathState(1);
            case 1:
                if(!shooter.shotbegan)follower.followPath(scorePreload);
                setPathState(-1);
        }


    }

    @Override

    public void init() {
        opmodeTimer = new ElapsedTime();




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
    }

    @Override
    public void loop() {

        follower.update();

        shooter.loop();


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
        intake.setPower(1);

        setPathState(0);
    }
}