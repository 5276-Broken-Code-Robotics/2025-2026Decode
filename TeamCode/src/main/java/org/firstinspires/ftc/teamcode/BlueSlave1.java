package org.firstinspires.ftc.teamcode;


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

@Autonomous
public class BlueSlave1 extends OpMode {
    boolean hasShotThisState;
    HoodedShooter shooter;

    private int pathState;

    private DcMotor intake;

    private Follower follower;
    GoBildaPinpointDriver pinpoint;
    private ElapsedTime pathTimer, actionTimer, opmodeTimer, shotTimer;



    //Why is y = 24?

    private final Pose startPose = new Pose(144-80, 24, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose finalPose = new Pose(144-96, 24, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
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
                shooter.AutoBeginShot(false,true);
                setPathState(1);
            case 1:

                if(!shooter.shotbegan){
                    follower.followPath(scorePreload);
                    setPathState(-1);
                }

        }
    }

    @Override
    public void init() {
        opmodeTimer = new ElapsedTime();


        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();

        pinpoint.setPosition( new Pose2D(DistanceUnit.INCH,144- 80, 0, AngleUnit.RADIANS, Math.toRadians(0)));

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

        shooter.init(hardwareMap, telemetry, pinpoint);
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
        intake.setPower(1);

        opmodeTimer.reset();
        setPathState(0);
    }
}