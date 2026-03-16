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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.mechanisms.HoodedShooter;

@Autonomous(group = "Red Auto")
public class RedSlave1 extends OpMode {
    boolean hasShotThisState;

    GoBildaPinpointDriver pinpoint;

    HoodedShooter shooter;

    public int pathState;

    private DcMotor intake;

    private Follower follower;
    private ElapsedTime pathTimer, actionTimer, opmodeTimer, shotTimer;
    //private final Pose startPose = new Pose(80, 9, Math.toRadians(0));
    //private final Pose finalPose = new Pose(96, 9, Math.toRadians(0));
    private final Pose startPose = new Pose(80, 9, Math.toRadians(0));
    private final Pose finalPose = new Pose(96 + 20, 9, Math.toRadians(0));
    private PathChain finalPosition;
    private Path scorePreload;


    public void buildPaths(){
        scorePreload = new Path(new BezierLine(startPose, finalPose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), finalPose.getHeading());
    }
    public void stop(){
        shooter.panTracking.pan.setTargetPosition(0);
    }

    boolean tweaking = false;


    ElapsedTime waiter;
    public void setPathState(int pState) {
        pathState = pState;
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.panTracking.autoPausing = true;
                shooter.panTracking.autoPausingTimer.reset();
                opmodeTimer.reset();


                setPathState(100);
                break;
            case 100:
                if(shooter.panTracking.autoPausingTimer.seconds() > 2){
                    shooter.firePattern();
                    setPathState(1);
                }
                break;

            case 1:
                if(!shooter.firingPat){
                    follower.followPath(scorePreload);
                    setPathState(-1);
                    shooter.panTracking.resettingpan = true;
                }else{
                    tweaking = true;
                }
                break;
        }


    }

    @Override

    public void init() {
        opmodeTimer = new ElapsedTime();



        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();

        pinpoint.setPosition( new Pose2D(DistanceUnit.INCH,80, 9,AngleUnit.RADIANS, Math.toRadians(0)));

        opmodeTimer.reset();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        shooter = new HoodedShooter();




        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);

        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");

        shooter.init(hardwareMap, telemetry, pinpoint, 24);

    }

    @Override
    public void loop() {

        if(shooter.freeSort.pos1 != 'e' && shooter.freeSort.pos2 != 'e' && shooter.freeSort.pos3!='e'){
            intake.setPower(-1);
        }else{
            intake.setPower(1);
        }


        follower.update();

        shooter.loop();




        autonomousPathUpdate();


        if(tweaking)telemetry.addData("Hi", "Why are you tweaking lil bro");

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Timer :", opmodeTimer.seconds());
        telemetry.update();
    }
    @Override
    public void start() {
        opmodeTimer.reset();
        intake.setPower(1);

        follower.setStartingPose(startPose);

        shooter.start();


        Pose2D startpos = new Pose2D(DistanceUnit.INCH, 80,9,AngleUnit.RADIANS,0);
        pinpoint.setPosition(startpos);

        setPathState(0);
    }
}