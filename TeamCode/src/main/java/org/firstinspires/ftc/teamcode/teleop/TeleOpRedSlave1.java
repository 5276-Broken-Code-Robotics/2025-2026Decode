package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.FieldRelativeDrive;
import org.firstinspires.ftc.teamcode.mechanisms.HoodedShooter;

@TeleOp(group = "TeleOp")



public class TeleOpRedSlave1 extends OpMode {
    boolean beganshot = false;

    double initpos = 0f;

    private DcMotor fr;

    private DcMotor fl;

    private DcMotor br;
    private DcMotor bl;




    FieldRelativeDrive fieldDrive;
    ElapsedTime elapsedTime;

    private DcMotor intake;





    int state = 0;

    //0 : looking for
    //1 : orienting correctly


    ElapsedTime timer;



    double positionnecessary = 0f;




    Pose aprilTagRed = new Pose(144,144);
    Pose aprilTagBlue = new Pose(0,144);

    HoodedShooter hShooter;

    ElapsedTime rewindTimer;

    GoBildaPinpointDriver pinpoint;


    Servo arm1;

    Limelight3A limelight;
    boolean rewinding = false;
    int num = 0;
    public void init() {

        rewindTimer = new ElapsedTime();
        rewindTimer.reset();

        num = 0;

//        follower = Constants.createFollower(hardwareMap);
//
//        follower.setPose(new Pose(0,144));


        fieldDrive = new FieldRelativeDrive();








        timer = new ElapsedTime();
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");


        arm1 = hardwareMap.get(Servo.class, "arm1");
        intake = hardwareMap.get(DcMotor.class, "intake");


        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");



        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        //pinpoint.resetPosAndIMU();



        intake = hardwareMap.get(DcMotor.class, "intake");

        //intake.setDirection(DcMotor.Direction.REVERSE);


        timer.reset();
        beganshot = false;
        elapsedTime = new ElapsedTime();

        elapsedTime.reset();

        hShooter = new HoodedShooter();


        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        hShooter.init(hardwareMap,telemetry, pinpoint, 24);

        state=  0;
        intake.setPower(0);

        fieldDrive.init(hardwareMap, gamepad1, pinpoint);


    }

    @Override
    public void start(){
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,144-9, 9, AngleUnit.RADIANS,0));

        hShooter.start();
        intake.setPower(1);



    }
    double val = 0.6;

    public void breaks(){
    }


    public void loop(){



        if(gamepad2.b){
            //rewindTimer.reset();
            if(intake.getPower() == 1 )intake.setPower(-1);


            //rewinding = true;
        }else{
            if(intake.getPower() == -1)intake.setPower(1);
        }





        if(gamepad2.y){
            fl.setPower(0.0);
            fr.setPower(0.0);
            br.setPower(0.0);
            bl.setPower(0.0);
        }else{
            fieldDrive.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            if(gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0 && gamepad1.right_stick_x == 0)fieldDrive.driveFieldRelative(-gamepad2.left_stick_y/4, gamepad2.left_stick_x/4, gamepad2.right_stick_x/4);

        }

        if(gamepad1.square){

            telemetry.addData("We are on square", "hooray");
            hShooter.Fire(1);
        }
        if(gamepad1.circleWasPressed()){

            telemetry.addData("We are on circle", "Wow");
            hShooter.Fire(2);
        }

        if(gamepad1.triangleWasPressed()){

            telemetry.addData("We are on triangle", "Wow");
            hShooter.Fire(3);
        }

        if(gamepad1.dpadLeftWasPressed()){
            hShooter.FireColor('g');
        }


        if(gamepad1.dpadRightWasPressed()){
            hShooter.FireColor('p');
        }





        if(gamepad1.dpadUpWasPressed()){

            hShooter.firePattern();
        }

        hShooter.loop();



        telemetry.addData("According to redslave 1 we are at : ", pinpoint.getPosX(DistanceUnit.INCH) + " " + pinpoint.getPosY(DistanceUnit.INCH) + " At an angle of " +pinpoint.getHeading(AngleUnit.RADIANS));
        //telemetry.update();

        telemetry.addData("Positions : ", hShooter.p1 + " , " + hShooter.p2 + " , " + hShooter.p3);


        telemetry.update();
    }


}
