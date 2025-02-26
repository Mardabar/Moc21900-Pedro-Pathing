package autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDController;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "Pedro Right", group = "autonomous")
public class PedroRight extends OpMode {

    private DcMotor Rlin = null;
    private DcMotor Llin = null;
    private DcMotor pickMeUp = null;
    private DcMotor rotat = null;
    Servo ankel;
    Servo imaTouchU;

    private ElapsedTime timer = new ElapsedTime();
    double dur = 1200;
    int timerCount = -1;
    public final double ticks_in_degree = 537.7; // ticks in degree for 312 motors
    public final double rotat_ticks_in_degree = 3895.9; // ticks in degree for the 43 rpm motor
    static final double COUNTS_PER_MOTOR_REV = 280;
    static final double DRIVE_GEAR_REDUCTION = 2.95;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    private final Pose startPose = new Pose(8.5, 70, Math.toRadians(0));  // Starting position
    private final Pose scoreprePose = new Pose(34, 70, Math.toRadians(0)); // Scoring position
    private final Pose linewith1Pose = new Pose(30,70, Math.toRadians(90)); // move robot back a bit after score pre

    //private final Pose cuvrewith1Pose = new Pose()



    private final Pose strafeto1Pose = new Pose(30,37, Math.toRadians(180));

   // private final Pose turn180Pose = new Pose(30,37, Math.toRadians(180));
    private final Pose lineto1Pose = new Pose(56,40, Math.toRadians(180));
    //private final Pose moveto1Pose = new Pose(59, 36, Math.toRadians(0)); // Lining up for sample with beziure curve idk how to spell
    private final Pose linewithsamp1Pose = new Pose(58,28, Math.toRadians(180)); // Strafing to line up with sample again b4 pushing to human player zone
    private final Pose pushsample1Pose = new Pose(23,28, Math.toRadians(180)); // Pushes 1st sample to da hmn plyer zone


    private final Pose linewithsamp2Pose = new Pose(58,20, Math.toRadians(180)); // Robot goes to 2nd sample using bezuier curve or smth
    private final Pose linewithsamp2ControlPose = new Pose(55,43, Math.toRadians(180)); /**** Using this bezieur curve EXPECT TO CHANGE THESE VALUES CAUSE DONT HAVE FIELD RN****/

    private final Pose pushsample2Pose = new Pose(23,20, Math.toRadians(180));


    private final Pose linewithsamp3Pose = new Pose(58,15, Math.toRadians(180)); // bot goes to 3rd sample using bezuer curve
    private final Pose linewithsamp3ControlPose = new Pose(49,40, Math.toRadians(180)); /**** Using this bezieur curve EXPECT TO CHANGE THESE VALUES **/

    private final Pose pushsample3Pose = new Pose(23,15, Math.toRadians(180)); // pushin in the final samp

    private final Pose pickupspec2Pose = new Pose(26,15, Math.toRadians(180)); // this picks up specimin from human play zone use multiple times

    private final Pose pickupspecsPose = new Pose(13,23, Math.toRadians(180)); // this picks up specimin from human play zone use multiple times

    private final Pose control2 = new Pose(10,64);
    private final Pose control3 = new Pose(15,72);
    private final Pose linescore2 = new Pose(30,68, Math.toRadians(0));
    private final Pose scorespec2 = new Pose(38,68, Math.toRadians(0));


    private final Pose scorespec3 = new Pose(38,66, Math.toRadians(0));


    private final Pose scorespec4 = new Pose(38,64, Math.toRadians(0));


    private final Pose scorespec5 = new Pose(38,62, Math.toRadians(0));





    private final Pose scorespecPose = new Pose(18, 129, Math.toRadians(180)); // Score sample from in claw
    /*private final Pose pickup2Pose = new Pose(31, 131, Math.toRadians(0)); // Second sample pickup
    private final Pose pickup3Pose = new Pose(34, 135, Math.toRadians(0)); // Third sample pickup

    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));    // Parking position
    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90)); // Control point for curved path */

   // private Path scorePre, park;

    private PathChain scorePre, movetofirst, pushinsamps, pickspecup, line2score, score2spec;

    private LinearOpMode OpMode;

    public Boolean armDone;
    public int pos;
    public static double pR = 0.0085, iR = 0.01, dR = 0.0001;
    public static double fR = 0;
    public static double p = 0.00067, i = 0.01, d = 0.0005;
    public static double f = 0;

    public PIDController LlinPID, rotatPID, pickmeupPID, RlinPID;
    public static double LlinTarget;
    public static double RlinTarget;
    public static double rotatTarget;
    public static double pickmeupTarget;

    public void setLlinTarget(double b) {
        LlinTarget = b;
    }
    public void setRlinTarget(double b) {
        RlinTarget = b;
    }
    public void setRotatTarget(double b) {
        rotatTarget = b;
    }
    public void setpickmeupTarget(double b) {
        pickmeupTarget = b;
    }
    public int getPos() {
        pos = rotat.getCurrentPosition();
        return rotat.getCurrentPosition();
    }


    public void buildPaths() {
        // Path for scoring preload
        scorePre = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scoreprePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scoreprePose.getHeading())
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        // Path chains for picking up and scoring samples
        movetofirst = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreprePose), new Point(linewith1Pose)))
                .setLinearHeadingInterpolation(scoreprePose.getHeading(), linewith1Pose.getHeading())
                .addPath(new BezierLine(new Point(linewith1Pose), new Point(strafeto1Pose)))
                .setLinearHeadingInterpolation(linewith1Pose.getHeading(), strafeto1Pose.getHeading())
                .addPath(new BezierLine(new Point(strafeto1Pose), new Point(lineto1Pose)))
                .setLinearHeadingInterpolation(strafeto1Pose.getHeading(), lineto1Pose.getHeading())
                .addPath(new BezierLine(new Point(lineto1Pose), new Point(linewithsamp1Pose)))
                .setLinearHeadingInterpolation(lineto1Pose.getHeading(), linewithsamp1Pose.getHeading())
                .build();

         pushinsamps = follower.pathBuilder()
                 .addPath(new BezierLine(new Point(linewithsamp1Pose), new Point(pushsample1Pose)))
                 .setLinearHeadingInterpolation(linewithsamp1Pose.getHeading(), pushsample1Pose.getHeading())
                 .addPath(new BezierCurve(new Point(pushsample1Pose), new Point(linewithsamp2ControlPose),new Point(linewithsamp2Pose)))
                 .setLinearHeadingInterpolation(pushsample1Pose.getHeading(), linewithsamp2Pose.getHeading())
                 .addPath(new BezierLine(new Point(linewithsamp2Pose), new Point(pushsample2Pose)))
                 .setLinearHeadingInterpolation(linewithsamp2Pose.getHeading(), pushsample2Pose.getHeading())
                 .addPath(new BezierCurve(new Point(pushsample2Pose), new Point(linewithsamp3ControlPose),new Point(linewithsamp3Pose)))
                 .setLinearHeadingInterpolation(pushsample2Pose.getHeading(), linewithsamp3Pose.getHeading())
                 .addPath(new BezierLine(new Point(linewithsamp3Pose), new Point(pushsample3Pose)))
                 .setLinearHeadingInterpolation(linewithsamp3Pose.getHeading(), pushsample3Pose.getHeading())
                 .build();

        pickspecup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushsample3Pose), new Point(pickupspec2Pose)))
                .setLinearHeadingInterpolation(pushsample3Pose.getHeading(), pickupspec2Pose.getHeading())
                .build();

        line2score = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupspec2Pose), new Point(linescore2)))
                .setLinearHeadingInterpolation(pickupspec2Pose.getHeading(), linescore2.getHeading())
                .build();

        score2spec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(linescore2), new Point(scorespec2)))
                .setLinearHeadingInterpolation(linescore2.getHeading(), scorespec2.getHeading())
                .build();



            //gotospec3 = follower.pathBuilder()
               //.addPath(new BezierLine(new Point(s)))

    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to scoring position
                if (!follower.isBusy() && timerCount == -1) {
                    follower.followPath(scorePre);
                    follower.setMaxPower(.8);
                    dur = 200;
                    timer.reset();

                    setRotatTarget(1500);
                }

                if (timer.milliseconds() >= dur && timerCount == -1){
                    timerCount = 0;
                }

                if (!follower.isBusy() && timerCount == 0) {
                    dur = 600;
                    timerCount = -1;
                    setPathState(1);
                }
                break;

            case 1:
                if (timerCount == -1) {
                    setRotatTarget(1300);
                    timer.reset();
                    timerCount = 0;
                }
                break;

            case 2: // Wait until the robot is near the scoring position
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(movetofirst);
                    imaTouchU.setPosition(.58);

                    setPathState(2);
                }
                break;
            case 3: // bot strafes to right of sub zone
                if (!follower.isBusy()) {
                    follower.followPath(pushinsamps);
                    setRotatTarget(500);
                    ankel.setPosition(.62);
                    imaTouchU.setPosition(.58);
                    setPathState(3);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(pickspecup);
                    setPathState(4);
                }
                break;
            case 5: // bot
                if (!follower.isBusy()) {
                    ankel.setPosition(.658); // was .658
                    imaTouchU.setPosition(.16);
                    setPathState(5);
                    setRotatTarget(1300);
                    setpickmeupTarget(450);
                    follower.followPath(line2score);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    ankel.setPosition(.658); // was .658
                    imaTouchU.setPosition(.16);
                    setRotatTarget(1300);
                    setpickmeupTarget(450);
                    follower.followPath(score2spec);
                    setPathState(6);
                }
           /* case 5:
                if (!follower.isBusy()) {
                    setRotatTarget(1300);
                    setpickmeupTarget(470);

                    follower.followPath(score2spec);
                    setPathState(6);
                } */
            /*case 4: // pick up 2nd spec
                 if (!follower.isBusy()) {
                    follower.followPath(pickspecup);



                    follower.followPath(score2spec);
                    setPathState(5);
                }
                /*
            case 4: // bot should move to first samp ready to push
                if (!follower.isBusy()) {
                    follower.followPath(linewithsamp1,true);
                    setPathState(5);
                }

            case 3: //bot lines up w sample
                if (follower.isBusy()) {
                    follower.followPath(linewithsamp1,true);
                    setPathState(4);
                }
            case 4: //bot pushes 1st sample YIPPE
                if (follower.isBusy()) {
                    follower.followPath(pushinsamp1,true);
                    setPathState(5);
                }
            case 5: // bot lines up with 2nd samp
                if (follower.isBusy()) {
                    follower.followPath(linewithsamp2);
                    setPathState(6);
                }
            case 6: // bot pushes in 2nd samp
                if (follower.isBusy()) {
                    follower.followPath(pushinsamp2, true);
                    setPathState(7);
                }
            case 7: // bot line up with 3rd samp
                if (follower.isBusy()) {
                    follower.followPath(linewithsamp3,true);
                    setPathState(8);
                }
            case 8: // bot pushes final samp in
                if (follower.isBusy()) {
                    follower.followPath(pushinsamp3,true);
                    setPathState(9);
                }
            case 9: // pick up spec 1
                if (follower.isBusy()) {
                    follower.followPath(pickspecup, true);
                    setPathState(10);
                }

             */
                /***** Guy gave this from dscord
            case 0:
                follower.followPath(scorePre);
                rotat.setTargetPosition(1500);
                setPathState(1);
            case 1:
                if(!follower.isBusy()){
                    rotat.setTargetPosition(1400);
                    setPathState(2);
                }
            case 2:
                if(Math.abs(rotat.getCurrentPosition() - 1400) < 10){ //if the arm has reached, also we already know the robot is stationary
                    pathTimer.resetTimer();//ideally ur setState method will do this, and i think the default setPathState does this
                    imaTouchU.setPosition(.58);
                    setPathState(3);
                }
            case 3:
                if(pathTimer.getElapsedTimeSeconds() > .250) {
                    follower.followPath(movetofirst);
                }
                //move on with states

            case 4:
                //move on with states *****/

            case 10: // Wait until the robot is near the parking position
                if (!follower.isBusy()) {
                    setPathState(-1); // End the autonomous routine
                }
                break;
        }
    }


    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void clampClaw() {
        imaTouchU.setPosition(.16);
        imaTouchU.setPosition(.58);
    }

    public void moveClaw() {
        ankel.setPosition(.658);
        ankel.setPosition(.567);
    }


    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init() {
        //call servos here do later tho after ask ty
        pickMeUp = hardwareMap.get(DcMotor.class, "pickmeup");
        Llin = hardwareMap.get(DcMotor.class, "Llin");
        Rlin = hardwareMap.get(DcMotor.class, "Rlin");
        rotat = hardwareMap.get(DcMotor.class, "rotat");

        imaTouchU = hardwareMap.get(Servo.class, "imaTouchU");
        ankel = hardwareMap.get(Servo.class, "ankel");

        // setting timers
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // setting motor pid values
        LlinPID = new PIDController(p,i,d);
        RlinPID = new PIDController(p,i,d);
        rotatPID = new PIDController(pR, iR, dR);
        pickmeupPID = new PIDController(p, i, d);

        pickMeUp.setDirection(DcMotor.Direction.REVERSE);
        Llin.setDirection(DcMotor.Direction.REVERSE);
        Rlin.setDirection(DcMotor.Direction.FORWARD);
        rotat.setDirection(DcMotor.Direction.FORWARD);

        pickMeUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Llin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rlin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pickMeUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Llin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rlin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clampClaw();
        moveClaw();
        ankel.setPosition(.62); // was .658
        imaTouchU.setPosition(.16);




        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void loop() {
        //these loop the movements of the wobot
        follower.update();
        autonomousPathUpdate();
        LlinPID.setPID(p,i,d);
        RlinPID.setPID(p,i,d);
        rotatPID.setPID(pR,iR,dR);
        pickmeupPID.setPID(p,i,d);

        int LlinPos = Llin.getCurrentPosition();
        int RlinPos = Rlin.getCurrentPosition();
        int rotatPos = rotat.getCurrentPosition();
        int pickmeupPos = pickMeUp.getCurrentPosition();

        double Llpid = LlinPID.calculate(LlinPos, LlinTarget);
        double Rlpid = RlinPID.calculate(RlinPos, RlinTarget);
        double rpid = rotatPID.calculate(rotatPos, rotatTarget);
        double mpid = pickmeupPID.calculate(pickmeupPos, pickmeupTarget);

        double Llff = Math.cos(Math.toRadians(LlinTarget / ticks_in_degree)) * f;
        double Rlff = Math.cos(Math.toRadians(RlinTarget / ticks_in_degree)) * f;
        double rff = Math.cos(Math.toRadians(rotatTarget / rotat_ticks_in_degree)) * fR;
        double mff = Math.cos(Math.toRadians(pickmeupTarget / ticks_in_degree)) * f;

        double LlinPower = Llff + Llpid;
        double RlinPower = Rlff + Rlpid;
        double rotatPower = rff + rpid;
        double pickmeupPower = mff + mpid;

        Llin.setPower(LlinPower);
        Rlin.setPower(LlinPower);
        rotat.setPower(rotatPower);
        pickMeUp.setPower(pickmeupPower);

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getTotalHeading());
        telemetry.addData("rotat pos", rotatPos);
        telemetry.addData("rotat target pos", rotatTarget);
        telemetry.update();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

