package autonomous;

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

@Autonomous(name = "Pedro Turning Test", group = "autonomous")
public class PedroTurningTest extends OpMode {

    private DcMotor Rlin = null;
    private DcMotor Llin = null;
    private DcMotor pickMeUp = null;
    private DcMotor rotat = null;
    Servo ankel;
    Servo imaTouchU;

    private ElapsedTime movementTimer = new ElapsedTime();
    private final double ticks_in_degree = 537.7;
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
    private final Pose scoreprePose = new Pose(38, 70, Math.toRadians(0)); // Scoring position
    private final Pose linewith1Pose = new Pose(30,70, Math.toRadians(0)); // move robot back a bit after score pre

    //private final Pose cuvrewith1Pose = new Pose()



    private final Pose strafeto1Pose = new Pose(30,37, Math.toRadians(0));

    // private final Pose turn180Pose = new Pose(30,37, Math.toRadians(180));
    private final Pose lineto1Pose = new Pose(56,40, Math.toRadians(0));
    //private final Pose moveto1Pose = new Pose(59, 36, Math.toRadians(0)); // Lining up for sample with beziure curve idk how to spell
    private final Pose linewithsamp1Pose = new Pose(62,28, Math.toRadians(0)); // Strafing to line up with sample again b4 pushing to human player zone
    private final Pose pushsample1Pose = new Pose(20,28, Math.toRadians(0)); // Pushes 1st sample to da hmn plyer zone


    private final Pose linewithsamp2Pose = new Pose(59,24, Math.toRadians(0)); // Robot goes to 2nd sample using bezuier curve or smth
    private final Pose linewithsamp2ControlPose = new Pose(58.5,56, Math.toRadians(0)); /**** Using this bezieur curve EXPECT TO CHANGE THESE VALUES CAUSE DONT HAVE FIELD RN****/

    private final Pose pushsample2Pose = new Pose(20,24, Math.toRadians(0));


    private final Pose linewithsamp3Pose = new Pose(64,14, Math.toRadians(0)); // bot goes to 3rd sample using bezuer curve
    private final Pose linewithsamp3ControlPose = new Pose(49,40, Math.toRadians(0)); /**** Using this bezieur curve EXPECT TO CHANGE THESE VALUES **/
    private final Pose pushsample3Pose = new Pose(20,14, Math.toRadians(0)); // pushin in the final samp

    private final Pose turntoHPZPose = new Pose(25,14, Math.toRadians(180));
    private final Pose pickupspecPose = new Pose(13,30, Math.toRadians(180)); // this picks up specimin from human play zone use multiple times





    private final Pose scorespecPose = new Pose(18, 129, Math.toRadians(0)); // Score sample from in claw
    /*private final Pose pickup2Pose = new Pose(31, 131, Math.toRadians(0)); // Second sample pickup
    private final Pose pickup3Pose = new Pose(34, 135, Math.toRadians(0)); // Third sample pickup

    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));    // Parking position
    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90)); // Control point for curved path */

    // private Path scorePre, park;

    private PathChain scorePre, movetofirst, pushinsamps, pickspecup, scorespec;

    private LinearOpMode OpMode;
    public DcMotor armMotor1, armMotor2, elbow;
    // public  RunAction toZero, toChamber;
    public PIDController PickmeupPID, elbowPID;
    public Boolean armDone;
    public int pos;
    public static double p = 0.01, i = 0, d = 0.0001;
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
                .addPath(new BezierLine(new Point(pushsample3Pose), new Point(turntoHPZPose)))
                .setLinearHeadingInterpolation(pushsample3Pose.getHeading(), turntoHPZPose.getHeading())
                .build();

        pickspecup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(turntoHPZPose), new Point(pickupspecPose)))
                .setLinearHeadingInterpolation(turntoHPZPose.getHeading(), pickupspecPose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to scoring position
                follower.followPath(scorePre);

                setRotatTarget(950);
                setpickmeupTarget(450);
                setPathState(1);
                break;

            case 1: // Wait until the robot is near the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(movetofirst);
                    imaTouchU.setPosition(.58);
                    setRotatTarget(10);
                    setpickmeupTarget(10);
                    setPathState(2);
                }
                break;
            case 2: // bot strafes to right of sub zone
                if (!follower.isBusy()) {
                    follower.followPath(pushinsamps);
                    setPathState(3);
                }
                break;
            case 3: // bot
                if (!follower.isBusy()) {
                    follower.followPath(pickspecup);
                    setPathState(4);
                }
                //break;
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
                } */

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

    /*public void armDrive(double LlinSpeed, double LlinTarget, double rotatSpeed, double rotatTarget) throws InterruptedException {
        int newrotatTarget;
        int newLlinTarget;
        int newpickMeUpTarget;
        this.OpMode = OpMode;
        LinearOpMode OpMode = null;
        DcMotor Llin = hardwareMap.get(DcMotor.class, "Llin");
        DcMotor Rlin = hardwareMap.get(DcMotor.class, "Rlin");
        DcMotor rotat = hardwareMap.get(DcMotor.class, "rotat");
        DcMotor pickMeUp = hardwareMap.get(DcMotor.class, "pickMeUp");
        newrotatTarget = rotat.getCurrentPosition() + (int) (rotatTarget * COUNTS_PER_INCH);
        newLlinTarget = Llin.getCurrentPosition() + (int) (LlinTarget * COUNTS_PER_INCH);
        newpickMeUpTarget = pickMeUp.getCurrentPosition() + (int) (pickmeupTarget * COUNTS_PER_INCH);
    } */

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
        rotatPID = new PIDController(p, i, d);
        pickmeupPID = new PIDController(p, i, d);

        pickMeUp.setDirection(DcMotor.Direction.REVERSE);
        Llin.setDirection(DcMotor.Direction.REVERSE);
        Rlin.setDirection(DcMotor.Direction.FORWARD);
        rotat.setDirection(DcMotor.Direction.FORWARD);

        clampClaw();
        moveClaw();
        ankel.setPosition(.658);
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
        rotatPID.setPID(p,i,d);
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
        double rff = Math.cos(Math.toRadians(rotatTarget / ticks_in_degree)) * f;
        double mff = Math.cos(Math.toRadians(pickmeupTarget / ticks_in_degree)) * f;

        double LlinPower = Llff + Llpid;
        double RlinPower = Rlff + Rlpid;
        double rotatPower = rff + rpid;
        double pickmeupPower = mff + mpid;

        Llin.setPower(LlinPower);
        Rlin.setPower(RlinPower);
        rotat.setPower(rotatPower);
        pickMeUp.setPower(pickmeupPower);

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getTotalHeading());
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

