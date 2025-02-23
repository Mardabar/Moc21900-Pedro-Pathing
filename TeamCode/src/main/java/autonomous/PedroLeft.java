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

@Autonomous(name = "Pedro Left", group = "autonomous")
public class PedroLeft extends OpMode {

    private DcMotor Rlin = null;
    private DcMotor Llin = null;
    private DcMotor pickMeUp = null;
    private DcMotor rotat = null;
    Servo ankel;
    Servo imatouchU;

    private ElapsedTime movementTimer = new ElapsedTime();
    private final double ticks_in_degree = 1425;
    static final double COUNTS_PER_MOTOR_REV = 280;
    static final double DRIVE_GEAR_REDUCTION = 2.95;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    private final Pose startPose = new Pose(8, 70, Math.toRadians(0));  // Starting position
    private final Pose scoreprePose = new Pose(33, 70, Math.toRadians(0)); // Scoring position
    private final Pose linewith1Pose = new Pose(28,70, Math.toRadians(0)); // move robot back a bit after score pre

    //private final Pose cuvrewith1Pose = new Pose()




    private final Pose strafeto1Pose = new Pose(28,46, Math.toRadians(90));
    private final Pose lineto1Pose = new Pose(56,40, Math.toRadians(180));
    private final Pose moveto1Pose = new Pose(59, 36, Math.toRadians(180)); // Lining up for sample with beziure curve idk how to spell
    private final Pose linewithsamp1Pose = new Pose(59,23, Math.toRadians(180)); // Strafing to line up with sample again b4 pushing to human player zone
    private final Pose pushsample1Pose = new Pose(17,23, Math.toRadians(180)); // Pushes 1st sample to da hmn plyer zone


    private final Pose linewithsamp2Pose = new Pose(59,14, Math.toRadians(180)); // Robot goes to 2nd sample using bezuier curve or smth
    private final Pose linewithsamp2ControlPose = new Pose(58.5,56, Math.toRadians(180)); /**** Using this bezieur curve EXPECT TO CHANGE THESE VALUES CAUSE DONT HAVE FIELD RN****/

    private final Pose pushsample2Pose = new Pose(17,14, Math.toRadians(180));


    private final Pose linewithsamp3Pose = new Pose(64,9, Math.toRadians(180)); // bot goes to 3rd sample using bezuer curve
    private final Pose linewithsamp3ControlPose = new Pose(49,40, Math.toRadians(180)); /**** Using this bezieur curve EXPECT TO CHANGE THESE VALUES **/
    private final Pose pushsample3Pose = new Pose(17,9, Math.toRadians(180)); // pushin in the final samp

    private final Pose pickupspecPose = new Pose(11,30, Math.toRadians(180)); // this picks up specimin from human play zone use multiple times





    private final Pose scorespecPose = new Pose(18, 129, Math.toRadians(0)); // Score sample from in claw
    /*private final Pose pickup2Pose = new Pose(31, 131, Math.toRadians(0)); // Second sample pickup
    private final Pose pickup3Pose = new Pose(34, 135, Math.toRadians(0)); // Third sample pickup

    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));    // Parking position
    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90)); // Control point for curved path */

    private Path scorePre, park;

    private PathChain movetofirst, pushinsamps, pushinsamp1, linewithsamp2, pushinsamp2, linewithsamp3, pushinsamp3, pickspecup, scorespec;

    private LinearOpMode OpMode;
    public DcMotor armMotor1, armMotor2, elbow;
    // public  RunAction toZero, toChamber;
    public PIDController PickmeupPID, elbowPID;
    public Boolean armDone;
    public int pos;
    public static double p = 0.01, i = 0, d = 0;
    public static double f = 0;

    public PIDController LlinPID, rotatPID, pickmeupPID;
    public static double LlinTarget;
    public static double rotatTarget;
    public static double pickmeupTarget;

    public void setLlinTarget(double b) {
        LlinTarget = b;
    }
    public void setRotatTarget(double b) {
        rotatTarget = b;
    }
    public void setpickmeupTarget(double b) {
        pickmeupTarget = b;
    }


    public void buildPaths() {
        // Path for scoring preload
        scorePre = new Path(new BezierLine(new Point(startPose), new Point(scoreprePose)));
        scorePre.setLinearHeadingInterpolation(startPose.getHeading(), scoreprePose.getHeading());

        // Path chains for picking up and scoring samples
        movetofirst = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreprePose), new Point(linewith1Pose)))
                .setLinearHeadingInterpolation(scoreprePose.getHeading(), linewith1Pose.getHeading())
                .addPath(new BezierLine(new Point(linewith1Pose), new Point(strafeto1Pose)))
                .setLinearHeadingInterpolation(linewith1Pose.getHeading(), strafeto1Pose.getHeading())
                /*.addPath(new BezierLine(new Point(strafeto1Pose), new Point(lineto1Pose)))
                .setLinearHeadingInterpolation(strafeto1Pose.getHeading(), lineto1Pose.getHeading())
                .addPath(new BezierLine(new Point(lineto1Pose), new Point(linewithsamp1Pose)))
                .setLinearHeadingInterpolation(lineto1Pose.getHeading(), linewithsamp1Pose.getHeading())*/
                .build();

         /*lineback1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreprePose), new Point(linewith1Pose)))
                .setLinearHeadingInterpolation(scoreprePose.getHeading(), linewith1Pose.getHeading())
                .build();

        strafinTo1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(linewith1Pose), new Point(strafeto1Pose)))
                .setLinearHeadingInterpolation(linewith1Pose.getHeading(), strafeto1Pose.getHeading())
                .build();

        lininTo1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(strafeto1Pose), new Point(lineto1Pose)))
                .setLinearHeadingInterpolation(strafeto1Pose.getHeading(), lineto1Pose.getHeading())
                .build();

        linewithsamp1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(lineto1Pose), new Point(linewithsamp1Pose)))
                .setLinearHeadingInterpolation(lineto1Pose.getHeading(), linewithsamp1Pose.getHeading())
                .build(); */

        pushinsamp1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(linewithsamp1Pose), new Point(pushsample1Pose)))
                .setLinearHeadingInterpolation(linewithsamp1Pose.getHeading(), pushsample1Pose.getHeading())
                .build();

        linewithsamp2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pushsample1Pose), new Point(linewithsamp2ControlPose),new Point(linewithsamp2Pose)))
                .setLinearHeadingInterpolation(pushsample1Pose.getHeading(), linewithsamp2Pose.getHeading())
                .build();

        pushinsamp2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(linewithsamp2Pose), new Point(pushsample2Pose)))
                .setLinearHeadingInterpolation(linewithsamp2Pose.getHeading(), pushsample2Pose.getHeading())
                .build();

        linewithsamp3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pushsample2Pose), new Point(linewithsamp3ControlPose),new Point(linewithsamp3Pose)))
                .setLinearHeadingInterpolation(pushsample2Pose.getHeading(), linewithsamp3Pose.getHeading())
                .build();

        pushinsamp3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(linewithsamp3Pose), new Point(pushsample3Pose)))
                .setLinearHeadingInterpolation(linewithsamp3Pose.getHeading(), pushsample3Pose.getHeading())
                .build();

        pickspecup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushsample3Pose), new Point(scorespecPose)))
                .setLinearHeadingInterpolation(pushsample3Pose.getHeading(), scorespecPose.getHeading())
                .build();

        /*scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(moveto1Pose), new Point(scorespecPose)))
                .setLinearHeadingInterpolation(moveto1Pose.getHeading(), scorespecPose.getHeading())
                .build();

        /*grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoresamplePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scoresamplePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scoresamplePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scoresamplePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoresamplePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scoresamplePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scoresamplePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scoresamplePose.getHeading())
                .build(); */

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to scoring position
                follower.followPath(scorePre);
                setPathState(1);
                break;

            case 1: // Wait until the robot is near the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(movetofirst, true);
                    setPathState(2);
                }
                break; /*
            case 2: // bot strafes to right of sub zone
                if (!follower.isBusy()) {
                    follower.followPath(strafinTo1,true);
                    setPathState(3);
                }
                break;
            case 3: // bot goes forward to sub zone
                if (!follower.isBusy()) {
                    follower.followPath(lininTo1,true);
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


    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init() {
        /* call servos here do later tho after ask ty
        ankel = hardwareMap.servo.get("ankel"); */
        // setting timers
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        /* setting motor pid values
        LlinPID = new PIDController(p,i,d);
        rotatPID = new PIDController(p, i, d);
        pickmeupPID = new PIDController(p, i, d); */

        // setting motor directions & names
        //Llin = hardwareMap.get(DcMotor.class, "Llin");

        // setting paths for motor
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
        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("acceleration", follower.getAcceleration());
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

