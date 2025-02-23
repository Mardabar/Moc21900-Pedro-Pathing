package autonomous;

import static java.lang.Math.abs;
import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import config.Actions.RunAction;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "odo_Right_v1", group = "Autonomous")
public class odo_Right_v1 extends OpMode {

    //    private DcMotor elbow = null;
    private DcMotor motorArm1 = null;
    private DcMotor motorArm2 = null;
    Servo pinch;
    Servo diffy1, diffy2, pushBar1, pushBar2;
    private ElapsedTime movementTimer = new ElapsedTime();
    private final double ticks_in_degree = 1425;
    static final double COUNTS_PER_MOTOR_REV = 280;
    static final double DRIVE_GEAR_REDUCTION = 2.95;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    private ElapsedTime runtime = new ElapsedTime();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose score1Pose = new Pose(27.75, 0, Math.toRadians(0));
    private final Pose awayPose = new Pose(18, -25, Math.toRadians(90));
    private final Pose readyPose1 = new Pose(50, -38, Math.toRadians(180));
    private final Pose control1 = new Pose(50, -25);
    private final Pose push1 = new Pose(20, -38, Math.toRadians(180));
    private final Pose readyPose2 = new Pose(50, -48, Math.toRadians(180));
    private final Pose control2 = new Pose(48, -40);
    private final Pose push2 = new Pose(15, -46, Math.toRadians(180));
    private final Pose readyPose3 = new Pose(35.5, -48.5, Math.toRadians(270));
    private final Pose push3 = new Pose(7, -50, Math.toRadians(180));
    private final Pose control3 = new Pose(8, 4);
    private final Pose control4 = new Pose(8, -30);
    private final Pose score2Pose = new Pose(27.75, 4, Math.toRadians(0));
    private final Pose control5 = new Pose(14, -29);
    private final Pose grab2Pose = new Pose(8, -29, Math.toRadians(180));
    private final Pose control6 = new Pose(14, 8);
    private final Pose score3Pose = new Pose(27.75, 8, Math.toRadians(0));
    private final Pose control7 = new Pose(14, -29);
    private final Pose grab3Pose = new Pose(8, -29, Math.toRadians(180));
    private final Pose control8 = new Pose(14, 12);
    private final Pose score4Pose = new Pose(27.75, 12, Math.toRadians(0));
    private PathChain score1, awayFromBar, pushSamples, pushFinal, grab1, score2, grab2, score3, grab3, score4;
    private LinearOpMode OpMode;
    public DcMotor armMotor1, armMotor2, elbow;
    //public  RunAction toZero, toChamber;
    public PIDController armPID, elbowPID;
    public static double armTarget;
    public static double elbowTarget;
    public Boolean armDone;
    public int pos;
    public static double p = 0.01, i = 0, d = 0;
    public static double f = 0;

    public void setArmTarget(double b) {
        armTarget = b;
    }

    public void setElbowTarget(double b) {
        elbowTarget = b;
    }

    public int getPos() {
        pos = armMotor1.getCurrentPosition();
        return armMotor1.getCurrentPosition();
    }

    public void buildPaths() {
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(score1Pose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), score1Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(4)
                .build();
        pushSamples = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(awayPose), new Point(control1), new Point(readyPose1)))
                .setLinearHeadingInterpolation(awayPose.getHeading(), readyPose1.getHeading())
                .addPath(new BezierLine(new Point(readyPose1), new Point(push1)))
                .setLinearHeadingInterpolation(readyPose1.getHeading(), push1.getHeading())
                .addPath(new BezierCurve(new Point(push1), new Point(control2), new Point(readyPose2)))
                .setLinearHeadingInterpolation(push1.getHeading(), readyPose2.getHeading())
                .addPath(new BezierLine(new Point(readyPose2), new Point(push2)))
                .setLinearHeadingInterpolation(readyPose2.getHeading(), push2.getHeading())
                .build();
        awayFromBar = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score1Pose), new Point(awayPose)))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), awayPose.getHeading())
                .build();
        pushFinal = follower.pathBuilder()
                .addPath(new BezierLine(new Point(push2), new Point(readyPose3)))
                .setLinearHeadingInterpolation(push2.getHeading(), readyPose3.getHeading())
                .build();
        grab1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(readyPose3), new Point(push3)))
                .setLinearHeadingInterpolation(readyPose3.getHeading(), push3.getHeading())
                .build();
        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(push3), new Point(control4), new Point(control3), new Point(score2Pose)))
                .setLinearHeadingInterpolation(push3.getHeading(), control4.getHeading(), score2Pose.getHeading())
                .build();
        grab2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score2Pose), new Point(control5), new Point(grab2Pose)))
                .setLinearHeadingInterpolation(score2Pose.getHeading(), grab2Pose.getHeading())
                .build();
        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grab2Pose), new Point(control6), new Point(score3Pose)))
                .setLinearHeadingInterpolation(grab2Pose.getHeading(), score3Pose.getHeading())
                .build();
        grab3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score3Pose), new Point(control7), new Point(grab3Pose)))
                .setLinearHeadingInterpolation(score3Pose.getHeading(), grab3Pose.getHeading())
                .build();
        score4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grab3Pose), new Point(control8), new Point(score4Pose)))
                .setLinearHeadingInterpolation(grab3Pose.getHeading(), score4Pose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setElbowTarget(1200);
                setArmTarget(750);
                clawUp();
                follower.setMaxPower(.7);
                follower.followPath(score1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    pinch.setPosition(.5);
                    follower.setMaxPower(1);
                    follower.followPath(awayFromBar);
                    setArmTarget(0);
                    setElbowTarget(200);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(pushSamples);
                    setPathState(3);
                }
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(pushFinal, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    pushBar1.setPosition(0.6);
                    pushBar2.setPosition(0.38);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds() > .8) {
                    follower.followPath(grab1, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    pinch.setPosition(.3);
                    setPathState(7);
                    pathTimer.resetTimer();
                }
                break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds() > .2) {
                    setElbowTarget(700);
                    pushBar1.setPosition(0);
                    pushBar2.setPosition(.97);
                    pathTimer.resetTimer();
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > .2) {
                    setElbowTarget(1350);
                    setArmTarget(750);
                    follower.followPath(score2);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    pinch.setPosition(.5);
                    setArmTarget(0);
                    setElbowTarget(200);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(grab2);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    pinch.setPosition(.3);
                    pathTimer.resetTimer();
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() > .2) {
                    setElbowTarget(700);
                    pathTimer.resetTimer();
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTimeSeconds() > .2) {
                    setElbowTarget(1350);
                    setArmTarget(750);
                    follower.followPath(score3);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    pinch.setPosition(.5);
                    setArmTarget(0);
                    setElbowTarget(200);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(grab3);
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    pinch.setPosition(.3);
                    pathTimer.resetTimer();
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTimer.getElapsedTimeSeconds() > .2) {
                    setElbowTarget(700);
                    pathTimer.resetTimer();
                    setPathState(18);
                }
                break;
            case 18:
                if (pathTimer.getElapsedTimeSeconds() > .2) {
                    setElbowTarget(1350);
                    setArmTarget(750);
                    follower.followPath(score4);
                    setPathState(19);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void clawHorizontal() {
        diffy1.setPosition(.4);
        diffy2.setPosition(.85);
    }

    public void clawUp() {
        diffy1.setPosition(.75);
        diffy2.setPosition(.5);
    }

    public void elbowEncoderDrive(double elbowSpeed, double elbowTarget, double armSpeed, double motorArmTarget) throws InterruptedException {
        int newElbowTarget;
        int newMotorArm1Target;
        int newMotorArm2Target;
        this.OpMode = OpMode;
        LinearOpMode OpMode = null;
        // Ensure that the opmode is still active
        // Determine new target position, and pass to motor controller
        DcMotor elbow = hardwareMap.get(DcMotor.class, "m4");
        DcMotor motorArm1 = hardwareMap.get(DcMotor.class, "m5");
        DcMotor motorArm2 = hardwareMap.get(DcMotor.class, "m6");
        newElbowTarget = elbow.getCurrentPosition() + (int) (elbowTarget * COUNTS_PER_INCH);
        newMotorArm1Target = motorArm1.getCurrentPosition() + (int) (motorArmTarget * COUNTS_PER_INCH);
        newMotorArm2Target = motorArm2.getCurrentPosition() + (int) (motorArmTarget * COUNTS_PER_INCH);
        elbow.setTargetPosition(newElbowTarget);
        motorArm1.setTargetPosition(-newMotorArm1Target);
        motorArm2.setTargetPosition(-newMotorArm2Target);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // reset the timeout time and start motion.
        runtime.reset();
        elbow.setPower(Math.abs(elbowSpeed));
        motorArm1.setPower(Math.abs(armSpeed));
        motorArm2.setPower(Math.abs(armSpeed));
        while (elbow.isBusy()) {
            sleep(1000);
        }
    }

    @Override
    public void init() {
        pinch = hardwareMap.servo.get("s0");
        diffy1 = hardwareMap.servo.get("s1");
        diffy2 = hardwareMap.servo.get("s2");
        pushBar1 = hardwareMap.servo.get("s3");
        pushBar2 = hardwareMap.servo.get("s4");
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        armPID = new PIDController(p, i, d);
        elbow = hardwareMap.get(DcMotor.class, "m4");
        armMotor1 = hardwareMap.get(DcMotor.class, "m5");
        armMotor2 = hardwareMap.get(DcMotor.class, "m6");
        elbow.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor1.setDirection(DcMotor.Direction.FORWARD);
        armMotor2.setDirection(DcMotor.Direction.REVERSE);
        elbowPID = new PIDController(p, i, d);
        clawHorizontal();
        pinch.setPosition(.3);
        pushBar1.setPosition(0);
        pushBar2.setPosition(.97);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void loop() {
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        armPID.setPID(p, i, d);
        int armPos = armMotor1.getCurrentPosition();
        double apid = armPID.calculate(armPos, armTarget);
        double aff = Math.cos(Math.toRadians(armTarget / ticks_in_degree)) * f;
        double armPower = aff + apid;
        armMotor1.setPower(armPower);
        armMotor2.setPower(armPower);
        elbowPID.setPID(p, i, d);
        int elbowPos = elbow.getCurrentPosition();
        double epid = elbowPID.calculate(elbowPos, elbowTarget);
        double eff = Math.cos(Math.toRadians(elbowTarget / ticks_in_degree)) * f;
        double elbowPower = eff + epid;
        elbow.setPower(elbowPower);
        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}