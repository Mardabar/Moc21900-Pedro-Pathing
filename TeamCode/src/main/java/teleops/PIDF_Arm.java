package teleops;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class PIDF_Arm extends OpMode {

    public PIDController controller;
    public PIDController LlinPID, rotatPID, pickmeupPID, RlinPID;

    public static double pR = 0.0085, iR = 0.01, dR = 0.000083;
    public static double fR = 0.1;
    public static double p = 0.0025, i = 0.01, d = 0.000001;
    public static double f = 0.1;
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
    private final double ticks_in_degree = 537.7;
    public final double rotat_ticks_in_degree = 3895.9; // ticks in degree for the 43 rpm motor

    private DcMotor rotat;
    private DcMotor pickmeup;
    private DcMotor Llin;
    private DcMotor Rlin;



    @Override
    public void init() {
        controller = new PIDController(p,i,d);
        LlinPID = new PIDController(p,i,d);
        RlinPID = new PIDController(p,i,d);
        rotatPID = new PIDController(pR, iR, dR);
        pickmeupPID = new PIDController(p, i, d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        rotat = hardwareMap.get(DcMotorEx.class, "rotat");
        pickmeup = hardwareMap.get(DcMotorEx.class, "pickmeup");
        Llin = hardwareMap.get(DcMotorEx.class, "Llin");
        Rlin = hardwareMap.get(DcMotorEx.class, "Rlin");


        pickmeup.setDirection(DcMotor.Direction.REVERSE);
        Llin.setDirection(DcMotor.Direction.REVERSE);
        Rlin.setDirection(DcMotor.Direction.FORWARD);
        rotat.setDirection(DcMotor.Direction.FORWARD);

        pickmeup.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Llin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rlin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pickmeup.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Llin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rlin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        controller.setPID(p,i,d);
        LlinPID.setPID(p,i,d);
        RlinPID.setPID(p,i,d);
        rotatPID.setPID(pR,iR,dR);
        pickmeupPID.setPID(p,i,d);

        int LlinPos = Llin.getCurrentPosition();
        int RlinPos = Rlin.getCurrentPosition();
        int rotatPos = rotat.getCurrentPosition();
        int pickmeupPos = pickmeup.getCurrentPosition();

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
        pickmeup.setPower(pickmeupPower);

        telemetry.addData("rotat pos", rotatPos);
        telemetry.addData("pickmeup pos", pickmeupPos);
        telemetry.addData("Llin pos", LlinPos);
        telemetry.addData("Rlin pos", RlinPos);
        telemetry.update();

    }

}