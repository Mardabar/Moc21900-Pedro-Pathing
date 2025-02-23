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

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int LlinTarget = 0;
    public static int RlinTarget = 0;
    public static int rotatTarget = 0;
    public static int pickmeupTarget = 0;

    private final double ticks_in_degree = 537.7;

    private DcMotor rotat;
    private DcMotor pickmeup;
    private DcMotor Llin;
    private DcMotor Rlin;



    @Override
    public void init() {
        controller = new PIDController(p,i,d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        rotat = hardwareMap.get(DcMotorEx.class, "rotat");
        pickmeup = hardwareMap.get(DcMotorEx.class, "pickmeup");
        Llin = hardwareMap.get(DcMotorEx.class, "Llin");
        Rlin = hardwareMap.get(DcMotorEx.class, "Rlin");

    }

    @Override
    public void loop() {
        controller.setPID(p,i,d);
        int rotatPos = rotat.getCurrentPosition();
        /*int pickmeupPos = pickmeup.getCurrentPosition();
        int LlinPos = Llin.getCurrentPosition();
        int RlinPos = Rlin.getCurrentPosition(); */


        double rpid = rotatPID.calculate(rotatPos, rotatTarget);
        /*double Llpid = LlinPID.calculate(LlinPos, LlinTarget);
        double Rlpid = RlinPID.calculate(RlinPos, RlinTarget);
        double mpid = pickmeupPID.calculate(pickmeupPos, pickmeupTarget); */

        double rff = Math.cos(Math.toRadians(rotatTarget / ticks_in_degree)) * f;
        /*double mff = Math.cos(Math.toRadians(pickmeupTarget / ticks_in_degree)) * f;
        double Llff = Math.cos(Math.toRadians(LlinTarget / ticks_in_degree)) * f;
        double Rlff = Math.cos(Math.toRadians(RlinTarget / ticks_in_degree)) * f; */

        double rotatPower = rff + rpid;
        /*double pickmeupPower = mff + mpid;
        double LlinPower = Llff + Llpid;
        double RlinPower = Rlff + Rlpid; */

        rotat.setPower(rotatPower);
        /*pickmeup.setPower(pickmeupPower);
        Llin.setPower(LlinPower);
        Rlin.setPower(RlinPower);*/

        telemetry.addData("rotat pos", rotatPos);
        /*telemetry.addData("pickmeup pos", pickmeupPos);
        telemetry.addData("Llin pos", LlinPos);
        telemetry.addData("Rlin pos", RlinPos); */
        telemetry.update();

    }

}