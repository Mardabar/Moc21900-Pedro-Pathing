package teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "StraferOpV3")
public class StraferOpV3 extends LinearOpMode {

  private DcMotor Lf;
  private DcMotor Rf;
  private DcMotor Lb;
  private DcMotor Rb;
  private DcMotor pickMeUp;
  private DcMotor Llin;
  private DcMotor Rlin;
  private DcMotor rotat;

  private Servo imaTouchU;
  private Servo ankel;

  private TouchSensor toeA;
  private TouchSensor toeB;

  private IMU imu;
  private ElapsedTime timer = new ElapsedTime();
  private double doneTime = 0;
  private boolean timerDone = false;

  private boolean specPress = false;
  private boolean specPressB = false;
  private boolean specPressC = false;

  private double RobotSpeed = .46;
  private double armSpeed = .6;
  private double TurnSpeed = 1.6;
  private boolean bigTurn = false;
  private boolean armMovement = false;
  private boolean extendMove = false;

  private boolean inSetPos = true;
  private boolean pickUpPos = false;
  private boolean barPos = false;
  private boolean limitReached = false;

  private int LRlinSetPos = 80;
  private int pMUSetPos = 30;
  private int rotatSetPos = 100;

  private boolean SAMSMode = true;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    Lf = hardwareMap.get(DcMotor.class, "Lf");
    Rf = hardwareMap.get(DcMotor.class, "Rf");
    Lb = hardwareMap.get(DcMotor.class, "Lb");
    Rb = hardwareMap.get(DcMotor.class, "Rb");
    pickMeUp = hardwareMap.get(DcMotor.class, "pickmeup");
    Llin = hardwareMap.get(DcMotor.class, "Llin");
    Rlin = hardwareMap.get(DcMotor.class, "Rlin");
    rotat = hardwareMap.get(DcMotor.class, "rotat");

    imaTouchU = hardwareMap.get(Servo.class, "imaTouchU");
    ankel = hardwareMap.get(Servo.class, "ankel");

    toeA = hardwareMap.get(TouchSensor.class, "toe1");
    toeB = hardwareMap.get(TouchSensor.class, "toe3");

    imu = hardwareMap.get(IMU.class, "imu");

    Lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    pickMeUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Llin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Rlin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rotat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    Lf.setDirection(DcMotor.Direction.FORWARD);
    Rf.setDirection(DcMotor.Direction.REVERSE);
    Lb.setDirection(DcMotor.Direction.FORWARD);
    Rb.setDirection(DcMotor.Direction.REVERSE);
    pickMeUp.setDirection(DcMotor.Direction.REVERSE);
    Llin.setDirection(DcMotor.Direction.REVERSE);
    Rlin.setDirection(DcMotor.Direction.FORWARD);
    rotat.setDirection(DcMotor.Direction.FORWARD);

    //pickMeUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //Llin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //Rlin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //rotat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    pickMeUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Llin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Rlin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rotat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    imaTouchU.scaleRange(.2, .8);
    ankel.scaleRange(0, 1);

    liftSystem(800);
    Llin.setPower(1);
    Rlin.setPower(1);
    armRotation(rotatSetPos);
    extendoGrip(pMUSetPos);
    ankel.setPosition(.567);
    armMovement = false;
    inSetPos = true;

    waitForStart();
    while (opModeIsActive()) {
      if (!bigTurn) {
        // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
        // We negate this value so that the topmost position corresponds to maximum forward power.
        Lf.setPower(TurnSpeed * -gamepad1.right_stick_x * RobotSpeed + RobotSpeed * -gamepad1.left_stick_x + RobotSpeed * gamepad1.left_stick_y);
        Rf.setPower(TurnSpeed * gamepad1.right_stick_x * RobotSpeed + RobotSpeed * gamepad1.left_stick_x + RobotSpeed * gamepad1.left_stick_y);
        // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
        // We negate this value so that the topmost position corresponds to maximum forward power.
        Lb.setPower(TurnSpeed * -gamepad1.right_stick_x * RobotSpeed + RobotSpeed * gamepad1.left_stick_x + RobotSpeed * gamepad1.left_stick_y);
        Rb.setPower(TurnSpeed * gamepad1.right_stick_x * RobotSpeed + RobotSpeed * -gamepad1.left_stick_x + RobotSpeed * gamepad1.left_stick_y);
      }

      // When pressing right trigger, the robot goes into turbo mode.
      // When pressing left trigger, the robot goes into snail mode.
      if (gamepad1.right_trigger > 0.1) {
        RobotSpeed = 1;
        TurnSpeed = 0.85;
      }
      else if (gamepad1.left_trigger > 0.1) {
        RobotSpeed = 0.2;
        TurnSpeed = 1;
      }
      else {
        RobotSpeed = 0.42;
        TurnSpeed = 1.6;
      }

      // if ((toeA.isPressed() || toeB.isPressed()) && limitReached){
      //   Llin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      //   Rlin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      //   limitReached = false;
      // } else if (!(toeA.isPressed() || toeB.isPressed()) && !limitReached){
      //   limitReached = true;
      // }

      if (timer.milliseconds() >= doneTime){
        timerDone = true;
      }
      else{
        timerDone = false;
      }

      if (gamepad2.right_stick_button){
        SAMSMode = true;
      } else if (gamepad2.left_stick_button){
        SAMSMode = false;
      }

      // This is the Supreme Arm Movement System (SAMS)
      // This chunk of a block is near full-automatic arm movement
      // also prepare for a lot of nesting
      if (SAMSMode){
        if (gamepad2.dpad_down){
          liftSystem(LRlinSetPos);
          Llin.setPower(1);
          Rlin.setPower(1);
          extendoGrip(960);
          ankel.setPosition(.567);
          armMovement = false;
          inSetPos = false;
          pickUpPos = true;
          barPos = false;
        } else if (gamepad2.dpad_right){
          liftSystem(3200);
          Llin.setPower(1);
          Rlin.setPower(1);
          armRotation(2080);
          ankel.setPosition(.568);
          armMovement = false;
          inSetPos = false;
          pickUpPos = false;
          barPos = true;
        } else if (gamepad2.dpad_up){
          armRotation(760);
          ankel.setPosition(.658);
          armMovement = true;
          inSetPos = false;
          pickUpPos = false;
          barPos = true;
        } else if (gamepad2.dpad_left){
          armRotation(190);
          ankel.setPosition(.62);
          armMovement = true;
          inSetPos = false;
          pickUpPos = false;
          barPos = true;
        } else if (gamepad2.y){
          liftSystem(800);
          Llin.setPower(1);
          Rlin.setPower(1);
          armRotation(rotatSetPos);
          extendoGrip(pMUSetPos);
          ankel.setPosition(.567);
          armMovement = false;
          inSetPos = true;
          pickUpPos = false;
          barPos = false;
        }

        if (gamepad2.a){
          armRotation(190);
          ankel.setPosition(.626);
          imaTouchU.setPosition(.52);
          liftSystem(200);
          Llin.setPower(1);
          Rlin.setPower(1);
          armMovement = false;
          inSetPos = false;
          pickUpPos = false;
          barPos = true;
          specPress = true;
        } else if (!gamepad2.a && specPress) {
          imaTouchU.setPosition(.13);
          doneTime = 550;
          timer.reset();
          timerDone = false;
          specPressB = true;
          specPress = false;
        }

        if (timerDone && specPressB) {
          armRotation(760);
          ankel.setPosition(.658);
          liftSystem(2000);
          Llin.setPower(1);
          Rlin.setPower(1);
          specPressB = false;
        }

        // SO MUCH NESTING I HATE IT
        if (armMovement){
          if (gamepad2.right_trigger > 0.1){
            liftSystem(7800);
            Llin.setPower(1);
            Rlin.setPower(1);
          } else if (gamepad2.left_trigger > 0.1){
            liftSystem(LRlinSetPos);
            Llin.setPower(1);
            Rlin.setPower(1);
          } else {
            Llin.setPower(0);
            Rlin.setPower(0);
          }


        }

        if (barPos){
          if (gamepad2.right_bumper){
            extendoGrip(960);
          } else {
            extendoGrip(pMUSetPos);
          }
        }

        if (pickUpPos){
          if (gamepad2.left_bumper){
            armRotation(rotatSetPos);
          } else {
            armRotation(300);
          }
        }

        if (gamepad2.x){
          imaTouchU.setPosition(.13);
        } else if (gamepad2.b){
          imaTouchU.setPosition(.52);
        }



        // if (Llin.getCurrentPosition() >= 1200 && !gamepad2.dpad_up && !gamepad2.dpad_down){
        //   armRotation(1100);
        //   ankel.setPosition(.584);
        // } else if (!gamepad2.dpad_up && !gamepad2.dpad_down){
        //   armRotation(rotatSetPos);
        //   ankel.setPosition(.567);
        // } else if (gamepad2.dpad_up){
        //   armRotation(800);
        //   ankel.setPosition(.6);
        // } else if (gamepad2.dpad_down){
        //   armRotation(300);
        //   ankel.setPosition(.618);
        // }
      } else if (!SAMSMode){
        if (gamepad2.right_trigger > 0.1){
          liftSystem(8100);
          Llin.setPower(1);
          Rlin.setPower(1);
        } else if (gamepad2.left_trigger > 0.1){
          liftSystem(-5000);
          Llin.setPower(1);
          Rlin.setPower(1);
        } else {
          Llin.setPower(0);
          Rlin.setPower(0);
        }

        if (gamepad2.right_bumper){
          //pickMeUp.setPower(.3);
          extendoGripManual(5000);
          pickMeUp.setPower(.5);

        } else if (gamepad2.left_bumper){
          extendoGripManual(-5000);
          pickMeUp.setPower(.5);
        } else{
          pickMeUp.setPower(0);
        }

        if (gamepad2.x){
          imaTouchU.setPosition(.16);
        } else if (gamepad2.b){
          imaTouchU.setPosition(.5);
        }

        if (gamepad2.a){
          ankel.setPosition(.567);
        } else if (gamepad2.y){
          ankel.setPosition(.592);
        }

        if (gamepad2.dpad_up){
          armRotationMove(5000);
          rotat.setPower(armSpeed);
        } else if (gamepad2.dpad_down){
          armRotationMove(-5000);
          rotat.setPower(armSpeed);
        } else {
          rotat.setPower(0);
        }
      }

      if (gamepad1.a){
        telemetry.addData("Yaw", getAngle());
        telemetry.update();
      }

      if (gamepad1.right_bumper) {
        if (gamepad1.x) {
          RightATW(90);
        } else {
          RightIMUTurn(90);
        }
        telemetry.addData("Yaw", getAngle());
        telemetry.update();
      }
      if (gamepad1.left_bumper) {
        if (gamepad1.x) {
          LeftATW(90);
        } else {
          LeftIMUTurn(90);
        }
        telemetry.addData("Yaw", getAngle());
        telemetry.update();
      }

      if (gamepad2.a && gamepad2.y){
        pickMeUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Llin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rlin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      }
    }
  }

  public void extendoGrip(int pos) {
    pickMeUp.setTargetPosition(pos);
    pickMeUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    pickMeUp.setPower(1);
  }

  public void extendoGripManual(int pos) {
    pickMeUp.setTargetPosition(pos);
    pickMeUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  public void armRotation(int pos) {
    rotat.setTargetPosition(pos);
    rotat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rotat.setPower(.6);
  }

  public void armRotationMove(int pos) {
    rotat.setTargetPosition(pos);
    rotat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  public void liftSystem(int pos){
    Llin.setTargetPosition(pos);
    Rlin.setTargetPosition(pos);
    Llin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Rlin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  public void RightIMUTurn(double degree) {
    imu.resetYaw();
    double togNum = getAngle() - (degree - 20);
    double angularTurn = 1;
    boolean enlarge = false;

    while (getAngle() > togNum) {
      if (gamepad1.a && !enlarge){
        togNum -= 90;
        enlarge = true;
      }
      Lf.setPower(-angularTurn);
      Rf.setPower(angularTurn);
      Lb.setPower(-angularTurn);
      Rb.setPower(angularTurn);
    }
  }

  public void LeftIMUTurn(double degree) {
    imu.resetYaw();
    double togNum = getAngle() + (degree - 20);
    double angularTurn = 1;
    boolean enlarge = false;

    while (getAngle() < togNum) {
      if (gamepad1.a && !enlarge){
        togNum += 90;
        enlarge = true;
      }
      Lf.setPower(angularTurn);
      Rf.setPower(-angularTurn);
      Lb.setPower(angularTurn);
      Rb.setPower(-angularTurn);
    }
  }

  public void RightATW(double degree) {
    imu.resetYaw();
    double togNum = getAngle() + (degree - 10);
    double angularTurn = 1;

    while (getAngle() < togNum) {
      Lf.setPower(-0.08 * angularTurn);
      Rf.setPower(0.08 * angularTurn);
      Lb.setPower(angularTurn);
      Rb.setPower(-angularTurn);
    }
  }

  public void LeftATW(double degree) {
    imu.resetYaw();
    double togNum = getAngle() - (degree - 10);
    double angularTurn = 1;

    while (getAngle() > togNum) {
      Lf.setPower(0.08 * angularTurn);
      Rf.setPower(-0.08 * angularTurn);
      Lb.setPower(-angularTurn);
      Rb.setPower(angularTurn);
    }
  }

  private double getAngle() {
    return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
  }
}