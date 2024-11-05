package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;

@TeleOp(name = "Ver.1.2")
public class TeleOp1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double PPR = 384.5;
        double pi = 3.14159;
        Motor FL = new Motor(hardwareMap, "FL", PPR, "left");
        Motor FR = new Motor(hardwareMap, "FR", PPR, "right");
        Motor BL = new Motor(hardwareMap, "BL", PPR, "left");
        Motor BR = new Motor(hardwareMap, "BR", PPR, "right");

        Motor slideMotor = new Motor(hardwareMap, "slideMotor", PPR, "right");
        slideMotor.ZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor armMotor = new Motor(hardwareMap, "armMotor", PPR, "right");
        armMotor.ZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // Servo continuousServo = hardwareMap.get(Servo.class, "intakeServo");



        ArrayList<Motor> tempMotors = new ArrayList<Motor>();
        tempMotors.add(FL);
        tempMotors.add(FR);
        tempMotors.add(BL);
        tempMotors.add(BR);
        MotorGroup motors = new MotorGroup(tempMotors);

        motors.reset();
        motors.ZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        SparkFunOTOS myOtos;

        BHI260IMU imu;
        Orientation angles;

        IMU.Parameters myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        new Orientation(
                                AxesReference.INTRINSIC,
                                AxesOrder.ZYX,
                                AngleUnit.DEGREES,
                                90,
                                0,
                                90,
                                0  // acquisitionTime, not used
                        )
                )
        );

        // Retrieve and initialize the IMU from the hardware map
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(myIMUparameters);

        telemetry.addData("Status", "BHI260 IMU initialized");
        telemetry.update();

        double OffsetYaw = imu.getRobotYawPitchRollAngles().getYaw();
        Boolean running = false;
        Boolean turning = false;
        Boolean negativeTurning = false;
        double angleF = 0;
        double Yaw = 0;
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            armMotor.setPower(gamepad2.right_stick_y);
            slideMotor.setPower(gamepad2.left_stick_y);
            // continuousServo.setPosition(0.0);


            if (!turning){
                robotOrientation = imu.getRobotYawPitchRollAngles();

                Yaw = (robotOrientation.getYaw(AngleUnit.DEGREES) - OffsetYaw) % 360;
                if (0 > Yaw){
                    Yaw+=360;
                }

                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double FLP = (y + x + rx) / denom;
                double BLP = (y - x + rx) / denom;
                double FRP = (y - x - rx) / denom;
                double BRP = (y + x - rx) / denom;

                FL.setPower(FLP);
                FR.setPower(FRP);
                BL.setPower(BLP);
                BR.setPower(BRP);


                telemetry.addData("Back Left Angle", BL.getDegrees());
                telemetry.addData("Front Left Angle", FL.getDegrees());
                telemetry.addData("Back Right Angle", BR.getDegrees());
                telemetry.addData("Front Right Angle", FR.getDegrees());

                telemetry.addData("Yaw", Yaw);

                telemetry.update();


                int degree = dpadButtons();
                if (degree > 0){
                    turning = true;
                    angleF = degree;
                }


                // telemetry.update();
            } else {
                for (double i = 1; i < 10; i++){
                    robotOrientation = imu.getRobotYawPitchRollAngles();
                    Yaw = (robotOrientation.getYaw(AngleUnit.DEGREES) - OffsetYaw) % 360;
                    double angleD = angleF - Yaw;
                    if(Math.abs(angleD) > 0.2) {
                        Boolean placeholder = turnTo( 1 / i, angleF, imu, OffsetYaw, motors, 0);
                    } else {
                       i = 10;
                    }

                }

                turning = false;
                /*
                turning = !turnTo(0.2, angleF, imu, OffsetYaw, motors, 0);
                turning = !turnTo(0.1, angleF, imu, OffsetYaw, motors, 0);
                */

            }
        }


    }


    public Boolean turnTo(double power, double angleF, IMU imu, Double OffsetYaw, MotorGroup motors, int run){
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();

        double Yaw = (robotOrientation.getYaw(AngleUnit.DEGREES) - OffsetYaw) % 360;
        if (0 > Yaw){
            Yaw+=360;
        }
        double angleD = angleF - Yaw;

        if (Math.abs(angleD) > 180){
            if (angleD > 0){
                angleD  = (360-Math.abs(angleD)) * -1;
            } else {
                angleD  = (360-Math.abs(angleD));
            }
        }

        /*

        if(Math.abs(angleD) < 2){
            return true;
        }
        */

        if (angleD < 0){
            power = power * -1; // right motor power first
        }

        if (angleD > 0){
            while (angleD > 0){
                for (Motor motor : motors.getMotors()) {
                    if (motor.getPosition().equals("right")) {
                        motor.setPower(power);
                    } else {
                        motor.setPower(power * -1);
                    }
                }
                robotOrientation = imu.getRobotYawPitchRollAngles();
                Yaw = (robotOrientation.getYaw(AngleUnit.DEGREES) - OffsetYaw) % 360;
                if (0 > Yaw){
                    Yaw+=360;
                }
                angleD = angleF - Yaw;

                if (Math.abs(angleD) > 180){
                    if (angleD > 0){
                        angleD  = (360-Math.abs(angleD)) * -1;
                    } else {
                        angleD  = (360-Math.abs(angleD));
                    }
                }
                telemetry.addData("angleF", angleF);
                telemetry.addData("Yaw", Yaw);
                telemetry.addData("angleD", Math.abs(angleD));
                telemetry.addData("runs", run);
                telemetry.addData("running", Math.random());

                telemetry.update();
            }
        } else {
            while (angleD < 0){
                for (Motor motor : motors.getMotors()) {
                    if (motor.getPosition().equals("right")) {
                        motor.setPower(power);
                    } else {
                        motor.setPower(power * -1);
                    }
                }
                robotOrientation = imu.getRobotYawPitchRollAngles();
                Yaw = (robotOrientation.getYaw(AngleUnit.DEGREES) - OffsetYaw) % 360;
                if (0 > Yaw){
                    Yaw+=360;
                }
                angleD = angleF - Yaw;

                if (Math.abs(angleD) > 180){
                    if (angleD > 0){
                        angleD  = (360-Math.abs(angleD)) * -1;
                    } else {
                        angleD  = (360-Math.abs(angleD));
                    }
                }
                telemetry.addData("angleF", angleF);
                telemetry.addData("Yaw", Yaw);
                telemetry.addData("angleD", Math.abs(angleD));
                telemetry.addData("runs", run);
                telemetry.addData("running", Math.random());

                telemetry.update();
            }
        }

        /*
        robotOrientation = imu.getRobotYawPitchRollAngles();
        Yaw = (robotOrientation.getYaw(AngleUnit.DEGREES) - OffsetYaw) % 360;
        angleD = angleF - Yaw;

        if (run > 10){
            return true;
        }
        if(Math.abs(angleD) > 3) {
            return turnTo(power * 0.2, angleF, imu, OffsetYaw, motors, run + 1);
        }

        telemetry.addData("angleD", angleD);
        telemetry.addData("Yaw", Yaw);
        telemetry.addData("runs", run);
        telemetry.update();

         */

        return true;



        /*
        return turnTo(power * -0.8, angleF, imu, OffsetYaw, motors, run + 1);
        */

    }

    public int dpadButtons(){
        int degree = 0;
        if (gamepad1.dpad_up) {
            degree = 360;
            if (gamepad1.dpad_left){
                degree = 45;
            } else if (gamepad1.dpad_right){
                degree = 315;
            }
        } else if (gamepad1.dpad_left) {
            degree = 90;
            if (gamepad1.dpad_down){
                degree = 135;
            }
        } else if (gamepad1.dpad_down) {
            degree = 180;
            if (gamepad1.dpad_right){
                degree = 225;
            }
        } else if (gamepad1.dpad_right) {
            degree = 270;
        }
        return degree;
    }

}