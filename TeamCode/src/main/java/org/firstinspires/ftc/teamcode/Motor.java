package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motor {
    private DcMotor motor;
    private double PPR;
    private String position;

    public Motor(HardwareMap hardwareMap, String motorName, double PPR, String position) {
        this.position = position;
        motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.PPR = PPR;
    }
    public void setPower(double power) {
        motor.setPower(power);
    }
    public void reset(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void ZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        motor.setZeroPowerBehavior(behavior);
    }
    public void setDirection(DcMotorSimple.Direction direction){
        motor.setDirection(direction);
    }
    public double getDegrees() {
        return (motor.getCurrentPosition()%PPR)/PPR * 360;
    }
    public double getRadians() {
        return (motor.getCurrentPosition()%PPR)/PPR * 2 * 3.14159;
    }
    public String getPosition(){
        return this.position;
    }


}
