package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.List;

public class MotorGroup {
    private List<Motor> motors = new ArrayList<Motor>();
    private boolean process = false;

    public MotorGroup(ArrayList<Motor> motors){
        this.motors = motors;
    }
    public void reset() {
        for (Motor motor : this.motors){
            motor.reset();
        }
    }
    public void ZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        for (Motor motor : this.motors){
            motor.ZeroPowerBehavior(behavior);
        }
    }
    public void setDirection(DcMotorSimple.Direction direction){
        for (Motor motor : this.motors){
            motor.setDirection(direction);
        }
    }
    public List<Motor> getMotors(){
        return motors;
    }
    public void setProcess(boolean bool){
        this.process = bool;
    }
    public boolean getProcess(){
        return this.process;
    }




}
