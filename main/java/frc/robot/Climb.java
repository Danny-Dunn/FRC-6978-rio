package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climb implements Runnable{
    Joystick driveStick;
    Joystick operatorStick;

    TalonSRX CBRMotor; 
    TalonSRX CBLMotor; 

    TalonSRX CFRMotor; 
    TalonSRX CFLMotor; 


    double catchPoint = -111; //point at which the lift is considered up and should begin bias power
    boolean frontState;
    boolean backState;

    double holdingBias = 0.2;
    double pullingPower = 0.5;
    double pushingPower = 0;

    public Climb(Joystick driveStick, Joystick operatorStick) {
        this.driveStick = driveStick;
        this.operatorStick = operatorStick;
    }



    public void init() {
        CFLMotor = new TalonSRX(20);
        CFRMotor = new TalonSRX(21);
        CBLMotor = new TalonSRX(22);
        CBRMotor = new TalonSRX(23);
        CFLMotor.setSelectedSensorPosition(0);
        CFRMotor.setSelectedSensorPosition(0);
        CBLMotor.setSelectedSensorPosition(0);
        CBRMotor.setSelectedSensorPosition(0);
        System.out.println("[Climb] finished initialisation");
    }

    /*void climbPID(double goal) {
        double p = 0.0015;
        double distance = climbMotor.getSelectedSensorPosition();
        
        double error = goal - distance;
        double output = error * p;
        double maxSpeed = 0.7;
        if (output > maxSpeed) {
            output = maxSpeed;
        } else if (output <= 0) {
            output = 0;
        }

        output = -output;
        //climbMotor.set(ControlMode.PercentOutput, output);
    }*/

    public boolean exitFlag; //this flag is set true when the loop is to be exited
    public void run() {
        exitFlag = false;
        backState = false;
        frontState = false;
        System.out.println("[Climb] entered independent service");
        while(!exitFlag) {
            
            if(operatorStick.getRawButton(5)){ //front pull 
                frontState = true;
                CFLMotor.set(ControlMode.PercentOutput, pullingPower);
                CFRMotor.set(ControlMode.PercentOutput, pullingPower);
            } else if (operatorStick.getRawButton(7)) { //front release
                frontState = false;
                CFLMotor.set(ControlMode.PercentOutput, pushingPower);
                CFRMotor.set(ControlMode.PercentOutput, pushingPower);
            } else if (frontState) { //apply bias
                CFLMotor.set(ControlMode.PercentOutput, holdingBias);
                CFRMotor.set(ControlMode.PercentOutput, holdingBias);
            }
            else {
                CFLMotor.set(ControlMode.PercentOutput, 0);
                CFRMotor.set(ControlMode.PercentOutput, 0);
            }

            if(operatorStick.getRawButton(6)){ //back pull 
                backState = true;
                CBLMotor.set(ControlMode.PercentOutput, pullingPower);
                CBRMotor.set(ControlMode.PercentOutput, pullingPower);
            } else if (operatorStick.getRawButton(8)) { //back release
                backState = false;
                CBLMotor.set(ControlMode.PercentOutput, pushingPower);
                CBRMotor.set(ControlMode.PercentOutput, pushingPower);
            } else if (backState) { //apply bias
                CBLMotor.set(ControlMode.PercentOutput, holdingBias);
                CBRMotor.set(ControlMode.PercentOutput, holdingBias);
            }
            else {
                CBLMotor.set(ControlMode.PercentOutput, 0);
                CBRMotor.set(ControlMode.PercentOutput, 0);
            }
            
            try {Thread.sleep(5);} catch (InterruptedException ie) {} //deliberately only updates around 200hz
        }
        System.out.println("[Climb] left independent service");

    }
}
