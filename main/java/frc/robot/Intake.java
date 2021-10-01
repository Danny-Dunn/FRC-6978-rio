package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake implements Runnable{
    Joystick driveStick; //joystick(only used for button inputs)

    TalonSRX intakeLiftMotor; //retracts ball pickup structure into robot, positive power=lift
    TalonSRX intakeMotor; //drives rollers to pick up balls, negative power=pull balls in

    double catchPoint = -111; //point at which the lift is considered up and should begin bias power
    boolean targetState = true; //determined by joystick buttons
    boolean realState; //actual state of the lift mechanism

    public Intake(Joystick driveStick) {
        this.driveStick = driveStick;
    }



    public void init() {
        intakeLiftMotor = new TalonSRX(16);
        intakeMotor = new TalonSRX(17);
        intakeLiftMotor.setSelectedSensorPosition(0);
        System.out.println("[Intake] Start OK");
    }


    public boolean exitFlag; //this flag is set true when the loop is to be exited
    public void run() {
        exitFlag = false;
        while(!exitFlag) {
            realState = (intakeLiftMotor.getSelectedSensorPosition() > catchPoint);
            //true = up
            
            if(driveStick.getRawButtonPressed(6)) targetState = true; //lift button
            else if(driveStick.getRawButtonPressed(5)) targetState = false; //drop button
            //intakeLiftMotor.set(ControlMode.PercentOutput, 0.1);
            
            if(targetState) { // go up
                if(realState) intakeLiftMotor.set(ControlMode.PercentOutput, 0.1); //carriage is up
                else intakeLiftMotor.set(ControlMode.PercentOutput, 0.7);
            } else { //go down
                if(realState) intakeLiftMotor.set(ControlMode.PercentOutput, -0.4); //carriage is up
                else if(intakeLiftMotor.getSelectedSensorPosition() < -700) intakeLiftMotor.set(ControlMode.PercentOutput, 0.1); //carriage is all stop(hold)
                else intakeLiftMotor.set(ControlMode.PercentOutput, 0.0); //carriage is downward in motion(slow down)
            }

            if(driveStick.getRawButton(4)) {
                intakeMotor.set(ControlMode.PercentOutput, -0.5);
            } else if(driveStick.getRawButton(2)) {
                intakeMotor.set(ControlMode.PercentOutput, 0.5);
            } else {
                intakeMotor.set(ControlMode.PercentOutput, 0);
            }
            
            SmartDashboard.putNumber("Lift Position", intakeLiftMotor.getSelectedSensorPosition());
            try {Thread.sleep(5);} catch (InterruptedException ie) {} //deliberately only updates around 200hz
        }
    }
}
