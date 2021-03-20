package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Joystick;

public class Intake {
    Joystick driveStick; //joystick(only used for button inputs)

    TalonSRX intakeLiftMotor; //retracts ball pickup structure into robot, positive power=lift
    TalonSRX intakeMotor; //drives rollers to pick up balls, negative power=pull balls in

    double catchPoint; //point at which the lift is considered up and should begin bias power
    boolean targetState; //determined by joystick buttons
    boolean realState; //actual state of the lift mechanism

    public Intake(Joystick driveStick) {
        this.driveStick = driveStick;
    }



    public void init() {
        intakeLiftMotor = new TalonSRX(16);
        intakeMotor = new TalonSRX(17);
    }


    public boolean exitFlag; //this flag is set true when the loop is to be exited
    public void run() {
        exitFlag = false;
        while(!exitFlag) {
            realState = (intakeLiftMotor.getSelectedSensorPosition() > catchPoint);
            //true = up
            
            if(driveStick.getRawButtonPressed(6)) targetState = true; //lift button
            else if(driveStick.getRawButtonPressed(5)) targetState = false; //drop button

            if(targetState) {
                if(realState) intakeLiftMotor.set(ControlMode.PercentOutput, -0.1);
                else intakeLiftMotor.set(ControlMode.PercentOutput, -0.5);
            } else {
                if(realState) intakeLiftMotor.set(ControlMode.PercentOutput, 0.5);
                else intakeLiftMotor.set(ControlMode.PercentOutput, -0.1);
            }

            if(driveStick.getRawButton(4)) {
                intakeMotor.set(ControlMode.PercentOutput, -0.5);
            } else if(driveStick.getRawButton(2)) {
                intakeMotor.set(ControlMode.PercentOutput, 0.5);
            } else {
                intakeMotor.set(ControlMode.PercentOutput, 0);
            }

            try {Thread.sleep(5);} catch (InterruptedException ie) {} //deliberately only updates around 200hz
        }
    }
}
