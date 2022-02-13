package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter implements Runnable{
    Joystick driveStick; //joystick(only used for button inputs)

    TalonSRX shooterMotor; 

    double catchPoint = -111; //point at which the lift is considered up and should begin bias power
    boolean targetState = true; //determined by joystick buttons
    boolean realState; //actual state of the lift mechanism

    public Shooter(Joystick driveStick) {
        this.driveStick = driveStick;
        SmartDashboard.putNumber("Shooter Power", -0.6);
    }



    public void init() {
        shooterMotor = new TalonSRX(10);
        shooterMotor.setSelectedSensorPosition(0);
        System.out.println("[Shooter] finished initialisation");
    }

    public boolean exitFlag; //this flag is set true when the loop is to be exited
    public void run() {
        exitFlag = false;
        System.out.println("[Shooter] entered independent service");
        while(!exitFlag) {
            
            if(driveStick.getRawButton(6)){ 
                shooterMotor.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Shooter Power", -0.6));
            } //lift button
            else {
                shooterMotor.set(ControlMode.PercentOutput, 0.0);

            }
            SmartDashboard.putNumber("Shooter Speed", shooterMotor.getSelectedSensorVelocity());
            try {Thread.sleep(5);} catch (InterruptedException ie) {} //deliberately only updates around 200hz
        }
        System.out.println("[Shooter] left independent service");
    }
}
