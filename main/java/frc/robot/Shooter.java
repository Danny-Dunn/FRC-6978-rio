package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter implements Runnable, ServiceableModule {
    Joystick driveStick; //joystick(only used for button inputs)

    TalonSRX shooterMotor; 

    double shooterPower;

    public Shooter(Joystick driveStick) {
        this.driveStick = driveStick;
        SmartDashboard.putNumber("Shooter Power", 0.6);
    }



    public boolean init() {
        shooterMotor = new TalonSRX(10);
        shooterMotor.setSelectedSensorPosition(0);
        shooterMotor.setSensorPhase(true);
        shooterMotor.setInverted(true);
        System.out.println("[Shooter] finished initialisation");
        return true;
    }

    public void standby(boolean takeConfigOptions) {
        SmartDashboard.putNumber("Shooter Speed", shooterMotor.getSelectedSensorVelocity());

        if(takeConfigOptions) {
            shooterPower = SmartDashboard.getNumber("Shooter Power", 0.6);
        }
    }

    public boolean exitFlag; //this flag is set true when the loop is to be exited
    public void run() {
        exitFlag = false;
        System.out.println("[Shooter] entered independent service");
        while(!exitFlag) {
            
            if(driveStick.getRawButton(6)){ 
                shooterMotor.set(ControlMode.PercentOutput, shooterPower);
            } //lift button
            else {
                shooterMotor.set(ControlMode.PercentOutput, 0.0);

            }
            
            try {Thread.sleep(5);} catch (InterruptedException ie) {} //deliberately only updates around 200hz
        }
        System.out.println("[Shooter] left independent service");
    }
}
