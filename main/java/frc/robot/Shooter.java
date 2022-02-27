package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter implements Runnable, ServiceableModule {
    Joystick driveStick; //joystick(only used for button inputs)
    int shootButton;

    TalonSRX shooterMotor; 

    double shooterPower;

    enum ShooterControlMode {
        velocity,
        direct,
        none
    };

    ShooterControlMode mShooterControlMode;

    public void setShooterControlMode(ShooterControlMode mode) {
        switch (mode) {
            case velocity:
                integralError = 0;
                integralTS = System.nanoTime();
                break;
            default:
                break;
        }
    }

    public Shooter(Joystick driveStick) {
        this.driveStick = driveStick;
        SmartDashboard.putNumber("Shooter Power", 0.6);
    }

    public boolean init() {
        shooterMotor = new TalonSRX(10);
        shooterMotor.setSelectedSensorPosition(0);
        shooterMotor.setSensorPhase(true);
        shooterMotor.setInverted(false);

        mShooterControlMode = ShooterControlMode.none;

        System.out.println("[Shooter] finished initialisation");
        return true;
    }

    long integralTS;
    double integralError;
    double shooterPID(double target, double kP, double kI) {
        double error = target - shooterMotor.getSelectedSensorVelocity();

        integralError += ((System.nanoTime() - integralTS) * error) / 1000000000;
        integralTS = System.nanoTime();

        return (error * kP) + (integralError * kI);
    }

    public void standby(boolean takeConfigOptions) {
        SmartDashboard.putNumber("shooterSpeed", shooterMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Shooter Current", shooterMotor.getStatorCurrent());

        if(takeConfigOptions) {
            shooterPower = SmartDashboard.getNumber("Shooter Power", 0.6);
        }
    }
    
    public boolean exitFlag; //this flag is set true when the loop is to be exited
    public void run() {
        exitFlag = false;
        System.out.println("[Shooter] entered independent service");
        while(!exitFlag) {
            if(driveStick.getRawButtonPressed(shootButton)) {
                setShooterControlMode(ShooterControlMode.velocity);
            } else if(driveStick.getRawButtonReleased(shootButton)) {
                setShooterControlMode(ShooterControlMode.none);
            }

            switch (mShooterControlMode) {
                case velocity:
                    shooterMotor.set(ControlMode.PercentOutput, -shooterPID(19000, 0.000111, 0.0000317));
                    break;
                case direct:
                    shooterMotor.set(ControlMode.PercentOutput, shooterPower);
                default:
                    shooterMotor.set(ControlMode.PercentOutput, 0.0);
                    break;
            }

            try {Thread.sleep(5);} catch (InterruptedException ie) {} //deliberately only updates around 200hz
        }
        System.out.println("[Shooter] left independent service");
    }
}
