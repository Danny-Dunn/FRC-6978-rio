package frc.robot;
//Cross The Road Electronics(CTRE) libs must be installed
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; TODO: SmartDashboard
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;

//simOuts are for simulation purposes only, these can be removed in final robot code to improve efficiency

public class RealTimeDrive implements Runnable {
    //motors
    TalonFX DL1Motor;
    TalonFX DL2Motor;
    TalonFX DR1Motor;
    TalonFX DR2Motor;
    public TalonSRX shooterMotor;
    TalonSRX loaderMotor;
    
    Joystick driverStick;
    public double aimInput;
    public double shooterInput;

    boolean alignEnabled;
    public boolean alignDCOK;
    public boolean shooterEnabled;

    NetworkTable simTable;

    public RealTimeDrive(Joystick driveStick) {
        //meta stuff
        this.driverStick = driveStick;
        this.aimInput = 0;
    }
    //output functions for simulation
    public void simOut(String tag, Double value) {
        if (!RobotBase.isReal()) {
            simTable.getEntry(tag).setDouble(value);
        }
    }
    public void simOut(String tag, Boolean value) {
        if (!RobotBase.isReal()) {
            simTable.getEntry(tag).setBoolean(value);
        }
    }

    public double getShooterVelocity() {
        return shooterMotor.getSelectedSensorVelocity();
    }

    public boolean setup() {
        //setup motors
        DL1Motor = new TalonFX(0);
        DL2Motor = new TalonFX(1);
        DR1Motor = new TalonFX(2);
        DR2Motor = new TalonFX(3);
        System.out.println("TalonFX 0-3 OK");
        shooterMotor = new TalonSRX(4);
        loaderMotor = new TalonSRX(5);
        System.out.println("TalonSRX 4-5 OK");
        if(!RobotBase.isReal()) simTable = NetworkTableInstance.getDefault().getTable("simTable"); //simulation dummy outputs
        return true; //everything went fine
    }
    
    public boolean exitFlag;
    public void run() { //might remove
        exitFlag = false; //clear flag on start
        while (!exitFlag) {
            long start = System.currentTimeMillis();
            if(driverStick.getRawButton(4) && alignDCOK/*replace with housekeeping*/ ) { //drive takeover, make sure alignDC is ok
                this.alignEnabled = true;
                //following is placeholder
                DL1Motor.set(ControlMode.PercentOutput, aimInput);
                DL2Motor.set(ControlMode.PercentOutput, aimInput);
                DR1Motor.set(ControlMode.PercentOutput, aimInput);
                DR2Motor.set(ControlMode.PercentOutput, aimInput);
            } else { //run the regular drive TODO: drive calculations
                double deadzone = 0.2;
                double y = driverStick.getY() * -1;
                double x = driverStick.getX();
                //deadzone calclations
                x = (x < deadzone && x > -deadzone)? 0 : x;
                if(x != 0.0) x = (x > 0.0)? x - deadzone : x + deadzone; //eliminate jump behaviour
                simOut("xval", x);
                
                y = (y < deadzone && y > -deadzone)? 0 : y;
                if(y != 0.0) y = (y > 0.0)? y - deadzone : y + deadzone; //eliminate jump behaviour
                simOut("yval", y);

                double leftDrive = y + x;
                double rightDrive = y - x;
                
                DL1Motor.set(ControlMode.PercentOutput, leftDrive);
                DL2Motor.set(ControlMode.PercentOutput, leftDrive);
                DR1Motor.set(ControlMode.PercentOutput, rightDrive);
                DR2Motor.set(ControlMode.PercentOutput, rightDrive);
                simOut("leftDrive", leftDrive);
                simOut("rightDrive", rightDrive);
            }
            shooterEnabled = driverStick.getRawButton(1);
            simOut("shooterEnabled", shooterEnabled);
            if (shooterEnabled) {
                //set the shooter motor
                if (alignDCOK) {shooterMotor.set(ControlMode.PercentOutput, shooterInput); simOut("shooterTarget", shooterInput);} else {
                    //failsafe
                    shooterMotor.set(ControlMode.PercentOutput, 0.5);
                    simOut("shooterTarget", shooterInput);
                }
            } else {
                shooterInput = 0;
                simOut("shooterTarget", shooterInput);
                shooterMotor.set(ControlMode.PercentOutput, 0.0);
                //try {Thread.sleep(8000);} catch (InterruptedException ie) {} //stall condition
            }
            if(System.currentTimeMillis() < (start + 1)) try {Thread.sleep(1);} catch (InterruptedException ie) {} //prevents the thread from running too fast
        }
    }
}
