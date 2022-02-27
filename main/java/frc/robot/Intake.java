package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake implements Runnable, ServiceableModule{
    Joystick operatorStick; //joystick(only used for button inputs)

    TalonSRX intakeLiftMotor; //retracts ball pickup structure into robot
    TalonSRX intakeRollerMotor; //drives rollers to pick up cargo

    boolean targetState = true; //determined by joystick buttons
    double intakeTargetPosition;

    NetworkTable simTable;

    double intakeFullOutPosition = 750;
    double intakeParkPosition = -25;
    double intakeTicksPerRadian = 651.8986469;
    double intakeUpperPosition = -15;
    double intakeHorizontalBias = 0.14;

    public Intake(Joystick operatorStick) {
        this.operatorStick = operatorStick;
    }

    public void simOut(String tag, Double value) {
        if (!RobotBase.isReal()) {
            simTable.getEntry(tag).setDouble(value);
        } else {
            SmartDashboard.putNumber(tag, value);
        }
    }

    public boolean init() {
        intakeLiftMotor = new TalonSRX(30);
        intakeRollerMotor = new TalonSRX(31);
        intakeLiftMotor.setSelectedSensorPosition(intakeUpperPosition);
        if(!RobotBase.isReal()) simTable = NetworkTableInstance.getDefault().getTable("simTable"); //simulation dummy outputs

        System.out.println("[Intake] finished initialisation");
        return true;
    }

    public void standby(boolean takeConfigOptions) {
        SmartDashboard.putNumber("Intake Lift Radians", intakeLiftMotor.getSelectedSensorPosition() / intakeTicksPerRadian);
        SmartDashboard.putNumber("Intake Lift Power", intakeLiftMotor.getStatorCurrent() * intakeLiftMotor.getBusVoltage()); //calculate the total power draw in watts
        simOut("Intake Target Position", intakeTargetPosition);
    }

    long integralTS;
    double integralError;

    double intakeLiftPID(double target, double kP) {
        double error = target - intakeLiftMotor.getSelectedSensorPosition();

        double weightBias = Math.sin(intakeLiftMotor.getSelectedSensorPosition() / intakeTicksPerRadian) * intakeHorizontalBias;

        double movePower = (error * kP);
        movePower = (movePower > 0.10)? 0.10 : movePower;
        movePower = (movePower < -0.065)? -0.065 : movePower;
        SmartDashboard.putNumber("Move Power", movePower);
        SmartDashboard.putNumber("Intake Error", error);

        return weightBias + movePower;
    }

    public boolean exitFlag; //this flag is set true when the loop is to be exited
    public void run() {
        exitFlag = false;
        intakeTargetPosition = 0;
        double deadZone = 0.1;
        double intakeFineAdjustSpeed = 0.5;
        long lastCalcTS = System.nanoTime();
        boolean intakeFineAdjustEnabled = true;
        boolean rollerFineAdjustEnabled = true;
        double rollerInSpeed = 0.4;
        double rollerOutSpeed = -0.2;
        System.out.println("[Intake] entered independent service");
        while(!exitFlag) {
            double deltaT = (System.nanoTime() - lastCalcTS) / 1000000.0d;
            lastCalcTS = System.nanoTime();

            if(intakeFineAdjustEnabled) {
                double y = -operatorStick.getY();
                y = (y < deadZone && y > -deadZone)? 0 : y;
                if(y != 0.0) y = (y > 0.0)? y - deadZone : y + deadZone; //eliminate jump behaviour
                y = y / (1 - deadZone); 

                intakeTargetPosition += y * deltaT * intakeFineAdjustSpeed;
            }

            intakeTargetPosition = (intakeTargetPosition > intakeFullOutPosition)? intakeFullOutPosition : intakeTargetPosition;
            intakeTargetPosition = (intakeTargetPosition < intakeParkPosition)? intakeParkPosition : intakeTargetPosition;            

            if(operatorStick.getPOV() == 0) { //set the intake forward
                intakeTargetPosition = intakeFullOutPosition;
            } else if(operatorStick.getPOV() == 180) {
                intakeTargetPosition = intakeParkPosition;
            }
            
            simOut("Intake Power Out", -intakeLiftPID(intakeTargetPosition, 0.0016));

            double liftOut = -intakeLiftPID(intakeTargetPosition, -0.0016);
            liftOut = (liftOut > 0.19)? 0.19 : liftOut;
            liftOut = (liftOut < -0.18)? -0.18 : liftOut;

            intakeLiftMotor.set(ControlMode.PercentOutput, liftOut);

            if(rollerFineAdjustEnabled) {
                double y = operatorStick.getRawAxis(3);
                y = (y < deadZone && y > -deadZone)? 0 : y;
                if(y != 0.0) y = (y > 0.0)? y - deadZone : y + deadZone; //eliminate jump behaviour
                y = y / (1 - deadZone); 

                y = y*0.75;

                simOut("Intake Roller Out", y);
                intakeRollerMotor.set(ControlMode.PercentOutput, y);
            } else if(operatorStick.getRawButton(2)) {
                intakeRollerMotor.set(ControlMode.PercentOutput, rollerInSpeed);
            } else if(operatorStick.getRawButton(4)) {
                intakeRollerMotor.set(ControlMode.PercentOutput, rollerOutSpeed);
            }

            try {Thread.sleep(5);} catch (InterruptedException ie) {} //deliberately only updates around 200hz
        }
        System.out.println("[Intake] left independent service");
    }
}
