package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RealTimeDrive.AutoMode;

public class LimelightController implements ServiceableModule, Runnable {
    private Thread mThread;
    private RealTimeDrive mRealTimeDrive;
    private boolean exitFlag;
    private boolean rotateFlag;
    private boolean shouldRotateFlag;
    private boolean shooterReady;
    
    InputManager mDriverInputManager;
    
    NetworkTableEntry tp;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ledMode;

    double targetHeight = 192; //height off ground

    public LimelightController(RealTimeDrive realTimeDrive, InputManager driverInputManager) {
        mRealTimeDrive = realTimeDrive;
        mDriverInputManager = driverInputManager;
    }

    public boolean init() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

        tp = table.getEntry("tv");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ledMode = table.getEntry("ledMode");

        ledMode.setDouble(1); //turn off LEDs
        return true;
    }

    public boolean start() {
        boolean ret;
        exitFlag = false;

        ret = mDriverInputManager.map();
        if(!ret) return ret;

        mThread = new Thread(this, "AutonomousController");
        mThread.start();

        shooterReady = false;

        return true;
    }

    public boolean stop() {
        if(mThread == null) return true;
        exitFlag = true;
        try {Thread.sleep(20);} catch (Exception e) {}

        if(mThread.isAlive()) {
            mThread.interrupt();
            try {Thread.sleep(20);} catch (Exception e) {}
            if(mThread.isAlive()) {
                return false;
            }
        }

        mThread = null;

        return true;
    }
    
    double calcDistance(double camY) {
        double angle = camY + 39;
        //angle = angle;
        //circle math
		double radians = Math.toRadians(angle);
		double tpy = Math.sin(radians);
		double tpx = Math.cos(radians);
		//calculate slope ratio
		double slope = (tpx / tpy); 
        //x = (y - height) / sl
        //NOTE: Target height is offset above robot
		double x_intersect = (0 - targetHeight) / slope;
		if (x_intersect < 6.0) {
			//x_intersect = 6.0; //? cant remember why, might remove
		}
		return x_intersect;
    }

    double cameraAngleToDistance(double camY) {
        double angle = camY + 39;
        return targetHeight / Math.tan(Math.toRadians(angle));
    }

    public double getDistance() {
        return cameraAngleToDistance(ty.getDouble(0));
    }

    public double getDistanceFinal() {
        double y = ty.getDouble(0);
        ledMode.setDouble(1); //turn off LEDs
        shooterReady = false;
        return cameraAngleToDistance(y);
    }

    public boolean getShooterReady() {
        return shooterReady;
    }

    public void standby(boolean takeInputs) {
        SmartDashboard.putNumber("estimatedDistance", cameraAngleToDistance(ty.getDouble(0)));
        SmartDashboard.putNumber("tY", ty.getDouble(0));
        SmartDashboard.putNumber("tX", tx.getDouble(0));
        SmartDashboard.putBoolean("tV", tp.getDouble(0) == 1);
        SmartDashboard.putBoolean("shooterReady", shooterReady);
        SmartDashboard.putBoolean("shouldRotate", shouldRotateFlag);
    }

    public void run() {
        System.out.println("[LimelightController] Entered independent service");
        while(!exitFlag) {
            if(mDriverInputManager.getWestButtonPressed()) {
                ledMode.setDouble(0);
                shouldRotateFlag = true;
                shooterReady = false;
            }

            if(mDriverInputManager.getWestButtonReleased()) {
                mRealTimeDrive.setAutoMode(AutoMode.user);
                rotateFlag = false;
                shouldRotateFlag = false;
            }

            if(shouldRotateFlag && (tp.getDouble(0) == 1)) {
                mRealTimeDrive.targetAngle = tx.getDouble(0);
                mRealTimeDrive.targetAngle = mRealTimeDrive.absyaw + tx.getDouble(0);
                if(mRealTimeDrive.mAutoMode == AutoMode.user) {
                    mRealTimeDrive.setAutoMode(AutoMode.rotate);
                }
                rotateFlag = true;
                shouldRotateFlag = false;
                shooterReady = true;
            }

            if(rotateFlag) {
                if(mRealTimeDrive.autoConditionSatisfied) {
                    mRealTimeDrive.setAutoMode(AutoMode.user);
                    rotateFlag = false;
                }
            }
        }
        System.out.println("[LimelightController] Left independent service");
        try {Thread.sleep(10);} catch (InterruptedException ie) {} //deliberately only updates around 200hz
    }
}
