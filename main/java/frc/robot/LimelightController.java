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
    
    InputManager mDriverInputManager;
    
    NetworkTableEntry tp;
    NetworkTableEntry tx;
    NetworkTableEntry ty;

    double targetHeight = 176.5; //height off ground

    public LimelightController(RealTimeDrive realTimeDrive, InputManager driverInputManager) {
        mRealTimeDrive = realTimeDrive;
        mDriverInputManager = driverInputManager;
    }

    public boolean init() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

        tp = table.getEntry("tp");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        return true;
    }

    public boolean start() {
        boolean ret;
        exitFlag = false;

        ret = mDriverInputManager.map();
        if(!ret) return ret;

        mThread = new Thread(this, "AutonomousController");
        mThread.start();

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

    public void standby(boolean takeInputs) {
        SmartDashboard.putNumber("estimatedDistance", calcDistance(ty.getDouble(0)));
        SmartDashboard.putNumber("tY", ty.getDouble(0));
        SmartDashboard.putNumber("tX", tx.getDouble(0));
    }

    public void run() {
        System.out.println("[LimelightController] Entered independent service");
        while(!exitFlag) {
            if(mDriverInputManager.getWestButtonPressed()) {
                mRealTimeDrive.targetAngle = tx.getDouble(0);
                mRealTimeDrive.targetAngle = mRealTimeDrive.absyaw + tx.getDouble(0);
                if(mRealTimeDrive.mAutoMode == AutoMode.user) {
                    mRealTimeDrive.setAutoMode(AutoMode.rotate);
                }
                rotateFlag = true;
            }

            if(mDriverInputManager.getWestButtonReleased()) {
                mRealTimeDrive.setAutoMode(AutoMode.user);
                rotateFlag = false;
            }

            if(rotateFlag) {
                if(mRealTimeDrive.autoConditionSatisfied) {
                    mRealTimeDrive.setAutoMode(AutoMode.user);
                    rotateFlag = false;
                }
            }
        }
        System.out.println("[LimelightController] Left independent service");
    }
}
