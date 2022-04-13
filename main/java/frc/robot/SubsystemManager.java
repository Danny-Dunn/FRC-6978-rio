package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SubsystemManager implements Runnable {
    private Thread mThread;
    private Subsystem mSubsystem;
    private boolean exitFlag;
    
    class SubsystemTimingLimits {
        long maxResponseTime = 200000000; //maximum time since the last loop start or end
        long maxLoopDuration = 200000000; //maximum time for a completed loop
    }

    private SubsystemTimingLimits mSubsystemTimingLimits;

    enum SubsystemStatus {
        inactive,
        normal,
        warning,
        fault,
    }

    private SubsystemStatus mSubsystemStatus = SubsystemStatus.inactive;

    private String mSubsystemMessage;

    //timing stats in uS
    class SubsystemTimingStatistics {
        long latestLoopTime;
        long lastStartTS;
        long lastEndTS;
        long lastResponseTS;
        boolean responseTSValid = false;
        boolean loopValid = false;
    }

    private SubsystemTimingStatistics mSubsystemTimingStatistics;

    public SubsystemManager(Subsystem subsystem) {
        mSubsystemTimingStatistics = new SubsystemTimingStatistics();
        mSubsystemTimingLimits = new SubsystemTimingLimits();
        mSubsystem = subsystem;
        mSubsystemMessage = "";
    }

    public void setTimingLimits(long maxResponseTime, long maxLoopDuration) {
        mSubsystemTimingLimits.maxLoopDuration = maxLoopDuration;
        mSubsystemTimingLimits.maxResponseTime = maxResponseTime;
    }

    public SubsystemStatus getStatus() {
        return mSubsystemStatus;
    }
    
    public String getMessage() {
        return mSubsystemMessage;
    }

    //Call this at the beginning of the subsystem loop
    public synchronized void beginLoop() {
        mSubsystemTimingStatistics.lastStartTS = System.nanoTime();
        mSubsystemTimingStatistics.lastResponseTS = mSubsystemTimingStatistics.lastStartTS;
        mSubsystemTimingStatistics.responseTSValid = true;
    }
    //Call this at the beginning of the subsystem loop
    public synchronized void endLoop() {
        mSubsystemTimingStatistics.lastEndTS = System.nanoTime();
        mSubsystemTimingStatistics.lastResponseTS = mSubsystemTimingStatistics.lastEndTS;
        mSubsystemTimingStatistics.latestLoopTime = mSubsystemTimingStatistics.lastEndTS - mSubsystemTimingStatistics.lastStartTS;
        mSubsystemTimingStatistics.responseTSValid = true;
        mSubsystemTimingStatistics.loopValid = true;
    }

    //perform a check
    public boolean checkStatus() {
        synchronized(mSubsystemTimingStatistics) {
            long now = System.nanoTime();

            if(now - mSubsystemTimingStatistics.lastResponseTS > mSubsystemTimingLimits.maxResponseTime && mSubsystemTimingStatistics.responseTSValid) {
                stop();
                mSubsystemStatus = SubsystemStatus.fault;
                mSubsystemMessage = "timeOut";
                
                return true;
            }

            if(mSubsystemTimingStatistics.latestLoopTime > mSubsystemTimingLimits.maxLoopDuration && mSubsystemTimingStatistics.loopValid) {
                stop();
                setFailure("loopTimeOverrun");
                
                return true;
            }
        }

        return false;
    }

    public void publishStatus() {
        SmartDashboard.putString(mSubsystem.getClass().getName() + ".status", mSubsystemStatus.toString());
        SmartDashboard.putString(mSubsystem.getClass().getName() + ".message", mSubsystemMessage);
    }

    public void setFailure(String message) {
        mSubsystemStatus = SubsystemStatus.fault;
        mSubsystemMessage = message;
        System.out.println("[" + mSubsystem.getClass().getName() + "]");
    }

    public void clearFailure(String message) {
        if(mSubsystemStatus != SubsystemStatus.fault && mSubsystemStatus != SubsystemStatus.warning)
            return;
        mSubsystemStatus = SubsystemStatus.fault;
        mSubsystemMessage = message;
        System.out.println("[" + mSubsystem.getClass().getName() + "]");
    }

    public boolean start() {
        if(mSubsystemStatus == SubsystemStatus.normal) return false;
        
        if(!mSubsystem.setup()) return false;
        
        exitFlag = false;
        mThread = new Thread(this, mSubsystem.getClass().getName());
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
        mSubsystemStatus = SubsystemStatus.inactive;
        synchronized(mSubsystemTimingStatistics) {
            mSubsystemTimingStatistics.loopValid = false;
            mSubsystemTimingStatistics.responseTSValid = false;
        }
        mThread = null;
        return true;
    }

    public void run() {
        mSubsystemStatus = SubsystemStatus.normal;
        while(!exitFlag) {
            beginLoop();
            mSubsystem.run();
            endLoop();
        }
    }
}
