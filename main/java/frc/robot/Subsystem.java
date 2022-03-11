package frc.robot;

public abstract class Subsystem {
    //call when the subsystem should grab any hardware interfaces and do start-up calibrations(called once/session?)
    public abstract boolean init();
    //call periodically to send telemetry and receive config from DS
    public abstract void standby(boolean takeConfigOptions);
    //call once to reset before the subsystem starts(called multiple times/session)
    public abstract boolean setup();
    //call continuously when running the subsystem
    public abstract void run();

    public SubsystemManager manager;

    public Subsystem() {
        manager = new SubsystemManager(this);
    }
}
