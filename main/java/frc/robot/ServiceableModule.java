package frc.robot;

public abstract interface ServiceableModule {
    public abstract boolean init();
    public abstract void standby(boolean takeConfigOptions);
}
