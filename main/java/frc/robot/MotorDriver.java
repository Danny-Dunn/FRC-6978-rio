package frc.robot;

public interface MotorDriver {
    public void set(double demand);

    public double getInputVoltage();
    
    public double getOutputVoltage();

    public double getOutputCurrent();

    public double getOutputPower();

    public void setEncoderInversion(boolean invert);

    public void setDriverInversion(boolean invert);

    public double getRelativePosition();

    public void setRelativePosition(double position);

    public double getVelocity();

    public void disable();
}
