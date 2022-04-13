package frc.robot.driver;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.MotorDriver;

public class TalonSRXDriver implements MotorDriver{
    private TalonSRX mTalonSRX;
    
    public TalonSRXDriver(int id) {
        mTalonSRX = new TalonSRX(id);
    }

    public void set(double demand) {
        mTalonSRX.set(ControlMode.PercentOutput, demand);
    }

    public void disable() {
        mTalonSRX.set(ControlMode.Disabled, 0);
    }

    public double getInputVoltage() {
        return mTalonSRX.getBusVoltage();
    }

    public double getOutputVoltage() {
        return mTalonSRX.getMotorOutputVoltage();
    }

    public double getOutputCurrent() {
        return mTalonSRX.getStatorCurrent();
    }

    public double getOutputPower() {
        return getOutputVoltage() / getOutputCurrent();
    }

    public void setEncoderInversion(boolean invert) {
        mTalonSRX.setSensorPhase(invert);
    }

    public void setDriverInversion(boolean invert) {
        mTalonSRX.setInverted(invert);
    }

    public double getRelativePosition() {
        return mTalonSRX.getSelectedSensorPosition();
    }

    public void setRelativePosition(double position) {
        mTalonSRX.setSelectedSensorPosition(position);
    }

    public double getVelocity() {
        return mTalonSRX.getSelectedSensorVelocity();
    }
}
