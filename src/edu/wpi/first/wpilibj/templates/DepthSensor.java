package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.AnalogModule;

public class DepthSensor
{
    double CONSTANTVOLTAGE = 0.0;
    double CURRENTVOLTAGE = 0.0;
    double DEADBAND = 1.3;
    AnalogChannel ac;
    public DepthSensor(int channel)
    {
        ac = new AnalogChannel(channel);
        CONSTANTVOLTAGE = getCurrentVoltage();
    }
    public double getCurrentVoltage()
    {
        CURRENTVOLTAGE = ac.getVoltage();
        return CURRENTVOLTAGE;
    }
    public double getConstantVoltage()
    {
        return CONSTANTVOLTAGE;
    }
    public void checkBallState()
    {
        getCurrentVoltage();
        System.out.println("Adjusted Value: " + (CURRENTVOLTAGE) + " Actual Value: " + CURRENTVOLTAGE* DEADBAND);
        if(CURRENTVOLTAGE >= CONSTANTVOLTAGE* DEADBAND)
        {
            System.out.println("Ball");
        }
        else
        {
            System.out.println("No Ball");
        }
    }
    public boolean isTriggered()
    {
        return getCurrentVoltage() > getConstantVoltage() * DEADBAND;
    }
}
