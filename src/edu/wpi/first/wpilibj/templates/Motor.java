package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

public class Motor extends CANJaguar implements Runnable
{
    int jagNum = 0;
    public Motor(int JaguarNumber) throws CANTimeoutException
    {
        this(JaguarNumber, ControlMode.kSpeed);
    }

    public Motor(int JaguarNumber, ControlMode mode) throws CANTimeoutException
    {
        super(JaguarNumber, mode);
        this.configMaxOutputVoltage(12.0);
        this.configEncoderCodesPerRev(360);
        jagNum = JaguarNumber;
    }

    public void run()
    {
        try
        {
            CheckStatus();

        }
        catch (Exception ex)
        {
            ex.printStackTrace();
        }
    }

    public void CheckStatus()
    {
        try
        {
            System.out.println("Jaguar: " + jagNum);
            System.out.println("Faults: " + this.getFaults() + " NUM: " + jagNum);
            System.out.println("Current: " + this.getOutputCurrent()+ " NUM: " + jagNum);
            System.out.println("Voltage: " + this.getOutputVoltage()+ " NUM: " + jagNum);
            System.out.println("Speed: " + this.getSpeed()+ " NUM: " + jagNum);
            System.out.println("Temperature: " + this.getTemperature()+ " NUM: " + jagNum);
            System.out.println("Expiration: " + this.getExpiration()+ " NUM: " + jagNum);
            System.out.println("\n----------------------------------\n");
            Thread.sleep(1000);
        }
        catch (Exception ex)
        {
            ex.printStackTrace();
        }
    }
}

