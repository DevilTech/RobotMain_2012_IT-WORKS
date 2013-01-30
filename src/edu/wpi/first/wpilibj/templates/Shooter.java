package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Jaguar;

public class Shooter
{
    //CANJaguar shoot;
    //CANJaguar belt;
    Jaguar shoot;


    public Shooter()
    {
            /*
            shoot = new CANJaguar(shootJag, CANJaguar.ControlMode.kVoltage);
            shoot.configMaxOutputVoltage(12);
            belt = new CANJaguar(beltJag, CANJaguar.ControlMode.kVoltage);
            belt.configMaxOutputVoltage(12);
            */
            shoot = new Jaguar(2);
    }

    public double CalculateLaunch(double distance)
    {
        double launchVelo = 0.0;

        return launchVelo;
    }

    public void LaunchBall()
    {
            shoot.set(.35);//fly
    }

    public void LaunchBall(double launchVelo)
    {
            shoot.set(launchVelo);
    }
    public void Stop()
    {
            shoot.set(0);
    }
}
