package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Victor;


public class Arm
{
    Victor arm;
    boolean goingDown;
    boolean stopped;
    long startTime = -1;

    public Arm(int victor)
    {
        arm = new Victor(victor);
    }

    public void MoveForward()
    {
        goingDown = true;
        stopped = false;
        arm.set(1.0);
    }

    public void MoveBack()
    {
        goingDown = false;
        stopped = false;
        arm.set(-1.0);
    }
    
    public void MoveForward(double input)
    {
        goingDown = true;
        stopped = false;
        input = Math.abs(input);
        arm.set(input);
    }

    public void MoveBack(double input)
    {
        goingDown = false;
        stopped = false;
        arm.set(-(input));
    }

    public void Stop()
    {
        stopped = true;
        arm.set(0);
    }

    /*public void pushBridge()
    {
        if(frontLimit.get())
        {
            arm.set(0);
            return;
        }

        if(startTime > 0)
        {
            if(System.currentTimeMillis() > startTime + 1000)
            {
                arm.set(1);


            }
            else
            {
                arm.set(0.3);
            }
        }
        else
        {
            startTime = System.currentTimeMillis();
            arm.set(0.3);
        }


    }
    public void liftUp()
    {
        startTime = -1;
        if(!backLimit.get())
        {
            arm.set(-.3);
        }
        else
        {
            arm.set(0);
        }
    }*/
}
