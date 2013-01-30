package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Victor;

public class Elevator
{
    Victor elevator;

    public Elevator(int vader)
    {
        elevator = new Victor(vader);
    }
    public void MoveUp()
    {
        elevator.set(1.0);
    }
    public void MoveDown()
    {
        elevator.set(-.5);
    }
    public void MoveUp(double howMuch)
    {
        elevator.set(howMuch);
    }
    public void MoveDown(double howMuch)
    {
        elevator.set(-(howMuch));
    }
    public void Stop()
    {
        elevator.set(0);
    }
}
