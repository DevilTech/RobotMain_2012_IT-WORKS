package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Relay;

public class Gatherer
{
    Relay gatherFront;
    Relay gatherBack;

    public Gatherer(int front, int back)
    {
        gatherFront = new Relay(front);
        gatherBack = new Relay(back);
    }

    public void FrontForward()
    {
        gatherFront.set(Relay.Value.kForward);
    }

    public void FrontReverse()
    {
        gatherFront.set(Relay.Value.kReverse);
    }

    public void BackForward()
    {
        gatherBack.set(Relay.Value.kForward);
    }

    public void BackReverse()
    {
        gatherBack.set(Relay.Value.kReverse);
    }
    public void StopBack()
    {
        gatherBack.set(Relay.Value.kOff);
    }
    public void StopFront()
    {
        gatherFront.set(Relay.Value.kOff);
    }
}
