
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Joystick;

public class BallMovementSystem implements Runnable
{
    boolean gathering = false;
    DepthSensor frontGather;
    DepthSensor backGather;
    DepthSensor[] elevatorSensors = new DepthSensor[3];
    Gatherer gather;
    int moveState = 0;
    int shootState = 0;
    Elevator elevator;
    int maxSensor = -1;
    double DEPTH_TOLERANCE = 1.3;


    public BallMovementSystem()
    {
        frontGather = new DepthSensor(Wiring.GATHER_FRONT_ANALOG_IN);
        backGather = new DepthSensor(Wiring.GATHER_BACK_ANALOG_IN);
        elevatorSensors[0] = new DepthSensor(Wiring.ELEVATOR_BOT_ANALOG_IN);
        elevatorSensors[1] = new DepthSensor(Wiring.ELEVATOR_MID_ANALOG_IN);
        elevatorSensors[2] = new DepthSensor(Wiring.ELEVATOR_TOP_ANALOG_IN);
        gather = new Gatherer(Wiring.GATHER_FRONT_SPIKE_ID, Wiring.GATHER_BACK_SPIKE_ID);
        elevator = new Elevator(Wiring.ELEVATOR_PWM_CHANNEL);
    }

    public void run()
    {
        while(true)
        {

            if(doGathering() == false)
            {
                continue;
            }
            MoveState();
        }
    }

    public synchronized boolean doGathering()
    {
        return gathering;
    }

    public synchronized void setGathering(boolean gather)
    {
        this.gathering = gather;
    }

    public void printSensorStates()
    {
        gather.StopFront();
        gather.StopBack();
        elevator.Stop();
        if(frontGather.isTriggered())
        {
            System.out.println("Front Gatherer");
        }
        if(backGather.isTriggered())
        {
            System.out.println("Back Gatherer");
        }

        for(int i = 0; i < elevatorSensors.length; i++)
        {

            if(elevatorSensors[i].isTriggered())
            {
                System.out.println("Elevator: " + (i + 1));
            }
            else if (!frontGather.isTriggered() && !backGather.isTriggered() && !elevatorSensors[i].isTriggered())
            {
                System.out.println("Nuffin");
            }
        }

    }
    public void startUp()
    {
        gather.FrontForward();
        gather.BackForward();
        elevator.MoveUp();
    }
    public void turnOff()
    {
        gather.StopFront();
        gather.StopBack();
        elevator.Stop();
    }

    public void MoveState()
    {
        switch(moveState)
        {
            default : System.out.println("default state.. WTF???");
            gather.StopFront();
            gather.StopBack();
            elevator.Stop();
            break;
            case 0: System.out.println("Start Your Engine");
            gather.FrontForward();
            gather.BackForward();
            moveState = 1;
            break;
            case 1: System.out.println("Load Ball 1");
            if(frontGather.isTriggered() || backGather.isTriggered())
            {
                moveState = 2;
            }
            break;
            case 2: System.out.println("Move 1 to point 1");
            elevator.MoveUp(.35);
            if(elevatorSensors[0].isTriggered())
            {
                elevator.Stop();
                moveState = 12;
            }
            break;
            case 12: System.out.println("State 12");
            if(!frontGather.isTriggered() && !backGather.isTriggered())
            {
                moveState = 3;
            }
            break;
            case 3: System.out.println("Load Ball 2, adjust to point 2");
            if(frontGather.isTriggered() || backGather.isTriggered())
            {
                elevator.MoveUp(.35);
                moveState = 4;
            }
            break;
            case 4: System.out.println("State 4");
            if(elevatorSensors[1].isTriggered())
            {
                elevator.Stop();

                moveState = 14;
            }
            break;
            case 14: System.out.println("State 14");
            if(!frontGather.isTriggered() && !backGather.isTriggered())
            {
                moveState = 5;
            }
            break;
            case 5: System.out.println("State 5");
            if(frontGather.isTriggered() || backGather.isTriggered())
            {
                elevator.MoveUp(.4);
                moveState = 15;
            }
            break;
            case 15: System.out.println("State 15");
            if(!frontGather.isTriggered() && !backGather.isTriggered())
            {
                moveState = 6;
            }
            break;
            case 6: System.out.println("State 6");
            if(elevatorSensors[2].isTriggered())
            {
                elevator.Stop();
                moveState = 7;
            }
            break;
            case 7: System.out.println("END STATE");
            gather.StopFront();
            gather.StopBack();
            elevator.Stop();
            break;
        }
    }
}