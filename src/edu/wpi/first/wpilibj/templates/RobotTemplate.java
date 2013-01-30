package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Kinect;
import edu.wpi.first.wpilibj.KinectStick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.camera.AxisCamera;
//UNTESTED CLEANED CODE
public class RobotTemplate extends SimpleRobot implements PIDSource, PIDOutput
{


    BallMovementSystem gAE = new BallMovementSystem();
    Thread ballMovement;
    Turret turret = new Turret();
    Timer timer;
    Thread curThread;
    Thread driveThread;
    boolean flag, flag2 = true;
    Joystick pilotJoystick;
    Joystick copilotJoystick;
    KinectStick kinectL;
    KinectStick kinectR;
    Shooter shoot;
    DigitalInput autonomousA;
    DigitalInput autonomousB;
    DigitalInput frontLimit;
    DigitalInput backLimit;
    Arm arm;
    boolean gatheringFront = false;
    boolean gFlag;
    boolean gatheringBack = false;
    boolean gFlag2;
    boolean armFlag;
    boolean armRunning = false;
    boolean shootFlag;
    boolean shootRunning = false;
    double delay = 0;
    
    public PIDController pidLoop;
    private final int TICKSPERREV = 360;     //encoder ticks per revolution
    private final int DRIVESPROCKET = 15;    //drive sprocket has 15 teeth
    private final int WHEELSPROCKET = 22;    //wheel sprocket has 22 teeth
    private final double MAXJAGVOLTAGE = 12.5;    //maximum voltage when in kvoltage mode
    private final double P = 0.29;
    private final double I = 0.048;
    private final double D = 0.0;
    Gyro gyro;
    DriveThread drive;
    Thread drivethread;


    public void robotInit()
    {
        timer = new Timer();
        gyro =  new Gyro(Wiring.VERTICAL_GYRO_ANALOG_IN);
        System.out.println("UNTESTED CODE CLEANED BEFORE HARTFORD");
     

            pidLoop = new PIDController(0,0,0,this,this,.100);
            pidLoop.enable();
            pidLoop.setContinuous();

            

            pilotJoystick = new Joystick(Wiring.PilotJoystick);
            copilotJoystick = new Joystick(Wiring.CoPilotJoystick);
            kinectL = new KinectStick(1);
            kinectR = new KinectStick(2);

            shoot = new Shooter();

            frontLimit = new DigitalInput(Wiring.ARM_LIMIT_FORWARD_ID);
            backLimit = new DigitalInput(Wiring.ARM_LIMIT_BACK_ID);

            autonomousA = new DigitalInput(Wiring.AUTONOMOUS_SWITCH_A);
            autonomousB = new DigitalInput(Wiring.AUTONOMOUS_SWITCH_B);
            arm = new Arm(Wiring.ARM_PWM_ID);
            arm.Stop();

           
        

      


        curThread = Thread.currentThread();
        ballMovement = new Thread(gAE, "Ball Movement Thread");
        //driveThread = new Thread(ds);

        turret.start();
        //driveThread.start();
        //curThread.yield();
        drive = new DriveThread(this, pilotJoystick);
        drivethread = new Thread(drive);
        
    }

    public void autonomous()
    {
        //gAE.setGathering(true);
        //turret.setAutoTracking(true);
        gAE.gather.StopBack();
        gAE.gather.StopFront();
        gAE.elevator.Stop();
        shoot.Stop();

        //
        //
        //switch
        if(autonomousA.get() && autonomousB.get())
        {
            delay = 2.0;
        }
        else if(autonomousA.get() && !autonomousB.get())
        {
            delay = 5.0;
        }
        else if(autonomousB.get() && !autonomousA.get())
        {
            delay = 10.0;
        }
        //switch
        //
        //

        while(isAutonomous())
        {
            //Kinect Code Removed

           
                shoot.LaunchBall();
         

                autoDelay(delay);
                gAE.elevator.MoveUp(1);
                autoDelay(2);
                gAE.gather.FrontForward();
                gAE.gather.BackForward();

       
        }
        //gAE.setGathering(false);
        //turret.setAutoTracking(false);
    }

    public void operatorControl()
    {
        //lF.start();
        //rF.start();
        //lB.start();
        //rB.start();
        gatheringFront = false;
        gatheringBack = false;
        gFlag = false;
        gFlag2 = false;
        armFlag = false;
        armRunning = false;
        gAE.setGathering(false);

        turret.setAim(false);
        drive.start();


        while(isEnabled())
        {
           
            shoot.LaunchBall();



            //
            //
            //arm
            if(pilotJoystick.getRawButton(Wiring.RIGHT_BUMPER))
            {
             
                arm.MoveForward();
    
            }
            else if(pilotJoystick.getRawButton(Wiring.LEFT_BUMPER))
            {
        
                   arm.MoveBack();
             
            }
            else {
                arm.Stop();
            }

     

            if(!turret.doAim()){
        
                double val = copilotJoystick.getX(Hand.kLeft);
                turret.setX(val * Math.abs(val));
 
            }

            turret.setAim(copilotJoystick.getRawButton(Wiring.START_BUTTON));

     
            //
            //
            //elevator
            if(copilotJoystick.getRawButton(Wiring.A_BUTTON))
            {
    
                gAE.elevator.MoveUp(1.0);
         
            }
            else if(copilotJoystick.getRawButton(Wiring.B_BUTTON))
            {
       
               gAE.elevator.MoveUp(.35);
     
            }
            else if(copilotJoystick.getRawButton(Wiring.X_BUTTON))
            {
    
                gAE.elevator.MoveDown(.35);
     
            }
            else
            {
          
                gAE.elevator.Stop();
      
            }
            //elevator
            //
            //

  

            //
            //
            //gather
            if(pilotJoystick.getRawButton(Wiring.A_BUTTON))
            {
          
                gAE.gather.FrontForward();
    
            }
            else if(pilotJoystick.getRawButton(Wiring.B_BUTTON))
            {
       
                gAE.gather.FrontReverse();
    
            }
            else
            {
     
                gAE.gather.StopFront();
      
            }
            
   

            if(pilotJoystick.getRawButton(Wiring.X_BUTTON))
            {
      
                gAE.gather.BackForward();
           
            }
            else if(pilotJoystick.getRawButton(Wiring.Y_BUTTON))
            {
    
                gAE.gather.BackReverse();
           
            }
            else
            {
         
                gAE.gather.StopBack();
     
            }
            //gather
            //
            //

     

            if(!turret.doAim()){
                double val = copilotJoystick.getX(Hand.kLeft);
                turret.setX(val * Math.abs(val));
            }
    
        }

        //gAE.setGathering(false);
        turret.setAim(false);
    }

    public void disabled()
    {
        // stop auto-tracking
        turret.setAim(false);
    //    gAE.setGathering(false);
//        gAE.moveState = 0;
  //      gAE.shootState = 0;

    }
    
    public void test()
    {
        
    }


    public double pidGet()
    {
        //System.out.println("Hit the loop.");
        try
        {
            //CheckStatus(shoot.shoot);
            //CheckStatus(shoot.belt);
        } catch (Exception ex)
        {
            ex.printStackTrace();
        }
        return 0.0;
    }

    public void pidWrite(double output) {
    }




        // Configure a Jaguar for Speed mode
    public void cfgSpeedMode(CANJaguar jag)
    {
        try
        {
            jag.disableControl();
            jag.changeControlMode(CANJaguar.ControlMode.kSpeed);
            jag.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);
            jag.setPID(P,I,D);
            jag.configEncoderCodesPerRev((TICKSPERREV * WHEELSPROCKET) / DRIVESPROCKET);
//            jag.configMaxOutputVoltage(MAXJAGVOLTAGE);
            //jag.setVoltageRampRate(50);
            jag.enableControl();
            //jag.setX(50.0);
        }
        catch (Exception ex)
        {
            System.out.println(ex.toString());
        }
    }   //  cfgSpeedMode

    // Configure a Jaguar for Position mode
    public void cfgPosMode(CANJaguar jag)
    {
        try
        {
            jag.disableControl();
            jag.changeControlMode(CANJaguar.ControlMode.kPosition);
            jag.setPositionReference(CANJaguar.PositionReference.kQuadEncoder);
            jag.setPID(P,I,D);
            jag.configEncoderCodesPerRev((TICKSPERREV * WHEELSPROCKET) / DRIVESPROCKET);
//            jag.configMaxOutputVoltage(MAXJAGVOLTAGE);
            //jag.setVoltageRampRate(50);
            jag.setX(100.0);
            jag.enableControl();
        }
        catch (Exception ex)
        {
            System.out.println(ex.toString());
        }
    }   //  cfgSpeedMode

    // Configure a Jaguar for normal (PWM like) mode
    public void cfgNormalMode(CANJaguar jag)
    {
        try
        {
            jag.disableControl();
            jag.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
            jag.configMaxOutputVoltage(MAXJAGVOLTAGE);
            jag.enableControl();
        }
        catch (Exception ex)
        {
            System.out.println(ex.toString());
        }
    }   //  cfgNormalMode

    private final int START = 0;
    private final int APPROACH = 1;
    private final int SLOWDOWN = 2;
    private final int FWDCLIMB = 3;
    private final int FWDBACKUP = 4;
    private final int FWDLEVEL = 5;
    private final int BACKLEVEL = 6;
    private final int STOP = 7;
    private final int MANUAL = 8;


//    public void Autolevel()
//    {
//        double speed = 0.0;
//        int direction = -1;
//        double rawAngle;
//        double previousAngle = 0.0;
//        double deltaAngle = 0.0;
//        int counter = 0;
//        int state = START;
//        double maxOut = 100.0;
//        double fwdLimit, backLimit;
//
//        System.out.println("Autolevel entered.");
////        cfgSpeedMode(leftFront);
////        cfgSpeedMode(rightFront);
////        cfgSpeedMode(leftBack);
////        cfgSpeedMode(rightBack);
////        drive.setMaxOutput(maxOut);
//        timer.start();
//        timer.reset();
//        gyro.reset();
//
//        fwdLimit = 20.0;
//        backLimit = 57.5;
//
//        if(pilotJoystick.getRawButton(5))
//        {
//           state = MANUAL;
//        }
//
//
//        while(isEnabled() && (pilotJoystick.getRawButton(Wiring.LEFT_BUMPER)))
//        {
//            if (MANUAL != state)
////                drive.drive(speed * direction,0.0);
//            else
//            {
//                maxOut = 250;
//            }
//
//            timer.delay(.02);
//
//            rawAngle = gyro.getAngle();
//            if (previousAngle == 0.0)//safety
//            {
//                previousAngle = rawAngle;
//            }
//
//            deltaAngle = (rawAngle - previousAngle);
//            previousAngle = rawAngle;
//            System.out.println("Raw: " + rawAngle + " , " + deltaAngle + ", " + state + ", " + fwdLimit + ", " + backLimit);
//
//            switch (state)
//            {
//                case START:
//                    gyro.reset();
//                    speed = 1.0;
//                    counter = 0;
//                    state++;
//                    break;
//
//                case APPROACH:
//                    if (rawAngle > 13.0)
//                    {
//                        counter++;
//                        if (counter > 20)
//                        {
//                            counter = 0;
////                            maxOut -= 10 ;
//                            drive.setMaxOutput(maxOut);
//                            drive.drive(speed * direction,0.0);
//                            state++;
//                        }
//                    }
//                    break;
//
//                case SLOWDOWN:
//                    if (maxOut > 20.0)
//                    {
//                        maxOut -= 10.0 ;
//                        drive.setMaxOutput(maxOut);
//                        drive.drive(speed * direction,0.0);
//                    }
//                    else
//                    {
//                        state++;
//                    }
//                   break;
//
//                case FWDCLIMB:
//                    // this needs a lower threshold 0.17 ?
//                    // 0.25 is too far down the slope
//                    if (Math.abs(deltaAngle) > 0.17)
//                    {
//                        // over threshold
//                        counter++;
//                        if (counter == 3)
//                        {
//                            // backup
//                            drive.setMaxOutput(backLimit);
//                            direction = 1;
//                            drive.drive(speed * direction,0.0);
//                            //System.out.println("Backup");
//                            counter = 0;
//                            state++;
//                        }
//                    }
//                    else
//                    {
//                        counter = 0;
//                    }
//                    break;
//
//                case FWDBACKUP:
//                    if (Math.abs(rawAngle) < 6.0)
//                    {
//                        state = FWDLEVEL;
//                        speed = 1.0;
//                        direction = -1;
////                        cfgPosMode(left);
////                        cfgPosMode(right);
//
//                        drive.setMaxOutput(10.0);
//                        drive.drive(speed * direction,0.0);
//
//                        System.out.println("Holding");
//                    }
//                    break;
//
//                case FWDLEVEL:
//                    break;
//
//                case BACKLEVEL:
//                    break;
//                case MANUAL:
//                    drive.arcadeDrive(pilotJoystick);
//                    break;
//                case STOP:
//
//                    drive.drive(speed * direction,0.0);
//                    break;
//            }   // switch
//        }   // while
//
//        cfgNormalMode(leftFront);
//        cfgNormalMode(rightFront);
//        cfgNormalMode(rightBack);
//        cfgNormalMode(leftBack);
//        drive.setMaxOutput(1.0);
//        //System.out.println("Autolevel finished.");
//    }

    public void autoDelay(double delay) {
        for(int i = 0; i < delay * 4; i++) {
            if(isAutonomous()) {
                Timer.delay(.25);
            }
            else {
               break;
            }

        }
    }


    public void CheckStatus(CANJaguar jag)
    {
        try
        {
            if(!jag.isAlive())
            {
                //System.out.println("Jaguar: " + jag);
                //System.out.println("Faults: " + jag.getFaults());
//              System.out.println("Current: " + jag.getOutputCurrent());
                //System.out.println("Voltage: " + jag.getOutputVoltage());
                //System.out.println("Speed: " + jag.getSpeed());
                //System.out.println("Temperature: " + jag.getTemperature());
                //System.out.println("Expiration: " + jag.getExpiration());
                //System.out.println("\n----------------------------------\n");
                Thread.sleep(100);
            }
        }
        catch (Exception ex)
        {
            ex.printStackTrace();
        }
    }
}
