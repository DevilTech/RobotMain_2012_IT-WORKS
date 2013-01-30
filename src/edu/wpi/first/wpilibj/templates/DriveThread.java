
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;


public class DriveThread extends Thread{
    public CANJaguar leftFront;
    public CANJaguar rightFront;
    public CANJaguar leftBack;
    public CANJaguar rightBack;
    RobotDrive drive;
    Joystick joystick;
    RobotTemplate RobotTemplate;
    public DriveThread(RobotTemplate robo, Joystick joy){        
        try{
            leftFront = new CANJaguar(Wiring.DRIVE_LEFTFRONT_JAGUAR_ID);
            rightFront = new CANJaguar(Wiring.DRIVE_RIGHTFRONT_JAGUAR_ID);
            leftBack = new CANJaguar(Wiring.DRIVE_LEFTREAR_JAGUAR_ID);
            rightBack = new CANJaguar(Wiring.DRIVE_RIGHTREAR_JAGUAR_ID);
            drive = new RobotDrive(leftFront, leftBack, rightFront, rightBack);
            drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
            drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
            drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
            drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
            drive.setMaxOutput(.8);
            joystick = joy;
            RobotTemplate = robo;
        }catch(Exception e){
            System.out.println("Jaguars did not init");
        }
    }
    public void run(){
         while(RobotTemplate.isOperatorControl()){
             drive.arcadeDrive(joystick);
         }
    }
}
