// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;


public class DriveTrainSubsystem extends SubsystemBase  {
  private final CANSparkMax m_rightmotor = new CANSparkMax(DriveConstants.kRightMotor1Port,MotorType.kBrushless);
  private final CANSparkMax m_rightmotor1 = new CANSparkMax(DriveConstants.kRightMotor2Port,MotorType.kBrushless);
  private final MotorControllerGroup m_rightMotors =
  new MotorControllerGroup(
      m_rightmotor,
      m_rightmotor1);

  private final CANSparkMax m_leftmotor =  new CANSparkMax(DriveConstants.kLeftMotor1Port,MotorType.kBrushless);
  private final CANSparkMax m_leftmotor1 = new CANSparkMax(DriveConstants.kLeftMotor1Port,MotorType.kBrushless);
  private final MotorControllerGroup m_leftMotors = 
          new MotorControllerGroup(
              m_leftmotor, m_leftmotor1);
  // private static final int kEncoderAChannel = 0;   
  // private static final int kEconderBChannel = 1;
  private Encoder encoder = new Encoder(0,1,true, EncodingType.k4X);
  // private final double kDriveTick2Feet = 1.0/ 128 *6*Math.PI/12;
  // private Joystick joy1 = new Joystick(0);  
  SparkMaxPIDController right = m_rightmotor.getPIDController();
  SparkMaxPIDController left = m_leftmotor.getPIDController();
  // double kP = DriveConstants.kP;
  // double kI = DriveConstants.kI;
  // double kD = DriveConstants.kD;
  // double iLimit = DriveConstants.iLimit; 
  // double setpoint = 0;
  // double errorSum = 0;
  // double lastTimestamp  = 0;
  // double lastError = 0;
  SendableChooser<String> m_chooser = new SendableChooser<>();
  
  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  public DriveTrainSubsystem()
  {
    m_chooser.setDefaultOption("Tank Drive", "Tank Drive");
    m_chooser.addOption("Arcade Drive", "Arcade Drive");
    m_chooser.addOption("Curvature Drive", "Curvature Drive");
    // right.setP(kP);
    // left.setP(kP);
    // right.setI(kI);
    // left.setI(kI);
    // right.setD(kD);
    // left.setD(kD);
    //  left.setIZone(iLimit);
    //  right.setIZone(iLimit);
    //  left.setOutputRange(-1, 1);

  }
  public void tankDrive(double leftSpeed, double rightSpeed)
  {
    m_drive.tankDrive(leftSpeed, rightSpeed, true);
  }

  public void arcadeDrive(double speed, double rot)
  {
    m_drive.arcadeDrive(speed, rot, true);
  }

  public void curvatureDrive(double speed, double rot)
  {
    m_drive.curvatureDrive(speed, rot, false);
  }

  // public void PIDDrive(double distance)
  // {
  //   left.setReference(distance, ControlType.kPosition);
  //   right.setReference(distance, ControlType.kPosition);
  // }
  
  public void stop()
  {
    m_leftMotors.set(0);
    m_rightMotors.set(0);
  }

  public double getLeftSpeed()
  {
    return m_leftMotors.get();
  }

  public double getRightSpeed()
  {
    return m_rightMotors.get();
  }

  


  public void setDrive(double lx, double ly, double ry)
  {
    switch (m_chooser.getSelected())
    {
      case "Tank Drive": 
        tankDrive(ly, ry);
        break;
      case "Arcade Drive": 
        arcadeDrive(ly, lx);
        break;
      case "Curvature Drive": 
        curvatureDrive(ly, lx);
        break;
    }
  }

  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public void autonomousPeriodic() {

  }
  public void autonomousInit(){
     
  }
}
