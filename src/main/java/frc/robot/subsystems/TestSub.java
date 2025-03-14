// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSub extends SubsystemBase {

  public SparkMax motor;
  private SparkClosedLoopController pidController;
  public RelativeEncoder encoder;
  
  public double kP = 0;
  public double kI = 0;
  public double kD = 0;
  public double kIz = 0;
  public double kFF = 0;
  public double kMinOutput = -1;
  public double kMaxOutput = 1;
  public double maxAcceleration = 0;
  public double maxVelocity = 0;
  public double actualPosition = 0;
  public double setpointGlobal = -45;
  public double allowedClosedLoopError = 0;
  public boolean doUpdatePID = true;

  private GenericEntry pEntry;
  private GenericEntry iEntry;
  private GenericEntry dEntry;
  private GenericEntry iZEntry;
  private GenericEntry ffEntry;
  private GenericEntry maxAccelEntry;
  private GenericEntry maxVelEntry;
  private GenericEntry closedLoopErrorEntry;
  private GenericEntry setPointEntry;

  public double maxRPM = 5700;
  public SparkMaxConfig motorConfig = new SparkMaxConfig();




  public TestSub() 
  {
  // assign entrys
    ShuffleboardTab pidPosition = Shuffleboard.getTab("Pid Position");
    pEntry = pidPosition.add("kP",0).getEntry();
    iEntry = pidPosition.add("kI",0).getEntry();
    dEntry = pidPosition.add("kD",0).getEntry();
    iZEntry = pidPosition.add("kIz",0).getEntry();
    ffEntry = pidPosition.add("kFF",0).getEntry();
    maxAccelEntry = pidPosition.add("kAccel",0).getEntry();
    maxVelEntry = pidPosition.add("kVel",0).getEntry();
    closedLoopErrorEntry = pidPosition.add("Allowed Closed Loop Error", 0).getEntry();
    setPointEntry = pidPosition.add("kSetPoint",0).getEntry();
    
  // Create motors
    motor = new SparkMax(5, SparkMax.MotorType.kBrushless);
    motorConfig.inverted(false);
    pidController = motor.getClosedLoopController();
    
   motorConfig.encoder.positionConversionFactor(5.143);
   //motorConfig.encoder.velocityConversionFactor(0.066);

    // Instantiate encoder
    encoder = motor.getEncoder();
    

    //motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf
    //(
    //  kP,
    //  kI,
    //  kD,
    //  kFF
    //);
    //motorConfig.closedLoop.iZone(kIz);
    //motorConfig.closedLoop.maxMotion.maxAcceleration(maxVelocity);
    //motorConfig.closedLoop.maxMotion.maxVelocity(maxVelocity);
  
    motorConfig.smartCurrentLimit(60, 60);
    //motor.configure(motorConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

  }

  public void updatePIDConstants() 
  {

    double p = pEntry.getDouble(kP);
    double i = iEntry.getDouble(kI);
    double d = dEntry.getDouble(kD);
    double iz = iZEntry.getDouble(kIz);
    double ff = ffEntry.getDouble(kFF);
    double maxAccel = maxAccelEntry.getDouble(maxAcceleration);
    double maxVel = maxVelEntry.getDouble(maxVelocity);
    double closedLoopError = closedLoopErrorEntry.getDouble(allowedClosedLoopError);
    double setPoint = setPointEntry.getDouble(setpointGlobal);

    if(p != kP){ 
      //motorConfig.closedLoop.p(kP);
      kP = p;
      doUpdatePID = true;
    }
          
    if (i != kI) {
      //motorConfig.closedLoop.i(kI);
      kI = i;
      doUpdatePID = true;
    }
    if (d != kD) {
      //motorConfig.closedLoop.d(kD);
      kD = d;
      doUpdatePID = true;
    }
    if (iz != kIz) {
      //motorConfig.closedLoop.iZone(kIz);
      kIz = iz;
      doUpdatePID = true;
    }
    if (ff != kFF) {
      //motorConfig.closedLoop.velocityFF(ff);
      kFF = ff;
      doUpdatePID = true;
    }
    if (maxAccel != maxAcceleration){
      //motorConfig.closedLoop.maxMotion.maxAcceleration(maxAcceleration);
      maxAcceleration = maxAccel;
      doUpdatePID = true;
    }

    if (maxVel != maxVelocity){
      //motorConfig.closedLoop.maxMotion.maxAcceleration(maxVelocity);
      maxVelocity = maxVel;
      doUpdatePID = true;
    }

    if(closedLoopError != allowedClosedLoopError){
      allowedClosedLoopError = closedLoopError;
      doUpdatePID = true;
    }

    if (setPoint != setpointGlobal){
      //  pidController.setReference(setpointGlobal, SparkMax.ControlType.kPosition);
        setpointGlobal = setPoint;
    }

    if(doUpdatePID){
      motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .pidf(p, i, d, ff)
      .iZone(iz)
      .outputRange(-1, 1);

    motorConfig.closedLoop.maxMotion
      // Set MAXMotion parameters for position control. We don't need to pass
      // a closed loop slot, as it will default to slot 0.
      .maxVelocity(maxVel)
      .maxAcceleration(maxAccel)
      .allowedClosedLoopError(closedLoopError);

      motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      doUpdatePID = false;
    }

  }
    
    public void updateMotorControl() {
        // Setpoint calculation based on joystick input
        updatePIDConstants();
        getPosition();
        // Set PID controller setpoint for velocity control
        pidController.setReference(setpointGlobal, SparkMax.ControlType.kMAXMotionPositionControl);

        // Push setpoint and current velocity to SmartDashboard
      /////  SmartDashboard.putNumber("SetPoint", setpoint);
      /////  SmartDashboard.putNumber("ProcessVariable", encoder.getPosition());
    }
    public void getPosition()
    {
      actualPosition = encoder.getPosition();
    }
    public void setSpeed()
    {
      motor.set(0.5);
    }
    public void stopSpeed()
    {
      motor.set(0.);
    }
    public void setZero()
    {
      encoder.setPosition(103.9);
    }
        
  }
