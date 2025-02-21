// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() 
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
    SmartDashboard.putNumber("P Gain", m_robotContainer.m_test.kP);
    SmartDashboard.putNumber("I Gain", m_robotContainer.m_test.kI);
    SmartDashboard.putNumber("D Gain", m_robotContainer.m_test.kD);
    SmartDashboard.putNumber("I Zone", m_robotContainer.m_test.kIz);
    SmartDashboard.putNumber("Feed Forward", m_robotContainer.m_test.kFF);
    SmartDashboard.putNumber("Max accelatation", m_robotContainer.m_test.maxAcceleration);
    SmartDashboard.putNumber("Max Velocity", m_robotContainer.m_test.maxVelocity);
    SmartDashboard.putNumber("Actual Postion", m_robotContainer.m_test.actualPosition);
    SmartDashboard.putNumber("Set Point", m_robotContainer.m_test.setpointGlobal);
    SmartDashboard.putNumber("Encoder value", m_robotContainer.m_test.encoder.getPosition());

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() 
  {
   
      // Read data from SmartDashboard
      double p = SmartDashboard.getNumber("P Gain", m_robotContainer.m_test.kP);
      double i = SmartDashboard.getNumber("I Gain", m_robotContainer.m_test.kI);
      double d = SmartDashboard.getNumber("D Gain", m_robotContainer.m_test.kD);
      double iz = SmartDashboard.getNumber("I Zone", m_robotContainer.m_test.kIz);
      double ff = SmartDashboard.getNumber("Feed Forward", m_robotContainer.m_test.kFF);
      double minOut = SmartDashboard.getNumber("Min Output", m_robotContainer.m_test.kMinOutput);
      double maxOut = SmartDashboard.getNumber("Max Output", m_robotContainer.m_test.kMaxOutput);

      // Update PIDController datapoints with the latest from SmartDashboard
    /*   if (p != kP) {
        motorConfig.closedLoop.p(p);
          kP = p;
      }
      */

      if(SmartDashboard.getNumber("P Gain", m_robotContainer.m_test.kP) != m_robotContainer.m_test.kP)
      { m_robotContainer.m_test.motorConfig.closedLoop.p(SmartDashboard.getNumber("P Gain", m_robotContainer.m_test.kP));}
      
      if (i != m_robotContainer.m_test.kI) {
        m_robotContainer.m_test.motorConfig.closedLoop.i(i);
        m_robotContainer.m_test.kI = i;
      }
      if (d != m_robotContainer.m_test.kD) {
        m_robotContainer.m_test.motorConfig.closedLoop.d(d);
        m_robotContainer.m_test.kD = d;
      }
      if (iz != m_robotContainer.m_test.kIz) {
        m_robotContainer.m_test. motorConfig.closedLoop.iZone(iz);
        m_robotContainer.m_test.kIz = iz;
      }
      if (ff != m_robotContainer.m_test.kFF) {
        m_robotContainer.m_test.motorConfig.closedLoop.velocityFF(ff);
        m_robotContainer.m_test.kFF = ff;
      }
      if (minOut != m_robotContainer.m_test.kMinOutput || maxOut != m_robotContainer.m_test.kMaxOutput) {
        m_robotContainer.m_test.motorConfig.closedLoop.outputRange(minOut, maxOut);
        m_robotContainer.m_test.kMinOutput = minOut;
        m_robotContainer.m_test.kMaxOutput = maxOut;
      }
  }

  

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
