// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** An example command that uses an example subsystem. */
public class ExampleCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final DriveTrainSubsystem m_subsystem;
  private final ShooterSubsystem s_subsystem;
  private final ElevatorSubsystem e_subsystem;
  double time;
  double x;
  boolean autoSwitch;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(DriveTrainSubsystem msubsystem, ShooterSubsystem ssubsystem, ElevatorSubsystem esubsystem, boolean eAutoSwitch) {

    m_subsystem = msubsystem;
    s_subsystem = ssubsystem;
    e_subsystem = esubsystem;
    autoSwitch = eAutoSwitch;
    addRequirements(msubsystem);
    addRequirements(ssubsystem);
    addRequirements(esubsystem);

  }
int z;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_subsystem.zeroEncoder();
    s_subsystem.autoBlinkin();
    z = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (autoSwitch == false) {        //upper hub
    s_subsystem.autoUpperShoot();
      if(m_subsystem.autoReverse() == false) {
        if( z == 0){
          x=Timer.getMatchTime();
          z=1; }
          if(Timer.getMatchTime() <= 10) {
            e_subsystem.elevatorUp();
          }
    }
    if (autoSwitch == true) {         //low hub
      s_subsystem.autoLowerShoot();
      if(Timer.getMatchTime() <=10){
        e_subsystem.elevatorUp();
      }
      if(Timer.getMatchTime()<=5){
        m_subsystem.autoReverse();
      }
    }

    }

    SmartDashboard.putBoolean("Lower Hub Shooting = ", autoSwitch);

/*
    if (autoSwitch == false) {
      s_subsystem.autoUpperShoot();
    } else {
      s_subsystem.autoLowerShoot();
    }
    
    if (Timer.getMatchTime() <= 13) {
      e_subsystem.elevatorUp();
    }

    if (Timer.getMatchTime() <= 10) {
      s_subsystem.shootStop();
      e_subsystem.elevatorStop();
    }

    if (Timer.getMatchTime() <= 2) {
      m_subsystem.autoReverse();
    }

    /*
    if ( x == 0 ) {
      m_subsystem.autoForward();
      s_subsystem.autoShoot();
      System.out.println("a");
    }

    if (m_subsystem.autoForward() == true) {
      e_subsystem.elevatorUp();
      x = x + 1;
      System.out.println("b,1");
      System.out.println(x);
      SmartDashboard.putNumber("X equals", x);
    }

    if ( x == 1 ) {
      time = Timer.getMatchTime();
      System.out.println("c,2");
      SmartDashboard.putNumber("time equals", time);
    }

    if ( x >= 1) {
      if (Timer.getMatchTime() + Constants.ELEVATOR_TIME <= time ) {
        // e_subsystem.elevatorStop();
        m_subsystem.autoReverse();
        System.out.println("d,3");
      }
    }
    SmartDashboard.putNumber("Match time equals", Timer.getMatchTime());
    System.out.println(x);
    */

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    s_subsystem.shootStop();
    e_subsystem.elevatorStop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
    
  }
}
