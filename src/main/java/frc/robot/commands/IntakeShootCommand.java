// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeShootCommand extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  public IntakeShootCommand( ShooterSubsystem sSubsystem, ElevatorSubsystem eSubsystem, IntakeSubsystem iSubsystem) {
    
    shooterSubsystem = sSubsystem;
    elevatorSubsystem = eSubsystem;
    intakeSubsystem = iSubsystem;
    addRequirements(sSubsystem);
    addRequirements(eSubsystem);
    addRequirements(iSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    shooterSubsystem.checkY();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //shooterSubsystem.ballShoot();
    elevatorSubsystem.elevatorUp();
    intakeSubsystem.intake();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    //shooterSubsystem.shootStop();
    elevatorSubsystem.elevatorStop();
    intakeSubsystem.intakeStop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
