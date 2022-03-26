// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private VictorSPX intakeMotor = new VictorSPX(Constants.INTAKE_VICTOR);

  public IntakeSubsystem() {}

  public void intakeJoystick(double I) {

    if ( I >= 0.5 ) {
      intake();
    } else if ( I <= -0.5 ) {
      outtake();
    } else {
      intakeStop();
    }

  }

  public void intake() {

    intakeMotor.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);

  }

  public void outtake() {

    intakeMotor.set(ControlMode.PercentOutput, -Constants.INTAKE_SPEED);
    

  }

  public void intakeStop() {

    intakeMotor.set(ControlMode.PercentOutput, 0);
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

