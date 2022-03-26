// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  
  private TalonSRX climberLeftMotor;
  private TalonSRX climberRightMotor;
  DigitalInput climberLeftLimitSwitch = new DigitalInput(Constants.CLIMBER_LEFT_LIMIT_SWITCH);
  DigitalInput climberRightLimitSwitch = new DigitalInput(Constants.CLIMBER_RIGHT_LIMIT_SWITCH);

  public ClimberSubsystem() {

    climberLeftMotor = new TalonSRX(Constants.CLIMBER_LEFT_MOTOR);
    climberRightMotor = new TalonSRX(Constants.CLIMBER_RIGHT_MOTOR);

    climberRightMotor.setNeutralMode(NeutralMode.Brake);
    climberLeftMotor.setNeutralMode(NeutralMode.Brake);

    climberRightMotor.setInverted(true);
    
  }

  public void climberUp(){
    
    if(climberLeftMotor.getSelectedSensorPosition()<= 18000){

      climberLeftMotor.set(ControlMode.PercentOutput, Constants.LEFT_CLIMBER_SPEED);

    } else {

      climberLeftMotor.set(ControlMode.PercentOutput, 0);

    }

    if(climberRightMotor.getSelectedSensorPosition()<= 17000){

      climberRightMotor.set(ControlMode.PercentOutput, Constants.RIGHT_CLIMBER_SPEED);

    } else {

      climberRightMotor.set(ControlMode.PercentOutput, 0);

    }
    
    SmartDashboard.putNumber("Talon Left Climber", ( climberLeftMotor.getSelectedSensorPosition()));
    SmartDashboard.putNumber("Talon Right Climber", ( climberRightMotor.getSelectedSensorPosition()));

  }

  public void climberDown(){

    if(climberLeftLimitSwitch.get() == false) {

      climberLeftMotor.set(ControlMode.PercentOutput, - Constants.LEFT_CLIMBER_SPEED);

    } else {

      climberLeftMotor.set(ControlMode.PercentOutput, 0);
      climberLeftMotor.setSelectedSensorPosition(0);

    }

    if(climberRightLimitSwitch.get() == false) {

      climberRightMotor.set(ControlMode.PercentOutput, - Constants.RIGHT_CLIMBER_SPEED);

    } else {

      climberRightMotor.set(ControlMode.PercentOutput, 0);
      climberRightMotor.setSelectedSensorPosition(0);

    }
  }

  public void climberStop() {

    climberLeftMotor.set(ControlMode.PercentOutput, 0);
    climberRightMotor.set(ControlMode.PercentOutput, 0);
    
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
