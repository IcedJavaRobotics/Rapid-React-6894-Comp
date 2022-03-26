// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ShooterCommand;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  
  private VictorSPX elevatorVictor = new VictorSPX(Constants.ELEVATOR_VICTOR);
  private DigitalInput ballDetect = new DigitalInput(Constants.BALL_DETECT);
  // private ShooterSubsystem shooterSubsystem;

  public ElevatorSubsystem() {}

  public void elevatorJoystick(double E, ShooterSubsystem shooterSubsystem) {
    
    if ( E >= 0.5 ) {

      if ( ballDetect.get() == true ) {             // this is when the button is pressed and the ball is there

        if (shooterSubsystem.maxSpeed == true) {    // this is when the shooter is at max speed

          elevatorUp();

        } else {

          elevatorStop();

        }

      }  else {

        elevatorUp();

      }

    } else if ( E <= -0.5 ) {
      
      elevatorDown();

    } else {

      elevatorStop();

    }

    SmartDashboard.putBoolean("Ball detected = ", ballDetect.get());

  }

  public void elevatorUp() {
  
    elevatorVictor.set(ControlMode.PercentOutput, Constants.ELEVATOR_SPEED);

  }

  public void elevatorDown() {
    
    elevatorVictor.set(ControlMode.PercentOutput, - Constants.ELEVATOR_SPEED);
    

  }

  public void elevatorStop() {

    elevatorVictor.set(ControlMode.PercentOutput, 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
