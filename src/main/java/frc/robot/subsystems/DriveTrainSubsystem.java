// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
  /** Creates a new DriveTrainSubsystem. */

  final TalonFX frontLeftTalon = new TalonFX(Constants.FRONT_LEFT_TALON);
  final TalonFX backLeftTalon = new TalonFX(Constants.BACK_LEFT_TALON);
  final TalonFX frontRightTalon = new TalonFX(Constants.FRONT_RIGHT_TALON);
  final TalonFX backRightTalon = new TalonFX(Constants.BACK_RIGHT_TALON);
  double driveTime;
  double speedMod;

  public DriveTrainSubsystem() {

    frontLeftTalon.setInverted(true);
    backLeftTalon.setInverted(true);
    frontRightTalon.setInverted(false);
    backRightTalon.setInverted(false);

  }

  // Autonomous Section
  public void zeroEncoder() {

    frontLeftTalon.setSelectedSensorPosition(0);
    
  }

  public boolean autoForward() {
    if ( Math.abs(frontLeftTalon.getSelectedSensorPosition()) <= ((Constants.ROTATIONAL_CONSTANT / 2) * Constants.AUTO_DISTANCE_FORWARD) ) {
      spinMotorForward();
    } else {
      stopMotor();
      return true;
    }
    if (Timer.getMatchTime() <= 15 ) {
      return true;
    }
    return false;
  }

  public boolean autoReverse() {

    spinMotorBackwards();

    if ( frontLeftTalon.getSelectedSensorPosition() <= ((Constants.ROTATIONAL_CONSTANT / 2) * -Constants.AUTO_DISTANCE_BACKWARDS)) {
      // frontLeftTalon.getSelectedSensorPosition() >= 0  
      spinMotorBackwards();
      return true;
    } else {
      stopMotor();
      return false;
    }
    
  }

  public void spinMotorForward() {

    frontLeftTalon.set(ControlMode.PercentOutput, 0);
    backLeftTalon.set(ControlMode.PercentOutput, 0);
    frontRightTalon.set(ControlMode.PercentOutput, 0);
    backRightTalon.set(ControlMode.PercentOutput, 0);

    SmartDashboard.putNumber("Talon 4 Velocity", ( frontLeftTalon.getSelectedSensorVelocity() / Constants.ROTATIONAL_CONSTANT ) );
    SmartDashboard.putNumber("Talon 4 Position", ( frontLeftTalon.getSelectedSensorPosition() / Constants.ROTATIONAL_CONSTANT ) );

    /*
    frontLeftTalon.set(ControlMode.Velocity, Constants.ROTATIONAL_CONSTANT * 0.5);
    backLeftTalon.set(ControlMode.Velocity, Constants.ROTATIONAL_CONSTANT * 0.5);
    frontRightTalon.set(ControlMode.Velocity, Constants.ROTATIONAL_CONSTANT * 0.5);
    backLeftTalon.set(ControlMode.Velocity, Constants.ROTATIONAL_CONSTANT * 0.5);
    */

  }

  public void spinMotorBackwards() {

    frontLeftTalon.set(ControlMode.PercentOutput, -0.4);
    backLeftTalon.set(ControlMode.PercentOutput, -0.4);
    frontRightTalon.set(ControlMode.PercentOutput, -0.4);
    backRightTalon.set(ControlMode.PercentOutput, -0.4);

    SmartDashboard.putNumber("Talon 4 Velocity", ( frontLeftTalon.getSelectedSensorVelocity() / Constants.ROTATIONAL_CONSTANT ) );
    SmartDashboard.putNumber("Talon 4 Position", ( frontLeftTalon.getSelectedSensorPosition() / Constants.ROTATIONAL_CONSTANT ) );

  }

  public void  stopMotor() {
    
    frontLeftTalon.set(ControlMode.PercentOutput, 0);
    backLeftTalon.set(ControlMode.PercentOutput, 0);
    frontRightTalon.set(ControlMode.PercentOutput, 0);
    backRightTalon.set(ControlMode.PercentOutput, 0);

    SmartDashboard.putNumber("Talon 4 Velocity", ( frontLeftTalon.getSelectedSensorVelocity() / Constants.ROTATIONAL_CONSTANT ) );
    SmartDashboard.putNumber("Talon 4 Position", ( frontLeftTalon.getSelectedSensorPosition() / Constants.ROTATIONAL_CONSTANT ) );

  }

  // Teleop Section
  public void moveMotor( double speed, TalonFX talon) {

    talon.set(ControlMode.PercentOutput, speed);

  }

  public double ensureRange( double val ) {

    return Math.min(Math.max(val, -1), 1);

  }

  public void mecanumDrive( double X, double Y, double R, double Z ) {

    Z = ( -Z + 1 )/2;

    if ( Math.abs(X) + Math.abs(Y) + Math.abs(R) == 0 ) {

      driveTime = Timer.getMatchTime();
    
    }

    if ( Timer.getMatchTime() - driveTime <= Constants.RAMP_UP_TIME ) {

      speedMod = -1 * ((0.5 * (driveTime - Timer.getMatchTime()) / Constants.RAMP_UP_TIME) + 0.5) ;

    } else {

      speedMod = 1;

    }

    moveMotor( Z * speedMod * ensureRange(Y + X + R), frontLeftTalon);
    moveMotor( Z * speedMod * ensureRange(Y - X + R), backLeftTalon);
    moveMotor( Z * speedMod * ensureRange(Y - X - R), frontRightTalon);
    moveMotor( Z * speedMod * ensureRange(Y + X - R), backRightTalon);

    // moveMotor( (1 * ((X / Math.abs(X)) * (X * X)) + ((Y / Math.abs(Y)) * (Y * Y)) + ((R / Math.abs(R)) * (R * R))), frontLeftTalon );
    // moveMotor( (1 * ((X / Math.abs(X)) * (X * X)) - ((Y / Math.abs(Y)) * (Y * Y)) + ((R / Math.abs(R)) * (R * R))), backLeftTalon );
    // moveMotor( (1 * ((X / Math.abs(X)) * (X * X)) - ((Y / Math.abs(Y)) * (Y * Y)) - ((R / Math.abs(R)) * (R * R))), frontRightTalon );
    // moveMotor( (1 * ((X / Math.abs(X)) * (X * X)) + ((Y / Math.abs(Y)) * (Y * Y)) - ((R / Math.abs(R)) * (R * R))), backRightTalon );

    // System.out.println( "Talon Velocity: " + ( frontLeftTalon.getSelectedSensorVelocity() / Constants.ROTATIONAL_CONSTANT ) );
    // System.out.println( "Talon Position: " + ( frontLeftTalon.getSelectedSensorPosition() / Constants.ROTATIONAL_CONSTANT ) );

    SmartDashboard.putNumber("Talon 4 Velocity", ( frontLeftTalon.getSelectedSensorVelocity() / Constants.ROTATIONAL_CONSTANT ) );
    SmartDashboard.putNumber("Talon 4 Position", ( frontLeftTalon.getSelectedSensorPosition() / Constants.ROTATIONAL_CONSTANT ) );
    SmartDashboard.putNumber("Magnitude", ( Z ) );

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
