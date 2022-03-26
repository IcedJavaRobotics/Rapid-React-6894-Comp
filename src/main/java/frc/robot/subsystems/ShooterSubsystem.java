// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  
  private Spark shooterBlinkin = new Spark (Constants.BLINKIN_SPARK); //Declares that shooterBlinkin is a spark and gives it the port
  final VictorSPX shooterMotor = new VictorSPX(Constants.SHOOTER_VICTOR);
  public boolean maxSpeed;
  double x;
  double time;
  double startTime;
  
  /** Creates a new ShooterSubsystem. */

  public ShooterSubsystem() {

    shooterMotor.setInverted(true);
    
  }
    
  double y = 1;

  public void checkY() {

    y = shooterBlinkin.get();                   //Assigns the current color value to Y
      
  }

  public void falseMaxSpeed() {

    maxSpeed = false;
    x = 0;
    
  }

  public void autoUpperShoot() {
  
    shooterMotor.set(ControlMode.PercentOutput, Constants.UPPER_SHOOTER_SPEED);
    shooterBlinkin.set(-0.99); //When the shooter activates it turns rainbow

  }

  public void autoLowerShoot() {
  
    shooterMotor.set(ControlMode.PercentOutput, Constants.LOWER_SHOOTER_SPEED);
    shooterBlinkin.set(-0.99); //When the shooter activates it turns rainbow

  }

  public void ballShoot() {

    shooterMotor.set(ControlMode.PercentOutput, Constants.UPPER_SHOOTER_SPEED);
    shooterBlinkin.set(-0.99); //When the shooter activates it turns rainbow
  
    x = x + 1;

    if ( x == 1 ) {
      time = Timer.getMatchTime();
    }

    if (time - Timer.getMatchTime() >= Constants.UPPER_SHOOTER_MAX_SPEED_TIME) {       // this is right if teleop time goes from 0s to more sec, if it's the opposite, referse time and getMatchTime
      maxSpeed = true;
    }

    SmartDashboard.putBoolean("Max Speed =", maxSpeed);

  }

  public void ballLowerShoot() {

    shooterMotor.set(ControlMode.PercentOutput, Constants.LOWER_SHOOTER_SPEED);
    shooterBlinkin.set(-0.99); //When the shooter activates it turns rainbow
    
    x = x + 1;

    if ( x == 1 ) {
      time = Timer.getMatchTime();
    }

    if (time - Timer.getMatchTime() >= Constants.LOWER_SHOOTER_MAX_SPEED_TIME) {       // this is right if teleop time goes from 0s to more sec, if it's the opposite, referse time and getMatchTime
      maxSpeed = true;
      shooterBlinkin.set(.33);
    }

  }

  public void ballUnShoot() {

    shooterMotor.set(ControlMode.PercentOutput, -Constants.UNSHOOTER_SPEED);

  }

  public void shootStop() {

    shooterMotor.set(ControlMode.PercentOutput, 0);
    shooterBlinkin.set(y);    //Changes the value to whatever y is set to

  }

  public void BlinkinBlueStart() {     

    shooterBlinkin.set(0.83);     //Sets Blinkin to blue when left trigger is pushed once.

  }

  public void BlinkinRedStart(){

    shooterBlinkin.set(0.61);   //Sets Blinkin to red when left trigger is pushed twice.

  }

  public void BlinkinEnd(){     //Sets Blinkin to black when left trigger is pushed three times.

    shooterBlinkin.set(0.99);
  }

  public void BlinkinShooter(){ 
    shooterBlinkin.set(-0.99);  //When shooter is activated it turns to rainbow
  }

  public void autoBlinkin() {
    DriverStation.Alliance color;
    color = DriverStation.getAlliance();
    System.out.println(color);
    if(color == DriverStation.Alliance.Blue) {
      shooterBlinkin.set(0.83);
    }
    if(color == DriverStation.Alliance.Red) {
      shooterBlinkin.set(0.61);
    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}