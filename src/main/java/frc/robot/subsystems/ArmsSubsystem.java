// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmsSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private VictorSPX rightArmMotor;
  private VictorSPX leftArmMotor;
  public double leftPosition = 0;
  public double rightPosition = 0;
  Counter leftMotorCounter = new Counter(new DigitalInput(0));
  Counter rightMotorCounter = new Counter(new DigitalInput(1));
  DigitalInput topLeftLimit = new DigitalInput(2);
  DigitalInput bottomLeftLimit = new DigitalInput(3);
  DigitalInput topRightLimit = new DigitalInput(4);
  DigitalInput bottomRightLimit = new DigitalInput(5);

  public ArmsSubsystem() {

    if (Math.abs(leftPosition - rightPosition) > 5) {
      System.out.println("problem occured, arms not synced");
    }
    rightArmMotor = new VictorSPX(Constants.RIGHT_ARM);
    leftArmMotor = new VictorSPX(Constants.LEFT_ARM);

    rightArmMotor.setInverted(true);
    leftArmMotor.setInverted(false);

  }

  public void armsDown() {

    if (bottomLeftLimit.get() == false) {

      leftArmMotor.set(ControlMode.PercentOutput, Constants.ARM_SPEED);
      leftPosition += leftMotorCounter.get();
      leftMotorCounter.reset();

    }else {

      leftArmMotor.set(ControlMode.PercentOutput, 0);
      leftPosition = 0;

    }

    if (bottomRightLimit.get() == false) {

      rightArmMotor.set(ControlMode.PercentOutput, Constants.ARM_SPEED);
      rightPosition += rightMotorCounter.get();
      rightMotorCounter.reset();

    }else {

      rightArmMotor.set(ControlMode.PercentOutput, 0);
      rightPosition = 0;

    }
    
  }

  public void armsUp() {

    if (topLeftLimit.get() == false) {

      leftArmMotor.set(ControlMode.PercentOutput, -Constants.ARM_SPEED);
      leftPosition -= leftMotorCounter.get();
      leftMotorCounter.reset();

    } else {

      leftArmMotor.set(ControlMode.PercentOutput, 0);

    }

    if (topRightLimit.get() == false) {

      rightArmMotor.set(ControlMode.PercentOutput, -Constants.ARM_SPEED);
      rightPosition -= rightMotorCounter.get();
      rightMotorCounter.reset();

    }else {

      rightArmMotor.set(ControlMode.PercentOutput, 0);

    }
    
  }

  public void armsStop() {

    rightArmMotor.set(ControlMode.PercentOutput, 0);
    leftArmMotor.set(ControlMode.PercentOutput, 0);
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
