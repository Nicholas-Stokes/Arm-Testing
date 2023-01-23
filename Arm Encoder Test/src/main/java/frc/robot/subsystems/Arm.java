// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  AnalogInput shoulderLamprey = new AnalogInput(0);
  TalonFX shoulderMotor = new TalonFX(16);
  TalonFX elbowMotor = new TalonFX(3);

  double kP = 0.2;
  double kI = 0.0;
  double kD = 0.0;

  
  /** Creates a new ExampleSubsystem. */
  public Arm() {
    

  }

  public void setElbowSpeed(double speed) {
    elbowMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setShoulderSpeed(double speed) {
    shoulderMotor.set(ControlMode.PercentOutput, speed);
  }

  public double getShoulderPositionTicks() {
    return shoulderMotor.getSelectedSensorPosition();

  }

  public double getElbowPositionTicks() {
    return elbowMotor.getSelectedSensorPosition();
  }

  public void holdElbowPosition(double ticks) {
    elbowMotor.set(ControlMode.Position, ticks);
  }

  public void holdShoulderPosition(double ticks) {
    shoulderMotor.set(ControlMode.Position, ticks);
  }


  //This returns the Shoulder Lamprey's Position in Degrees
  public double getShoulderEncoderPosition(){
    return shoulderLamprey.getVoltage() * (360.0/3.30);
    
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder Position", getShoulderEncoderPosition());
    SmartDashboard.putNumber("Motor Speed", shoulderMotor.getMotorOutputPercent());
    // This method will be called once per scheduler run
  }

}
