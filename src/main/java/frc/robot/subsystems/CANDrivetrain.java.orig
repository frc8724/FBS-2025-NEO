// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;

/* This class declares the subsystem for the robot drivetrain if controllers are connected via CAN. Make sure to go to
 * RobotContainer and uncomment the line declaring this subsystem and comment the line for PWMDrivetrain.
 *
 * The subsystem contains the objects for the hardware contained in the mechanism and handles low level logic
 * for control. Subsystems are a mechanism that, when used in conjuction with command "Requirements", ensure
 * that hardware is only being used by 1 command at a time.
 */
public class CANDrivetrain extends SubsystemBase {
  /*
   * Class member variables. These variables represent things the class needs to
   * keep track of and use between
   * different method calls.
   */
  DifferentialDrive m_drivetrain;

  TalonSRX leftFront = new TalonSRX(kLeftFrontID);
  TalonSRX rightFront = new TalonSRX(kRightFrontID);
  TalonSRX leftRear = new TalonSRX(kLeftRearID);
  TalonSRX rightRear = new TalonSRX(kRightRearID);

  /*
   * Constructor. This method is called when an instance of the class is created.
   * This should generally be used to set up
   * member variables and perform any configuration or set up necessary on
   * hardware.
   */
  public CANDrivetrain() {

    /*
     * Sets current limits for the drivetrain motors. This helps reduce the
     * likelihood of wheel spin, reduces motor heating
     * at stall (Drivetrain pushing against something) and helps maintain battery
     * voltage under heavy demand
     */
    leftFront.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 1000);
    leftRear.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 1000);
    rightRear.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 1000);
    rightFront.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 1000);

    // The left motor is CCW+
    leftFront.setInverted(false);
    leftRear.setInverted(false);

    // The right motor is CW+
    rightFront.setInverted(true);
    rightRear.setInverted(true);

    // Ensure our followers are following their respective leader
    leftRear.follow(leftFront);
    rightRear.follow(rightFront);
  }

  /*
   * Method to control the drivetrain using arcade drive. Arcade drive takes a
   * speed in the X (forward/back) direction
   * and a rotation about the Z (turning the robot about it's center) and uses
   * these to control the drivetrain motors
   */
  public void arcadeDrive(double speed, double rotation) {
    SmartDashboard.putNumber("speed", speed);
    SmartDashboard.putNumber("rotation", rotation);

    double leftPower = speed;
    double rightPower = speed;

    leftPower -= rotation;
    rightPower += rotation;

    // clip left to [-1, +1]
    leftPower = Math.min(1.0, leftPower);
    leftPower = Math.max(-1.0, leftPower);

    // clip right to [-1, +1]
    rightPower = Math.min(1.0, rightPower);
    rightPower = Math.max(-1.0, rightPower);

    leftFront.set(TalonSRXControlMode.PercentOutput, leftPower);
    rightFront.set(TalonSRXControlMode.PercentOutput, rightPower);
  }

  @Override
  public void periodic() {
    /*
     * This method will be called once per scheduler run. It can be used for running
     * tasks we know we want to update each
     * loop such as processing sensor data. Our drivetrain is simple so we don't
     * have anything to put here
     */
  }
}
