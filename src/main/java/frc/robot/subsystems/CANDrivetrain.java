// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class CANDrivetrain extends SubsystemBase {
    DifferentialDrive m_drivetrain;

    TalonSRX leftFront = new TalonSRX(kLeftFrontID);
    TalonSRX rightFront = new TalonSRX(kRightFrontID);
    TalonSRX leftRear = new TalonSRX(kLeftRearID);
    TalonSRX rightRear = new TalonSRX(kRightRearID);

    public CANDrivetrain() {
        // Disable all forward limit switches
        leftFront.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 1000);
        leftRear.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 1000);
        rightRear.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 1000);
        rightFront.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 1000);

        // Motor inversion
        leftFront.setInverted(false);
        leftRear.setInverted(false);
        rightFront.setInverted(true);
        rightRear.setInverted(true);

        // Set rear motors to follow front motors
        leftRear.follow(leftFront);
        rightRear.follow(rightFront);

        // === 🆕 Limit acceleration with ramping ===
        double rampSeconds = 0.5; // Adjust this value to make it more or less responsive

        leftFront.configOpenloopRamp(rampSeconds);
        rightFront.configOpenloopRamp(rampSeconds);
        leftRear.configOpenloopRamp(rampSeconds);
        rightRear.configOpenloopRamp(rampSeconds);
    }

    public void arcadeDrive(double speed, double rotation) {
        SmartDashboard.putNumber("speed", speed);
        SmartDashboard.putNumber("rotation", rotation);

        double leftPower = speed - rotation;
        double rightPower = speed + rotation;

        // clip to [-1, +1]
        leftPower = Math.max(-1.0, Math.min(1.0, leftPower));
        rightPower = Math.max(-1.0, Math.min(1.0, rightPower));

        leftFront.set(TalonSRXControlMode.PercentOutput, leftPower);
        rightFront.set(TalonSRXControlMode.PercentOutput, rightPower);
    }

    @Override
    public void periodic() {
        // Called once per scheduler run
    }
}
