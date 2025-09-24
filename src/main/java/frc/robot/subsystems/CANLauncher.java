package frc.robot.subsystems;

import static frc.robot.Constants.LauncherConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANLauncher extends SubsystemBase {
    private final SparkMax m_launchWheel;
    private final SparkMax m_feedWheel;

    private final RelativeEncoder m_launchEncoder;
    private final RelativeEncoder m_feedEncoder;

    public CANLauncher() {
        m_launchWheel = new SparkMax(kLauncherID, MotorType.kBrushless);
        m_feedWheel = new SparkMax(kFeederID, MotorType.kBrushless);

        // Encoders
        m_launchEncoder = m_launchWheel.getEncoder();
        m_feedEncoder = m_feedWheel.getEncoder();

        // === Configure Launch Motor ===
        SparkMaxConfig launchConfig = new SparkMaxConfig();
        launchConfig.smartCurrentLimit(40);
        launchConfig.idleMode(IdleMode.kCoast);
        launchConfig.openLoopRampRate(0.3);

        m_launchWheel.configure(launchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // === Configure Feed Motor ===
        SparkMaxConfig feedConfig = new SparkMaxConfig();
        feedConfig.smartCurrentLimit(30);
        feedConfig.idleMode(IdleMode.kCoast);
        feedConfig.openLoopRampRate(0.2);

        m_feedWheel.configure(feedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command getIntakeCommand() {
        return this.startEnd(
                () -> {
                    setFeedWheel(kIntakeFeederSpeed);
                    setLaunchWheel(kIntakeLauncherSpeed);
                },
                this::stop);
    }

    public void setLaunchWheel(double speed) {
        m_launchWheel.set(speed);
    }

    public void setFeedWheel(double speed) {
        m_feedWheel.set(speed);
    }

    public void stop() {
        m_launchWheel.set(0.0);
        m_feedWheel.set(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Launcher RPM", m_launchEncoder.getVelocity());
        SmartDashboard.putNumber("Launcher Output %", m_launchWheel.getAppliedOutput());
        SmartDashboard.putNumber("Feeder RPM", m_feedEncoder.getVelocity());
    }
}
