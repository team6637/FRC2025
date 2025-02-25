// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of that. You aren't reading this eric?
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
    private final SparkMax algaeIntakeMotor;
    private final SparkMax algaeUpDownMotor;
    private final DutyCycleEncoder throughboreEncoder;
    private final PIDController pid;

    private double kP = 0.003;
    private boolean usingPID = true;

    private final double upSetpoint = 127.0;
    private final double downSetpoint = 52.0;

    private double setpoint = upSetpoint;
    private double minSetpoint = downSetpoint;
    private double maxSetpoint = upSetpoint;

    double maxLiftSpeed;

    
    
    public AlgaeIntake() {
        SmartDashboard.putNumber("algae kp", kP);
        algaeIntakeMotor = new SparkMax(8,MotorType.kBrushless);
        algaeUpDownMotor = new SparkMax(9,MotorType.kBrushless);
        throughboreEncoder = new DutyCycleEncoder(1);
        pid = new PIDController(kP, 0.0, 0.0);
        pid.setTolerance(2, 2);

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig algaeUpDownConfig = new SparkMaxConfig();

        globalConfig
            .smartCurrentLimit(30)
            .idleMode(IdleMode.kCoast);

        algaeUpDownConfig
            .apply(globalConfig)
            .inverted(true);

        algaeUpDownMotor.configure(algaeUpDownConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public double getSensorAsDegrees() {
        return throughboreEncoder.get() * 360.0;
    }


    public void up() {
        setSetpoint(upSetpoint);
    }
    
    
    public void down() {
        setSetpoint(downSetpoint);
    }


    public void setSetpoint(double newSetpoint) {
        if (newSetpoint > maxSetpoint) {
            setpoint = maxSetpoint;
        } else if (newSetpoint < minSetpoint) {
            setpoint = minSetpoint;
        } else {
            setpoint = newSetpoint;
        }
    }

        public void stop() {
            if(!usingPID) {
                algaeUpDownMotor.set(0.0);
            }
        }
    
    public void intake() {
        if (getSensorAsDegrees() < 110.0) {
            algaeIntakeMotor.set(0.8);
        }
    }
    
    public void stopIntake() {
        algaeIntakeMotor.set(0.0);
    }

    public void score() {
        if (getSensorAsDegrees() < 110.0) {
            algaeIntakeMotor.set(-0.8);
        }
    }

    public boolean atSetpoint() {
        return pid.atSetpoint();
    }

    public void setUsingPID(boolean v) {
        usingPID = v;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("algae intake position", getSensorAsDegrees());

        double speed = pid.calculate(getSensorAsDegrees(), setpoint);
        SmartDashboard.putNumber("algae intake setpoint", setpoint);
        SmartDashboard.putNumber("algae intake speed", speed);

        maxLiftSpeed = 0.7;

        // if going down, just set speed limit
        if(setpoint < 50) maxLiftSpeed = 0.1;

        if(Math.abs(speed) > maxLiftSpeed) speed = maxLiftSpeed * Math.signum(speed);

        if(usingPID) {
            algaeUpDownMotor.set(speed);
        }
    }
}



































//Forrest Was Here