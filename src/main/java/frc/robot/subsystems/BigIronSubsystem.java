package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import static frc.robot.Constants.ShooterData.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.BigIronConstants.*;

/** Subsystem of the BIG IRON */
public class BigIronSubsystem extends SubsystemBase {
    // motors
    private final TalonSRX intakeMotor = new TalonSRX(kIntakeID);
    private final CANSparkMax drumLeadMotor = new CANSparkMax(kDrumLeadID, MotorType.kBrushless);
    private final CANSparkMax drumFollowMotor = new CANSparkMax(kDrumFollowID, MotorType.kBrushless);
    private final TalonSRX hoodMotor = new TalonSRX(kHoodID);
    private final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(kHoodEncoder);
    private final TalonSRX beltMotor = new TalonSRX(kBeltID);

    // solenoids
    private final DoubleSolenoid intakeForward = new DoubleSolenoid(2, PneumaticsModuleType.CTREPCM,
            kChannelIntakeForwardGo, kChannelIntakeForwardVent);
    private final DoubleSolenoid intakeBackward = new DoubleSolenoid(2, PneumaticsModuleType.CTREPCM,
            kChannelIntakeBackwardGo, kChannelIntakeBackwardVent);
    private final static Value kGo = Value.kForward;
    private final static Value kVent = Value.kReverse;

    // PID Controllers
    private final PIDController pidH = new PIDController(0, 0, 0);
    private final PIDController pidD = new PIDController(0, 0, 0);

    // Sensors
    public boolean breachSensor = false;
    public boolean intakeSensor = false;
    AnalogInput pressureSensor = new AnalogInput(3);// set to constant later

    double sensor_current_pressure;
    double sensor_current_voltage;

    // Public values
    public boolean fireTheBigIron = false;
    public boolean drumControllerOn = false;
    public boolean drumIdle = false;
    public double drumCurrentSpeed = 0;
    public double drumSP = 0;
    public boolean ejectBall = false;
    public boolean runBeltMan = false;
    public boolean runHoodMan = false;
    public double hoodManPower = 0;

    // Misc.
    private final Timer ejectTimer = new Timer();

    // Private process variables
    private boolean calibrated = false;
    private double hoodSet = 0;
    private boolean drumPIDRunning = false;
    private double hoodCurrentPosition = 0;
    private boolean hoodLowLimit = false;
    private boolean ballOnTheWay = false;

    /** Creates a BigIronSubsystem */
    public BigIronSubsystem() {
        pidD.reset();
        pidH.reset();

        intakeMotor.configFactoryDefault();
        intakeMotor.configNeutralDeadband(0);

        drumLeadMotor.restoreFactoryDefaults();
        drumFollowMotor.restoreFactoryDefaults();
        drumFollowMotor.follow(drumLeadMotor);

    }

    public void runIntake(boolean b) {
        if (b)
            intakeMotor.set(ControlMode.PercentOutput, kIntakePower);
        else
            intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    public boolean readyToFire() {
        return Math.abs(drumSP - drumCurrentSpeed) < kDrumSpeedTolerance
                && Math.abs(hoodSet - hoodCurrentPosition) < kHoodPositionTolerance;
    }

    /**
     * Do the intake
     * 
     * @param intakeState 1 or 3=vent both, 0=intake out, 2= intake retract
     */
    public void intakeDo(int intakeState) {
        if (intakeState == 1 || intakeState == 3) {
            intakeForward.set(kVent);
            intakeBackward.set(kVent);
        } else if (intakeState == 0) {
            intakeForward.set(kGo);
            intakeBackward.set(kVent);
        } else if (intakeState == 2) {
            intakeForward.set(kVent);
            intakeBackward.set(kGo);
        }

    }

    @Override
    public void periodic() {
        readSensors();
        logic();
        if (!runHoodMan)
            runHood();
        else {
            hoodMotor.set(ControlMode.PercentOutput, hoodManPower);
        }
        runDrum();
        runFeedBelt();
    }

    private void logic() {
        if (ejectBall) {
            if (breachSensor)
                ejectTimer.reset();
            else
                ejectTimer.start();
            if (ejectTimer.hasElapsed(1)) {
                ejectTimer.stop();
                ejectTimer.reset();
                ejectBall = false;
            }
        }
    }

    private void readSensors() {

        // display voltage from the pressure sensor (from DEMOboard)
        sensor_current_voltage = pressureSensor.getVoltage();
        SmartDashboard.putNumber("Voltage", sensor_current_voltage);

        // According to Rev documentation pressure = 250 (voltageOut/voltageSupply)-25
        // display calculated pressure
        sensor_current_pressure = 250 * (sensor_current_voltage / 5) - 25;
        SmartDashboard.putNumber("Pressure", sensor_current_pressure);
    }

    private void runHood() {
        if (calibrated) {
            double control = MathUtil.clamp(pidH.calculate(hoodCurrentPosition), -1 * HC, HC);
            if (!hoodLowLimit)
                hoodMotor.set(ControlMode.PercentOutput, control);// set that hood thing
            else
                MathUtil.clamp(control, 0, 1);// limit that hood thing
        }
    }

    private void runDrum() {
        if (drumControllerOn) {
            drumPIDRunning = true;
            drumLeadMotor.set(kDrumDirection * pidD.calculate(drumCurrentSpeed, drumSP));
        } else if (drumIdle) {
            drumPIDRunning = true;
            drumLeadMotor.set(kDrumDirection * pidD.calculate(drumCurrentSpeed, kDrumIdleSpeed));
        } else if (ejectBall) {
            drumLeadMotor.set(kDrumDirection * 0.4);
        } else {
            drumLeadMotor.set(0);
            if (drumPIDRunning) {
                pidD.reset();
                drumPIDRunning = false;
            }
        }
    }

    private void runFeedBelt() {
        if ((fireTheBigIron && breachSensor && readyToFire()) || ballOnTheWay || ejectBall || runBeltMan) {
            if (breachSensor)
                ballOnTheWay = false;
            beltMotor.set(ControlMode.PercentOutput, kBeltPower);// run belt
        } else if (!breachSensor && intakeSensor) {
            ballOnTheWay = true;
            beltMotor.set(ControlMode.PercentOutput, kBeltPower);// run belt
        } else
            beltMotor.set(ControlMode.PercentOutput, 0); // don't run belt
    }
}