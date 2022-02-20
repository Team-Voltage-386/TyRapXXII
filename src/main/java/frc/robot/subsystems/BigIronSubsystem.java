package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import static frc.robot.Constants.ShooterData.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.robot.Constants.BigIronConstants.*;

/**
 * Subsystem of the BIG IRON
 * (Ball movement stuff)
 */
public class BigIronSubsystem extends SubsystemBase {
    // motors
    private final TalonSRX intakeMotor = new TalonSRX(kIntakeID);
    private final CANSparkMax drumLeadMotor = new CANSparkMax(kDrumLeadID, MotorType.kBrushless);
    private final CANSparkMax drumFollowMotor = new CANSparkMax(kDrumFollowID, MotorType.kBrushless);
    private final TalonSRX hoodMotor = new TalonSRX(kHoodID);
    private final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(kHoodEncoderPin);
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
    private final DigitalInput breachSensor = new DigitalInput(kBreachSensorPin);
    private final ColorSensorV3 intakeSensor = new ColorSensorV3(I2C.Port.kMXP);
    private final AnalogInput pressureSensor = new AnalogInput(3);// airTank sensor

    double tankPressure;

    // Public values
    public boolean fireTheBigIron = false;
    public boolean drumControllerOn = false;
    public boolean drumIdle = false;
    public double drumCurrentSpeed = 0;
    public double drumSP = 0;
    public boolean ejectBall = false;
    public boolean runBeltMan = false;
    public boolean runHoodMan = false;
    public String ball1Col = "null";
    public String ball2Col = "null";
    public double hoodManPower = 0;
    public int ballCount = 0;
    public String colorOurs;
    public String colorAnti;

    // Misc.
    private final Timer ejectTimer = new Timer();

    // Private process variables
    private boolean calibrated = false;
    private double hoodSet = 0;
    private boolean drumPIDRunning = false;
    private double hoodCurrentPosition = 0;
    private boolean hoodLowLimit = false;
    public boolean ballOnTheWay = false;
    private boolean breachSensorFlag = false;
    private boolean intakeSensorFlag = false;

    /** Creates a BigIronSubsystem */
    public BigIronSubsystem() {
        pidD.reset();
        pidH.reset();

        intakeMotor.configFactoryDefault();
        intakeMotor.configNeutralDeadband(0);

        drumLeadMotor.restoreFactoryDefaults();
        drumFollowMotor.restoreFactoryDefaults();
        drumFollowMotor.follow(drumLeadMotor);

        colorOurs=DriverStation.getAlliance().toString();
        if(colorOurs.equals("Blue")) colorAnti="Red";
        else if(colorOurs.equals("Red")) colorAnti="Blue";
        else colorOurs.equals("Invalid");
    }

    public void reset() {
        beltTimer.stop();
        beltTimer.reset();
        ejectTimer.stop();
        ejectTimer.reset();
        ejectBall = false;
        calibrated = false;
        drumIdle = false;
        ball1Col = "null";
        ball2Col = "null";
        ballOnTheWay = false;
        ballCount = 0;
        drumControllerOn = false;
        drumControllerOn = false;
        ejectBelt = false;
        ef = false;
    }

    public void runIntake(boolean b) {
        if (intakeOut) {
            if (b)
                intakeMotor.set(ControlMode.PercentOutput, kIntakePower);
            else {
                intakeMotor.set(ControlMode.PercentOutput, kIntakeReversePower);
            }
        } else
            intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    public boolean readyToFire() {
        return Math.abs(drumSP - drumCurrentSpeed) < kDrumSpeedTolerance
                && Math.abs(hoodSet - hoodCurrentPosition) < kHoodPositionTolerance;
    }

    private boolean intakeOut = false;
    private Timer t = new Timer();

    /**
     * Do the intake
     * 
     * @param hOut    - controller raw button
     * @param trigger - controller raw button pressed
     */
    public void intakeDo(boolean b) {
        if (b) {
            if (!intakeOut) {
                intakeOut = true;
                t.start();
                intakeForward.set(kGo);
                intakeBackward.set(kVent);
            } else {
                intakeOut = false;
                t.stop();
                t.reset();
                intakeForward.set(kVent);
                intakeBackward.set(kGo);
            }
        } else if (t.hasElapsed(0.5)) {
            intakeForward.set(kVent);
            intakeBackward.set(kVent);
        }
    }

    public void reLoad() {
        ball1Col = ball2Col;
        ballCount--;
        if (ballCount > 0) {
            ball2Col = "null";
            ballCount = 0;
            ballOnTheWay = true;
        }
    }

    @Override
    public void periodic() {
        readSensors();
        logic();
        runHood();
        runDrum();
        runFeedBelt();
        updateWidgets();
    }

    boolean ef = false;

    private void logic() {
        if (ejectBall) {
            if (ballCount == 1) {
                if (!ef) {
                    ejectTimer.reset();
                    ejectTimer.start();
                    ef = true;
                }
                if (!breachSensorFlag && !ejectTimer.hasElapsed(2)) ejectBelt = true;
                else if (breachSensorFlag && ejectTimer.hasElapsed(2)) {
                    ejectBall = false;
                    ejectBall = true;
                } else ejectBelt = false;
            } 
        } else if (ejectBelt) {
            if (!breachSensorFlag) {
                reLoad();
                ejectBelt = false;
                ef = false;
            }
        } else ejectBelt = false;
    }

    private void readSensors() {
        hoodCurrentPosition = hoodEncoder.get();
        breachSensorFlag = !breachSensor.get();
        double inSens = intakeSensor.getProximity();
        intakeSensorFlag = (inSens > 310 || inSens < 230) && intakeOut;
        // According to Rev documentation pressure = 250 (voltageOut/voltageSupply)-25
        tankPressure = 250 * (pressureSensor.getVoltage() / 5) - 25;
    }

    private void runHood() {
        if (calibrated) {
            double control = MathUtil.clamp(pidH.calculate(hoodCurrentPosition), -1 * HC, HC);
            if (!hoodLowLimit)
                hoodMotor.set(ControlMode.PercentOutput, control);// set that hood thing
            else
                MathUtil.clamp(control, -1, 0);// limit that hood thing
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
            drumLeadMotor.set(kDrumDirection * 0.6);
        } else {
            drumLeadMotor.set(0);
            if (drumPIDRunning) {
                pidD.reset();
                drumPIDRunning = false;
            }
        }
    }

    private Timer beltTimer = new Timer();
    private boolean woundBack = false;
    private boolean ejectBelt = false;

    private void runFeedBelt() {
        if (ballCount == 0) {
            if (!ballOnTheWay) {
                if (!breachSensorFlag && intakeSensorFlag) {
                    ballOnTheWay = true;
                    beltTimer.start();
                    if (ball1Col.equals("null"))
                        ball1Col = getColor();
                }
                beltMotor.set(ControlMode.PercentOutput, 0);
            } else {
                beltMotor.set(ControlMode.PercentOutput, kBeltPower);
                if (breachSensorFlag || beltTimer.hasElapsed(5)) {
                    beltTimer.stop();
                    beltTimer.reset();
                    ballOnTheWay = false;
                    beltMotor.set(ControlMode.PercentOutput, 0);
                    if (breachSensorFlag) {
                        woundBack = false;
                        ballCount++;
                    }
                }
            }
        } else if (ballCount == 1) {
            if (!woundBack) {
                if (breachSensorFlag)
                    beltTimer.start();
                beltMotor.set(ControlMode.PercentOutput, kBeltReversePower);
                if (beltTimer.hasElapsed(0.6)) {
                    beltTimer.stop();
                    beltTimer.reset();
                    woundBack = true;
                    beltMotor.set(ControlMode.PercentOutput, 0);
                }
            } else {
                if (intakeSensorFlag) {
                    ballCount++;
                }
                if (ejectBall) {
                    if (breachSensorFlag) {
                        if (ejectTimer.hasElapsed(2)) beltMotor.set(ControlMode.PercentOutput, kBeltPower);
                        else beltMotor.set(ControlMode.PercentOutput, 0);
                    } else beltMotor.set(ControlMode.PercentOutput, kBeltPower);
                }
                else beltMotor.set(ControlMode.PercentOutput, 0); 
            }
        } else if (ballCount == 2) {
            if (ball2Col.equals("null")) ball2Col = getColor();
            if (!breachSensorFlag) beltMotor.set(ControlMode.PercentOutput, kBeltPower);
            else beltMotor.set(ControlMode.PercentOutput, 0);
      beltMotor.set(ControlMode.PercentOutput, 0);  
    } 
}

    public final ShuffleboardTab tab = Shuffleboard.getTab("BigIron");
    private final NetworkTableEntry eWidget = tab.add("Eject", false).withPosition(0, 0).getEntry();
    private final NetworkTableEntry ejectBeltWidget = tab.add("EjectBelt", false).withPosition(0, 1).getEntry();
    private final NetworkTableEntry encWidget = tab.add("HEncoder", 0).withPosition(0, 2).getEntry();
    private final NetworkTableEntry botwWidget = tab.add("botw", false).withPosition(1, 0).getEntry();
    private final NetworkTableEntry presWidget = tab.add("pres", 0).withPosition(1, 1).getEntry();
    private final NetworkTableEntry colWidget = tab.add("col", 0).withPosition(1, 2).getEntry();
    private final NetworkTableEntry bcWidget = tab.add("ballCount", 0).withPosition(2, 0).getEntry();
    private final NetworkTableEntry b1Widget = tab.add("ball1", "null").withPosition(2, 1).getEntry();
    private final NetworkTableEntry b2Widget = tab.add("ball2", "null").withPosition(2, 2).getEntry();

    private String getColor() {
        double b = intakeSensor.getBlue();
        double r = intakeSensor.getRed();
        if (b > r)
            return "blue";
        return "red";
    }

    private void updateWidgets() {
        eWidget.setBoolean(ejectBall);
        ejectBeltWidget.setBoolean(ejectBelt);
        encWidget.setDouble(hoodCurrentPosition);
        botwWidget.setBoolean(ballOnTheWay);
        presWidget.setDouble(tankPressure);
        colWidget.setDouble(intakeSensor.getProximity());
        bcWidget.setDouble(ballCount);
        b1Widget.setString(ball1Col);
        b2Widget.setString(ball2Col);
    }
}