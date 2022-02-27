package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.Constants.ShooterData;
import frc.robot.Utils.Flags;

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

import static frc.robot.Constants.BigIronConstants.*;

/**
 * Subsystem of the BIG IRON
 * (Ball movement stuff)
 */
public class BigIronSubsystem extends SubsystemBase {
    // motors
    private final TalonSRX intakeMotor = new TalonSRX(kIntakeID);
    private final CANSparkMax drumOneMotor = new CANSparkMax(kDrumOneID, MotorType.kBrushless);
    private final CANSparkMax drumTwoMotor = new CANSparkMax(kDrumTwoID, MotorType.kBrushless);
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
    private final PIDController pidH = new PIDController(HP, HI, HD);
    public final PIDController pidD = new PIDController(DP, DI, DD);

    // Sensors
    private final DigitalInput breachSensor = new DigitalInput(kBreachSensorPin);
    private final ColorSensorV3 intakeSensor = new ColorSensorV3(I2C.Port.kMXP);
    private final AnalogInput pressureSensor = new AnalogInput(3);// airTank sensor
    private final DigitalInput hoodLimit = new DigitalInput(kHoodDownLimitPin);

    double tankPressure;

    // Public values
    public boolean fireTheBigIron = false;
    public boolean drumControllerOn = false;
    public boolean drumIdle = false;
    public double drumCurrentSpeed = 0;
    public double drumSP = 2000;
    public boolean ejectBall = false;
    public boolean runBeltMan = false;
    public boolean runHoodMan = false;
    public String ball1Col = "null";
    public String ball2Col = "null";
    public double hoodManPower = 0;
    public int ballCount = 0;

    // Misc.
    private final Timer ejectTimer = new Timer();

    // Private process variables
    private boolean calibrated = false;
    public double hoodSet = 0.05;
    private boolean drumPIDRunning = false;
    private double hoodCurrentPosition = 0;
    private boolean hoodLowLimit = false;
    public boolean ballOnTheWay = false;
    public boolean breachSensorFlag = false;
    private boolean intakeSensorFlag = false;

    /** Creates a BigIronSubsystem */
    public BigIronSubsystem() {
        pidD.reset();
        pidH.reset();

        intakeMotor.configFactoryDefault();
        intakeMotor.configNeutralDeadband(0);
        drumOneMotor.setInverted(false);
        drumTwoMotor.setInverted(false);

        //drumLeadMotor.restoreFactoryDefaults();
        //drumFollowMotor.restoreFactoryDefaults();
        drumTwoMotor.follow(drumOneMotor,true);

    }

    public void ballFailedDebug() {
        if (ballCount == 2) {
            ballCount = 1;
            woundBack = false;
        }
    }

    public void empty() {
        ballCount = 0;
        ballOnTheWay = false;
        woundBack = false;
    }

    public void reset() {
        pidD.reset();
        pidH.reset();
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
        fireTheBigIron = false;
        ef = false;
        eff = false;
    }

    public void runIntake(boolean b) {
        if (intakeOut) {
            if (b)
                intakeMotor.set(ControlMode.PercentOutput, kIntakePower);
            else {
                intakeMotor.set(ControlMode.PercentOutput, 0);
            }
        } else
            intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    public boolean readyToFire() {
        return Math.abs(drumSP - drumCurrentSpeed) < kDrumSpeedTolerance && Math.abs(hoodSet - hoodCurrentPosition) < kHoodPositionTolerance;
    }

    public boolean intakeOut = false;
    private Timer t = new Timer();

    /**
     * Do the intake
     * 
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
        } else if (t.hasElapsed(1)) {
            if (Flags.complianceOverride) {
                intakeForward.set(kGo);
                intakeBackward.set(kVent);
            } else {
                intakeForward.set(kVent);
                intakeBackward.set(kVent);
            }
        }
    }

    public void reLoad() {
        ball1Col = ball2Col;
        if (ballCount > 1) {
            ball2Col = "null";
            ballOnTheWay = true;
        }
        ballCount = 0;
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
    boolean eff = false;

    private void logic() {
        if (ejectBall) {
            if (!ef) {
                ejectTimer.reset();
                ejectTimer.start();
                ef = true;
                eff = false;
            }
        }
    }

    private void readSensors() {
        hoodCurrentPosition = hoodEncoder.get();
        breachSensorFlag = !breachSensor.get();
        double inSens = intakeSensor.getProximity();
        intakeSensorFlag = (inSens > 310 || inSens < 230) && intakeOut;
        // According to Rev documentation pressure = 250 (voltageOut/voltageSupply)-25
        tankPressure = 250 * (pressureSensor.getVoltage() / 5) - 25;
        hoodLowLimit = !hoodLimit.get();
        drumCurrentSpeed = -1*drumOneMotor.getEncoder().getVelocity();
    }

    private void runHood() {
        if (calibrated) {
            double control = MathUtil.clamp(pidH.calculate(hoodCurrentPosition, hoodSet), -1, 1);
            if (!hoodLowLimit) hoodMotor.set(ControlMode.PercentOutput, control);// set that hood thing
            else hoodMotor.set(ControlMode.PercentOutput, MathUtil.clamp(control, 1, 0));// limit that hood thing*/
        } else {
            if (hoodLowLimit) {
                calibrated = true;
                pidH.reset();
                hoodEncoder.reset();
            }
            else {
                hoodMotor.set(ControlMode.PercentOutput, -0.5);
            }
        }
    }

    private void runDrum() {
        if (fireTheBigIron) {
            drumPIDRunning = true;
            double control = kDrumDirection * pidD.calculate(drumCurrentSpeed, drumSP);
            drumOneMotor.set(control);
        } else if (drumIdle) {
            drumPIDRunning = true;
            double control = kDrumDirection * pidD.calculate(drumCurrentSpeed, kDrumIdleSpeed);
            drumOneMotor.set(control);
        } else if (ejectBall || eff) {
            drumOneMotor.set(kDrumDirection*0.3);
        } else {
            drumOneMotor.set(0);
            if (drumPIDRunning) {
                pidD.reset();
                drumPIDRunning = false;
            }
        }
    }

    private Timer beltTimer = new Timer();
    private boolean woundBack = false;
    private boolean fF = false;

    private void runFeedBelt() {
        if (fireTheBigIron) {
            if (readyToFire()) {
                beltMotor.set(ControlMode.PercentOutput, kBeltPower); 
            } else beltMotor.set(ControlMode.PercentOutput, 0);
        } else if (ballCount == 0) {
            if (!ballOnTheWay) {
                if (!breachSensorFlag && intakeSensorFlag) {
                    ballOnTheWay = true;
                    beltTimer.start();
                    if (ball1Col.equals("null")) ball1Col = getColor();
                }
                beltMotor.set(ControlMode.PercentOutput, 0);
            } else {
                runIntake(true);
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
                if (breachSensorFlag) beltTimer.start();
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
                } else if (ejectBall) {
                    if (breachSensorFlag) {
                        if (ejectTimer.hasElapsed(0.5)) {
                            beltMotor.set(ControlMode.PercentOutput, kBeltPower);
                            eff = true;
                            ejectBall = false;
                        }
                        else beltMotor.set(ControlMode.PercentOutput, 0);
                    } else beltMotor.set(ControlMode.PercentOutput, kBeltPower);
                } else if (eff) {
                    if (!breachSensorFlag) {
                        beltMotor.set(ControlMode.PercentOutput, 0);
                        reLoad();
                        ef = false;
                        eff = ef;
                    }
                } else beltMotor.set(ControlMode.PercentOutput, 0); 
            }
        } else if (ballCount == 2) {
            if (ball2Col.equals("null")) ball2Col = getColor();
            else if (ejectBall) {
                if (ejectTimer.hasElapsed(0.5)) {
                    beltMotor.set(ControlMode.PercentOutput, kBeltPower);
                    ejectBall = false;
                    eff = true;
                } else beltMotor.set(ControlMode.PercentOutput, 0);
            } else if (eff) {
                if (!breachSensorFlag) {
                    beltMotor.set(ControlMode.PercentOutput, 0);
                    ball1Col = ball2Col;
                    ball2Col = "null";
                    ballCount = 1;
                    ef = false;
                    eff = ef;
                    fireTheBigIron = false;
                    ejectBall = true;
                    woundBack = true;
                }
            } else beltMotor.set(ControlMode.PercentOutput, 0);  
            if (!breachSensorFlag) beltMotor.set(ControlMode.PercentOutput, kBeltPower);
    } 
}

    public final ShuffleboardTab tab = Shuffleboard.getTab("BigIron");
    private final NetworkTableEntry eWidget = tab.add("Eject", false).withPosition(0, 0).getEntry();
    private final NetworkTableEntry ejectBeltWidget = tab.add("BreachSensor", false).withPosition(0, 1).getEntry();
    private final NetworkTableEntry colWidget = tab.add("col", 0).withPosition(0, 2).getEntry();
    private final NetworkTableEntry botwWidget = tab.add("botw", false).withPosition(1, 0).getEntry();
    private final NetworkTableEntry presWidget = tab.add("pres", 0).withPosition(1, 1).getEntry();
    private final NetworkTableEntry inWidget = tab.add("intake",false).withPosition(1, 2).getEntry();
    private final NetworkTableEntry bcWidget = tab.add("ballCount", 0).withPosition(2, 0).getEntry();
    private final NetworkTableEntry b1Widget = tab.add("ball1", "null").withPosition(2, 1).getEntry();
    private final NetworkTableEntry b2Widget = tab.add("ball2", "null").withPosition(2, 2).getEntry();
    private final NetworkTableEntry dsWidget = tab.add("drumSpeed",0).withPosition(0,3).getEntry();
    private final NetworkTableEntry hWidget = tab.add("hoodPosition",0).withPosition(1, 3).getEntry();
    private final NetworkTableEntry rtfWidget = tab.add("rtf",false).withPosition(2, 3).getEntry();
    private final NetworkTableEntry dWidget = tab.add("distance",0).withPosition(3,3).getEntry();

    private void updateWidgets() {
        eWidget.setBoolean(hoodLowLimit);
        ejectBeltWidget.setBoolean(breachSensorFlag);
        colWidget.setDouble(intakeSensor.getProximity());
        botwWidget.setBoolean(ballOnTheWay);
        presWidget.setDouble(tankPressure);
        inWidget.setBoolean(intakeSensorFlag);
        bcWidget.setDouble(ballCount);
        b1Widget.setString(ball1Col);
        b2Widget.setString(ball2Col);
        dsWidget.setDouble(drumCurrentSpeed);
        hWidget.setDouble(hoodCurrentPosition);
        rtfWidget.setBoolean(readyToFire());
        dWidget.setDouble(Utils.Flags.targetDistance);
    }

    private String getColor() {
        double b = intakeSensor.getBlue();
        double r = intakeSensor.getRed();
        if (b > r) return "Blue";
        return "Red";
    }

    public void setAimDistance(double m) {
        int i = ShooterData.distances.length-1;
        for (int j = 1; j < ShooterData.distances.length; j++) if (m < ShooterData.distances[j]) i = j;
        double upper = ShooterData.distances[i];
        double lower = ShooterData.distances[i-1];
        double lerpFactor = (m-lower)/(upper-lower);
        upper = ShooterData.drumSpeeds[i];
        lower = ShooterData.drumSpeeds[i-1];
        drumSP = (int)Utils.lerp(lower, upper, lerpFactor);
        upper = ShooterData.hoodPositions[i];
        lower = ShooterData.hoodPositions[i-1];
        hoodSet = Utils.lerp(lower, upper, lerpFactor);
    }
}