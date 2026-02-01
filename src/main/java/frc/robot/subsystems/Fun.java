package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.io.Console;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;

@Logged(name = "FunSubsystem")
public class Fun extends SubsystemBase {

  private final TalonFX motor = new TalonFX(14);
  private final double kP = 15;
  private final double kD = 1;
  private final double kG = 0.833;
  private final double kV = 0.1;
  private final double kS = 0;
  private final double kA = 0;

  private static final double MIN_ANGLE = -1.5708;
  private static final double MAX_ANGLE = 1.5708;
  private double targetAngleRad = 0;
  private final VoltageOut voltageControl = new VoltageOut(0);
  private final VelocityTorqueCurrentFOC control = new VelocityTorqueCurrentFOC(0);

  private final PIDController pid = new PIDController(kP, 0, kD);

  private final ArmFeedforward ff = new ArmFeedforward(kS, kG, kV, kA);

  // Initialize simulation
  private SingleJointedArmSim armSim = new SingleJointedArmSim(
      DCMotor.getKrakenX60(1), // Motor type
      50,
      SingleJointedArmSim.estimateMOI(0.5, 10), // Arm moment of inertia
      0.5, // Arm length (m)
      (-1.5708), // Min angle (rad)
      (1.5708), // Max angle (rad)
      true, // Simulate gravity
      (0) // Starting position (rad)
  );

  // private final DCMotorSim m_motorSimModel = new DCMotorSim(
  // LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, 1),
  // DCMotor.getKrakenX60Foc(1));

  public Fun() {
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = kP;
    slot0.kI = 0;
    slot0.kD = kD;
    slot0.kG = kG;
    slot0.kV = kV;
    slot0.kS = kS;
    slot0.kA = kA;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    motor.getConfigurator().apply(slot0);
  }

  public void setAngle(double angleRad) {
    targetAngleRad = angleRad;
  }

  public void addAngle(double deltaRad) {
    targetAngleRad = MathUtil.clamp(
        targetAngleRad + deltaRad,
        MIN_ANGLE,
        MAX_ANGLE);
  }

  public void runMotor(double velocity) {
    if (RobotBase.isSimulation()) {
      // crude velocity → voltage mapping just for sim
      motor.setControl(voltageControl.withOutput(velocity * 0.1));
    } else {
      motor.setControl(control.withVelocity(velocity));
    }
  }

  public Command setVelocity(double velocity) {
    return runEnd(() -> runMotor(velocity), () -> runMotor(0));
  }

  public SingleJointedArmSim getSimulation() {
    return armSim;
  }

  @Override
  public void periodic() {
    double currentAngle = armSim.getAngleRads();

    double pidOut = pid.calculate(currentAngle, targetAngleRad);
    double ffOut = ff.calculate(currentAngle, 0);

    double volts = pidOut + ffOut;

    motor.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void simulationPeriodic() {
    var talonFXSim = motor.getSimState();
    // set the supply voltage of the TalonFX
    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    // get the motor voltage of the TalonFX
    // var motorVoltage = talonFXSim.getMotorVoltageMeasure();
    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    // m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
    // m_motorSimModel.update(0.020);
    // assume 20 ms loop time
    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    armSim.setInput(talonFXSim.getMotorVoltage());
    armSim.update(0.020);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            armSim.getCurrentDrawAmps()));
    // Convert radians to rotations (revolutions) and radians/sec to rotations/sec
    double motorPosition = armSim.getAngleRads() / (2.0 * Math.PI);
    double motorVelocity = armSim.getVelocityRadPerSec() / (2.0 * Math.PI);

    motor.getSimState().setRawRotorPosition(motorPosition);
    motor.getSimState().setRotorVelocity(motorVelocity);
    // talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(1));
    // talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(1));
    System.out.println(Units.radiansToDegrees(targetAngleRad));
  }
}
