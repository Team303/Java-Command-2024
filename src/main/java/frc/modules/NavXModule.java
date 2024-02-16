package frc.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.IMUProtocol.StreamResponse;;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;

import static frc.robot.RobotMap.Sensors.NAVX_UPDATE_RATE_HZ;

import java.util.Queue;

public class NavXModule {
  public static class NavXInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    public double yawVelocityRadPerSec = 0.0;
  }

  private final AHRS navX = new AHRS(SPI.Port.kMXP, NAVX_UPDATE_RATE_HZ);
  private final double yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<Double> yawVelocity = pigeon.getAngularVelocityZWorld();

  public NavXModule() {
    navX.reset();
    navX.getActualUpdateRate();
  }
}