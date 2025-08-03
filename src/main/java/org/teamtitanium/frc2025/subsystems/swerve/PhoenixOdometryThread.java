package org.teamtitanium.frc2025.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import org.teamtitanium.frc2025.TunerConstants;

public class PhoenixOdometryThread extends Thread {
  private final Lock signalsLock = new ReentrantLock();
  private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();
  private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
  private final List<Queue<Double>> genericQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private static boolean isCANFD =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD();
  private static PhoenixOdometryThread instance;

  public static PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  private PhoenixOdometryThread() {
    setName("PhoenixOdometryThread");
    setDaemon(true);
  }

  @Override
  public void start() {
    if (!timestampQueues.isEmpty() && RobotBase.isReal()) {
      super.start();
    }
  }

  public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    Swerve.odometryLock.lock();
    try {
      var newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
      System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
      newSignals[phoenixSignals.length] = signal;
      phoenixSignals = newSignals;
      phoenixQueues.add(queue);
    } finally {
      signalsLock.unlock();
      Swerve.odometryLock.unlock();
    }
    return queue;
  }

  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    Swerve.odometryLock.lock();
    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
      timestampQueues.add(new ArrayBlockingQueue<>(20));
    } finally {
      signalsLock.unlock();
      Swerve.odometryLock.unlock();
    }
    return queue;
  }

  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Swerve.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Swerve.odometryLock.unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    while (true) {
      signalsLock.lock();
      try {
        if (isCANFD && phoenixSignals.length > 0) {
          BaseStatusSignal.waitForAll(2.0 / Swerve.ODOMETRY_FREQUENCY, phoenixSignals);
        } else {
          Thread.sleep((long) (1000.0 / Swerve.ODOMETRY_FREQUENCY));
          if (phoenixSignals.length > 0) {
            BaseStatusSignal.refreshAll(phoenixSignals);
          }
        }
      } catch (Exception e) {
        e.printStackTrace();
      } finally {
        signalsLock.unlock();
      }

      Swerve.odometryLock.lock();
      try {
        var timestamp = RobotController.getFPGATime() / 1e6;
        var totalLatency = 0.0;
        for (var singal : phoenixSignals) {
          totalLatency += singal.getTimestamp().getLatency();
        }
        if (phoenixSignals.length > 0) {
          timestamp -= totalLatency / phoenixSignals.length;
        }

        for (int i = 0; i < phoenixSignals.length; i++) {
          phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
        }
        for (int i = 0; i < genericSignals.size(); i++) {
          genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
        }
        for (int i = 0; i < timestampQueues.size(); i++) {
          timestampQueues.get(i).offer(timestamp);
        }
      } finally {
        Swerve.odometryLock.unlock();
      }
    }
  }
}
