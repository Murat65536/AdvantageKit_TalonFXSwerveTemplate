package frc.robot.util;

/**
 * Numerically stable streaming mean/variance (Welford's algorithm), used to measure real
 * closed-loop tracking noise on the robot instead of guessing it.
 */
public class WelfordAccumulator {
  private long count = 0;
  private double mean = 0.0;
  private double m2 = 0.0;

  public void add(double value) {
    count++;
    double delta = value - mean;
    mean += delta / count;
    double delta2 = value - mean;
    m2 += delta * delta2;
  }

  public long getCount() {
    return count;
  }

  public double getMean() {
    return mean;
  }

  /** Sample standard deviation (n-1 denominator). Returns 0 if fewer than 2 samples. */
  public double getStdDev() {
    if (count < 2) {
      return 0.0;
    }
    return Math.sqrt(m2 / (count - 1));
  }
}
