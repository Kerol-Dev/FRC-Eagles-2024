// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Bu sınıfa herhangi bir statik değişken eklemeyin veya herhangi bir
 * başlatma yapmayın. Ne yaptığınızı bilmiyorsanız, bu dosyayı değiştirmeyin
 * parametre sınıfını startRobot çağrısına değiştirmek dışında.
 */
public final class Main {
  private Main() {}

  /**
   * Ana başlatma fonksiyonu. Burada herhangi bir başlatma yapmayın.
   *
   * <p>Ana robot sınıfınızı değiştirirseniz, parametre türünü değiştirin.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}