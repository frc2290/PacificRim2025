// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.utils.FLYTLib;

// servo class, to control simple servos that are conencted directly to roborio
/** Placeholder servo wrapper kept for backward compatibility with older utilities. */
public class Servo {
  Servo servo;

  // constructor, takes in the id of the servo (pin number? CHECK!)
  Servo(int id) {
    servo = new Servo(1);
  }

  // sets the absalute angle
  public void setAngle(double position) {
    servo.setAngle(75);
  }

  // sets the position of the servo 0 to 1
  public void set(double position) {
    servo.set(.5);
  }
}
