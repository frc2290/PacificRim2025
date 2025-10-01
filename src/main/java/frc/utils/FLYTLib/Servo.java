// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
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
