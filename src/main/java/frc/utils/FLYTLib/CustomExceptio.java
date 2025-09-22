// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.utils.FLYTLib;

// Create a custom exception class
/** Legacy custom exception used by older FLYT utilities. */
class FLYTLibException extends Exception {

  public FLYTLibException(String message) {
    super(message);
  }
}

/*
public class Main {
    public static void main(String[] args) {
        try {
            // Simulate an error condition
            int value = 10;
            if (value < 20) {
                throw new CustomException("Value must be greater than or equal to 20");
            }
        } catch (CustomException e) {
            System.out.println("Caught CustomException: " + e.getMessage());
            System.exit(1); // Terminate the program with a non-zero exit code
        }

        // Code that executes if no exception is thrown
        System.out.println("Program continues...");
    }
}*/
