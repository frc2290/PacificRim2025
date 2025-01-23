package frc.robot.FLYTLib;

// Create a custom exception class
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