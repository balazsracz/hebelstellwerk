#ifndef _UTILS_COMMAND_HANDLER_H
#define _UTILS_COMMAND_HANDLER_H

#include <Arduino.h>

#include "utils/Executor.h"

/**
 * @class CommandHandler
 * @brief Processes line-based commands from the Serial port in a non-blocking
 * way.
 *
 * This class reads characters from the Arduino Serial object, buffers them
 * until a newline character is received, and then processes the complete line
 * as a command. It is designed to be used in the main Arduino loop without
 * blocking execution. To use, simply add this file as a new tab in your Arduino
 * IDE named "CommandHandler.h" and #include "CommandHandler.h" in your main
 * .ino file.
 */
class CommandHandler : public Executable {
 public:
  CommandHandler() { Executor::instance()->add(this); }
  /**
   * @brief Initializes the CommandHandler.
   *
   * This should be called once in the Arduino setup() function.
   * It sets up the Serial communication at the specified baud rate.
   * @param baud_rate The serial communication speed (e.g., 9600, 115200).
   */
  void begin() override {
    // Pre-allocate buffer to improve performance and reduce memory
    // fragmentation.
    input_buffer.reserve(64);
    Serial.println(F("Command handler ready."));
    Serial.println(F("Example command: servo 3 180"));
  }

  /**
   * @brief Checks for and processes incoming serial data.
   *
   * This should be called repeatedly in the Arduino loop() function.
   * It is non-blocking and will return immediately if no data is available.
   */
  void loop() override {
    if (!Serial) {
      return;
    }
    // Process all available characters from the serial buffer.
    while (Serial.available() > 0) {
      char received_char = Serial.read();

      // Handle newline characters, which signify the end of a command.
      // This handles both LF (\n) and CR (\r) line endings.
      if (received_char == '\n' || received_char == '\r') {
        if (input_buffer.length() > 0) {
          process_command();
        }
        // Reset buffer for the next command.
        input_buffer = "";
      } else if (isPrintable(received_char)) {
        // Add valid, printable characters to the buffer.
        input_buffer += received_char;
      }
    }
  }

 private:
  // Buffer to store incoming characters.
  String input_buffer;

  /**
   * @brief Parses and acts on the command stored in the input_buffer.
   */
  void process_command() {
    // Make a copy to keep the original for logging if needed.
    String command = input_buffer;
    command.trim();  // Clean up any extra whitespace.

    if (command.startsWith(F("servo"))) {
      // Find the space after "servo".
      int first_space = command.indexOf(' ');
      if (first_space == -1) {
        print_error(F("Malformed servo command: Missing parameters."));
        return;
      }

      // Find the space between the two numbers.
      int second_space = command.indexOf(' ', first_space + 1);
      if (second_space == -1) {
        print_error(F("Malformed servo command: Missing second parameter."));
        return;
      }

      // Extract substrings for the two parameters.
      String num_str = command.substring(first_space + 1, second_space);
      String deg_str = command.substring(second_space + 1);

      // Convert substrings to integer values.
      int servo_num = num_str.toInt();
      int degrees = deg_str.toInt();

      // Print a confirmation message to the Serial monitor.
      Serial.print(F("CMD: Set servo #"));
      Serial.print(servo_num);
      Serial.print(F(" to "));
      Serial.print(degrees);
      Serial.println(F(" degrees."));

      // Call the function to handle the command.
      set_servo(servo_num, degrees);
    } else {
      // Handle any command that is not recognized.
      Serial.print(F("Unknown command: "));
      Serial.println(command);
    }
  }

  /**
   * @brief A helper to print error messages to Serial using Flash memory for
   * strings.
   * @param error_message The message to print.
   */
  void print_error(const __FlashStringHelper* error_message) {
    Serial.print(F("Error: "));
    Serial.println(error_message);
  }

  /**
   * @brief The target function for the "servo" command. The user has to
   * implement this in the main .ino sketch.
   *
   * @param servo_num The integer identifier for the servo.
   * @param degrees The integer for the servo's target angle (-90 to 270).
   */
  static void set_servo(int servo_num, int degrees);
};

#endif  // _UTILS_COMMAND_HANDLER_H
