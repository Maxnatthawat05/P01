#include <Arduino.h>
#include <tcs3200.h> // Include TCS3200 library
#include <bits/stdc++.h>
#include <ESP32Servo.h>

using namespace std;

Servo servoRed;    // Servo motor for red pepper
Servo servoGreen;  // Servo motor for green pepper

tcs3200 tcs(2, 4, 5, 18, 22); // (S0, S1, S2, S3, output pin)

double h = 0; // Initialize H value
double h_sum = 0; // Initialize sum of H values
int h_count = 0; // Initialize count of H values

void rgb_to_hsv(double r, double g, double b);

void setup()
{
  Serial.begin(9600);

  servoRed.attach(13);   // Attach the servo motor for red pepper to pin 9
  servoGreen.attach(12); // Attach the servo motor for green pepper to pin 10
}

void loop()
{
  double red, green, blue;

  red = tcs.colorRead('r');   // reads color value for red
  green = tcs.colorRead('g'); // reads color value for green
  blue = tcs.colorRead('b');  // reads color value for blue

  rgb_to_hsv(red, green, blue);

  // Check if H is within the specified range
  if ((240 <= h && h <= 301)) {
    servoRed.write(0);
    servoGreen.write(0);;
    //Serial.println("No color detected");
  } else {
    // Add H value to sum and increment count
    h_sum != 0;
    h_sum += h;
    h_count++;
  }

  // Read H value for 10 loops, then calculate average and check the color range
  if (h_count == 10) {
    double h_avg = h_sum / h_count;

    // Check the H average value and control the servo motors accordingly
 if (h_avg >= 310 && h_avg <= 355) {
      Serial.println("Red");
      servoRed.write(120);   // Rotate servo for red pepper to 120 degrees
      delay(2000); // Delay for 5 seconds
      servoRed.write(0);   // Rotate servo back to 0 degrees
    } else if (h_avg >= 12 && h_avg <= 60) {
      Serial.println("Green");
      servoGreen.write(120); // Rotate servo for green pepper to 120 degrees
      delay(2000); // Delay for 5 seconds
      servoGreen.write(0); // Rotate servo back to 0 degrees
    } else {
      Serial.println("Not");
    }

    // Reset sum and count for next set of readings
    h_sum = 0;
    h_count = 0;
  }

  delay(100);
}

void rgb_to_hsv(double r, double g, double b)
{
  // R, G, B values are divided by 255
  // to change the range from 0..255 to 0..1
  r = r / 255.0;
  g = g / 255.0;
  b = b / 255.0;

  // h, s, v = hue, saturation, value
  double cmax = max(r, max(g, b)); // maximum of r, g, b
  double cmin = min(r, min(g, b)); // minimum of r, g, b
  double diff = cmax - cmin;       // diff of cmax and cmin.

  // if cmax and cmax are equal then h = 0
  if (cmax == cmin)
    h = 0;

  // if cmax equal r then compute h
  else if (cmax == r)
    h = fmod(60 * ((g - b) / diff) + 360, 360);

  // if cmax equal g then compute h
  else if (cmax == g)
    h = fmod(60 * ((b - r) / diff) + 120, 360);

  // if cmax equal b then compute h
  else if (cmax == b)
    h = fmod(60 * ((r - g) / diff) + 240, 360);
  Serial.print("H= ");
  Serial.print(h);
  Serial.print("    ");
  Serial.println();

  delay(50);
}