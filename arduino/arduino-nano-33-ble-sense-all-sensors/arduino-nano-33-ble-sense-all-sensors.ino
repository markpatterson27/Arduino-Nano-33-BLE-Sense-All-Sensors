/**
 * Sketch to check that all sensors on Arduino Nano 33 BLE Sense are working
 **/

#include <Arduino_LSM9DS1.h>    // library for 9-axis IMU
#include <Arduino_LPS22HB.h>    // library to read pressure 
#include <Arduino_HTS221.h>     // library to read temperature and humidity 
#include <Arduino_APDS9960.h>   // library for colour, proximity and gesture recognition

// declare variables to hold sensor data
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float mag_x, mag_y, mag_z;
float pressure;
float temperature, humidity;
int red, green, blue, ambient;
int proximity;
int gesture;


void setup() {
    Serial.begin(9600); // serial monitor

    // initialize LED pins as as outputs.
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);

    if (!IMU.begin()) { // initialize IMU sensor
        Serial.println("Failed to initialize IMU!"); while (1);
    }

    if (!BARO.begin()) { // initialize Pressure sensor
        Serial.println("Failed to initialize Pressure Sensor!"); while (1);
    }

    if (!HTS.begin()) { // initialize Temperature and Humidity sensor
        Serial.println("Failed to initialize Temperature and Humidity Sensor!"); while (1);
    }

    if (!APDS.begin()) { // initialize Colour, Proximity and Gesture sensor
        Serial.println("Failed to initialize Colour, Proximity and Gesture Sensor!"); while (1);
    }
}


void loop() {
    // Flash LEDs
    // for the RGB LED, high is off
    digitalWrite(LED_BUILTIN, HIGH);    // turn on BUILTIN
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);     // turn off BUILTIN
    digitalWrite(LEDR, LOW);           // turn on red LED
    delay(250);
    digitalWrite(LEDR, HIGH);           // turn off red LED
    digitalWrite(LEDG, LOW);           // turn on green LED
    delay(250);
    digitalWrite(LEDG, HIGH);           // turn off green LED
    digitalWrite(LEDB, LOW);           // turn on blue LED
    delay(250);
    digitalWrite(LEDB, HIGH);           // turn off blue LED


    // Read IMU values
    if (IMU.accelerationAvailable()) {  // accelerometer values
        IMU.readAcceleration(accel_x, accel_y, accel_z);
        Serial.print("Accelerometer = ");Serial.print(accel_x);Serial.print(", ");Serial.print(accel_y);Serial.print(", ");Serial.println(accel_z);
    }
    if (IMU.gyroscopeAvailable()) { // gyroscope values 
        IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
        Serial.print("Gyroscope = ");Serial.print(gyro_x);Serial.print(", ");Serial.print(gyro_y);Serial.print(", ");Serial.println(gyro_z);
    }
    if (IMU.magneticFieldAvailable()) { // magnetometer values
        IMU.readMagneticField(mag_x, mag_y, mag_z);
        Serial.print("Magnetometer = ");Serial.print(mag_x);Serial.print(", ");Serial.print(mag_y);Serial.print(", ");Serial.println(mag_z);
    }
    delay (200);

    // Read Pressure sensor value
    pressure = BARO.readPressure();
    Serial.print("Pressure = ");Serial.println(pressure);
    delay (200);

    // Read Temperature and Humidity sensor values
    temperature = HTS.readTemperature();    // read temperature value
    Serial.print("Temperature = ");Serial.println(temperature);
    humidity = HTS.readHumidity();  // read humidity value
    Serial.print("Humidity = ");Serial.println(humidity);
    delay (200);

    // Read Gesture, Color, Proximity sensor values
    while (!APDS.colorAvailable()) {   // wait for color reading
        delay(5);
    }
    APDS.readColor(red, green, blue, ambient);    // read the color
    Serial.print("Colors = ");
        Serial.print("R: ");Serial.print(red);Serial.print(", ");
        Serial.print("G: ");Serial.print(green);Serial.print(", ");
        Serial.print("B: ");Serial.print(blue);Serial.print(", ");
        Serial.print("A: ");Serial.print(ambient);Serial.println();

    if (APDS.proximityAvailable()) {    // read proximity
        proximity = APDS.readProximity();
        Serial.print("Proximity = ");Serial.println(proximity); 
    }

    // ** Gesture and color reading don't seem to be able to both work at same time **

    // while (!APDS.gestureAvailable()){   // wait for gesture reading
    //     delay(5);
    // }
    // gesture = APDS.readGesture();   // read gesture
    // Serial.print("Gesture = ");
    // switch (gesture) {
    //     case GESTURE_UP: 
    //         Serial.println("UP");break;
    //     case GESTURE_DOWN: 
    //         Serial.println("DOWN");break;
    //     case GESTURE_LEFT: 
    //         Serial.println("LEFT"); break;
    //     case GESTURE_RIGHT: 
    //         Serial.println("RIGHT");break;
    //     default: // ignore
    //         break;
    // }
    delay (200);

    Serial.println("_____________________________________________________"); 
    delay(1000);
}
