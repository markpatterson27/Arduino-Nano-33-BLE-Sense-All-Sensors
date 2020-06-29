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
int proximity;


void setup() {
    Serial.begin(9600); // serial monitor

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

    // Read Proximity sensor value
    if (APDS.proximityAvailable()) {
        proximity = APDS.readProximity();
        Serial.print("Proximity = ");Serial.println(proximity); 
    }
    delay (200);

    Serial.println("_____________________________________________________"); 
    delay(1000);
}
