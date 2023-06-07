#include <Arduino.h>
#include <SD.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define DELAYTIME 100
#define CHIPSELECT 4
#define DATABUFFERSIZE 50

double altitudeBuffer[DATABUFFERSIZE] = {};
double temperatureBuffer[DATABUFFERSIZE] = {};
double pressureBuffer[DATABUFFERSIZE] = {};
double humidityBuffer[DATABUFFERSIZE] = {};
char printfBuffer[50];
const int START_BUTTON = 7;
const int END_BUTTON = 3;

Adafruit_BME280 bme; // I2C

void setup() {
    Serial.begin(9600);
    pinMode(START_BUTTON, INPUT_PULLUP);
    while(!Serial);
    while(digitalRead(START_BUTTON) == HIGH);    // time to get serial running and button pressed

    Serial.println("Initializing BME280...");

    unsigned status;
    // Initializes the BME280
    status = bme.begin();
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        exit(0);
    }

    Serial.println("Initializing SD card...");

    // see if the card is present and can be initialized:
    if (!SD.begin(CHIPSELECT)) {
        Serial.println("Card failed, or not present");
        // don't do anything more:
        exit(0);
    }
    Serial.println("Successfully Initialized");
    File dataFile = SD.open("datalog.csv", FILE_WRITE);
    if (dataFile) {
        dataFile.println("Altitude (m), Temperature (C), Pressure (kpa), Humidity (%)");
        dataFile.close();
        Serial.println("Header Created");
    }
    else {
        Serial.println("error opening datalog.csv");
        exit(0);
    }
    delay(1000);
}

void writeToSD() {
    File dataFile = SD.open("datalog.csv", FILE_WRITE);
    if (dataFile) {
        for (int i = 0; i < DATABUFFERSIZE; i++) {
            sprintf(printfBuffer, "%.2f, %.2f, %.2f, %.2f", altitudeBuffer[i], temperatureBuffer[i], pressureBuffer[i], humidityBuffer[i]);
            dataFile.println(printfBuffer);
        }
        dataFile.close();
        Serial.println("Write Successful");
    }
    else {
        Serial.println("error opening datalog.csv");
    }
}

void writeToDataBuffer() {
    int index = 0;
    for (int i = 0; i < DATABUFFERSIZE; i++) {
        altitudeBuffer[i] = bme.readAltitude(SEALEVELPRESSURE_HPA);
        temperatureBuffer[i] = bme.readTemperature();
        pressureBuffer[i] = bme.readPressure() / 1000.0F;
        humidityBuffer[i] = bme.readHumidity();
        sprintf(printfBuffer, "%d: %.2f m, %.2f C, %.2f kpa, %.2f %%", index, altitudeBuffer[i], temperatureBuffer[i], pressureBuffer[i], humidityBuffer[i]);
        Serial.println(printfBuffer);
        index++;

        delay(DELAYTIME);
    }
}

void loop() {
    writeToDataBuffer();
    writeToSD();
    if (digitalRead(END_BUTTON) == LOW) {
        exit(0);
    }
}