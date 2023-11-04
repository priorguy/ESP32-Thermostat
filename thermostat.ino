#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// GPIO pins for DS18B20 sensors and relays
const int sensorPins[4] = {2, 4, 5, 16};  // Example pins
const int relayPins[4] = {25, 26, 27, 14};  // Example pins

//Virtual pins for Blynk
const int TEMP_VPINS[4] = {V0, V1, V2, V3};
const int SETPOINT_VPINS[4] = {V4, V5, V6, V7};

DallasTemperature sensors[4];
OneWire oneWire[4];

// Blynk authentication and server settings
char auth[] = "YourAuthToken";
char server[] = "YourServerIP";
int port = YourServerPort;

// Temperature setpoints
float setpoints[4] = {20.0, 20.0, 20.0, 20.0};  // Example setpoints

float hysteresis = 0.5;  // Default hysteresis value of 0.5°C

// Additional Global Variables
const int numCycles = 10;
const unsigned long cooldownTime = 2 * 60 * 1000;  // 2 minutes in milliseconds
unsigned long lastOffTimes[4] = {0};  // Time when heating was last turned off for each room

float offset[4] = {0};  // Offset to adjust when the pump is turned on/off
float offsetValues[4][10] = {0};  // Array to store the last 10 offset values for each room
int offsetIndex[4] = {0};  // Index to keep track of the current offset value for each room
const unsigned long reconnectInterval = 30 * 1000;  // 30 seconds
unsigned long lastReconnectAttempt = 0;\

// Static assert to ensure numCycles matches the size of offsetValues array
static_assert(sizeof(offsetValues[0]) / sizeof(offsetValues[0][0]) == numCycles, "numCycles does not match the size of offsetValues array");

void setup() {
  Serial.begin(115200);
  
  for (int i = 0; i < 4; i++) {
    oneWire[i] = OneWire(sensorPins[i]);
    sensors[i] = DallasTemperature(&oneWire[i]);
    sensors[i].begin();
    pinMode(relayPins[i], OUTPUT);
  }
  
  WiFi.begin("YourSSID", "YourPASSWORD");
  Blynk.config(auth, server, port);

  Blynk.onConnect([]() {
    Blynk.virtualWrite(V9, hysteresis);  // Update the display whenever Blynk connects
  });
}

void loop() {
    // Check WiFi Connection
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi disconnected, attempting to reconnect...");
        WiFi.begin("YourSSID", "YourPASSWORD");
        return;  // Skip the rest of the loop if WiFi is not connected
    }

    // Run Blynk if connected
    if (Blynk.connected()) {
        Blynk.run();
    } else if (millis() - lastReconnectAttempt > reconnectInterval) {  // Attempt reconnection every 30 seconds
        lastReconnectAttempt = millis();
        Blynk.connect();
    }

    // Process temperatures, control relays, and update setpoints
    for (int i = 0; i < 4; i++) {
        sensors[i].requestTemperatures();  // Request temperatures from sensors
        
        float temp = sensors[i].getTempCByIndex(0);  // Get temperature reading
        if (temp != DEVICE_DISCONNECTED_C) {
            controlRelay(i, temp);  // Control relay based on temperature
            updateSetpoints(i);  // Update offset values after controlling the relay
            
            // Update Blynk with current temperature if connected
            if (Blynk.connected()) {
                Blynk.virtualWrite(TEMP_VPINS[i], temp);  
            }
        } else {
            Serial.println("Error: Could not read temperature for sensor " + String(i));
        }
    }
}


void controlRelay(int room, float temp) {
  int relayPin = relayPins[room];
  unsigned long currentTime = millis();

  // Use setpoints[room] - hysteresis - offset[room] and setpoints[room] + hysteresis + offset[room] as the thresholds
  if (temp < setpoints[room] - hysteresis - offset[room] && currentTime - lastOffTimes[room] > cooldownTime) {
    digitalWrite(relayPin, HIGH);  // Turn on heating
  } else if (temp > setpoints[room] + hysteresis + offset[room]) {
    lastOffTimes[room] = currentTime;  // Record the time heating stops
    digitalWrite(relayPin, LOW);  // Turn off heating
  }
}

void updateSetpoints(int room) {
  float temp = sensors[room].getTempCByIndex(0);  // Current temperature
  float currentOffset = 0;

  // Determine the current offset based on the relay state and temperature
  bool isHeatingOn = digitalRead(relayPins[room]) == HIGH;
  if (isHeatingOn && temp > setpoints[room] + hysteresis) {
    currentOffset = temp - (setpoints[room] + hysteresis / 2);
  } else if (!isHeatingOn && temp < setpoints[room] - hysteresis) {
    currentOffset = (setpoints[room] - hysteresis / 2) - temp;
  }

  // Update the offset values array and index for the room
  offsetValues[room][offsetIndex[room]] = currentOffset;
  offsetIndex[room] = (offsetIndex[room] + 1) % numCycles;  // Assuming numCycles is the size of offsetValues array

  // Calculate the average offset
  float sum = 0;
  float validOffsetsCount = 0;
  for (int i = 0; i < numCycles; i++) {
    sum += offsetValues[room][i];
  }
  float average = sum / numCycles;

  // Adjust the average based on deviation
  sum = 0; // Reset sum to use for adjusted average calculation
  for (int i = 0; i < numCycles; i++) {
    // Include only values within 20% deviation from the average
    if (abs(offsetValues[room][i] - average) < 0.2 * average) {
      sum += offsetValues[room][i];
      validOffsetsCount++;
    }
  }

  // Update the offset for the room if there are valid offsets
  if (validOffsetsCount > 0) {
    offset[room] = sum / validOffsetsCount;
  }
}



// BLYNK_WRITE functions to update setpoints when the Slider or Step widgets are changed
BLYNK_WRITE(V4) {
  setpoints[0] = param.asFloat();
}
BLYNK_WRITE(V5) {
  setpoints[1] = param.asFloat();
}
BLYNK_WRITE(V6) {
  setpoints[2] = param.asFloat();
}
BLYNK_WRITE(V7) {
  setpoints[3] = param.asFloat();
}
BLYNK_WRITE(V8) {  // Assuming V8 is the virtual pin assigned for hysteresis control
  hysteresis = param.asFloat();
  Blynk.virtualWrite(V9, hysteresis);  // Update the display whenever hysteresis changes
}
