// INCLUDES & DEFINES
  #include <LcdBarGraph.h>
  #include <WiFi.h>
  #include <PubSubClient.h>
  #include <LiquidCrystal.h>
  #include <ArduinoJson.h>
  #include <EEPROM.h>

  // EEPROM-geheugenadres definities
  #define EEPROM_SIZE 512     // Grootte van de EEPROM
  #define EEPROM_ADDR_STEP 0   // Adres voor het aantal stappen per dosering
  #define EEPROM_ADDR_DAY 20   // Startadres voor de dagstring
  #define EEPROM_ADDR_TIME 40  // Startadres voor de tijdstring
  #define EEPROM_ADDR_LAST_FEEDING 60   // Adres voor laatste voerbeurt
  #define EEPROM_ADDR_FEED_TIMES 100    // Startadres voor de voertijden
// PIN CONFIGURATIE & CONSTANTEN
  LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
  LcdBarGraph lbg(&lcd, 10, 0, 1);

  // Ultrasone sensor voor het reservoir
  const int trigPin = 7; 
  const int echoPin = 6; 
  long duration;         
  int distance;          
  const int maxDistance = 17; // Maximale afstand (voor leeg reservoir)
  const int minDistance = 1;  // Minimale afstand (vol reservoir)

  // Motor setup
  const int motorStepPin = 10;    
  const int motorDirPin = 9;
  const int motorSleepPin = 8;    
  const int motorEnablePin = 13;  
  int stepsPerDosis = 800;        
  bool motorRunning = false;      

  const char* ssid = "";        
  const char* password = "m";

  // WiFi-configuratie
  //const char* ssid = "";        
  //const char* password = "";
  //IPAddress local_IP(192, 168, 1, 200); 
  //IPAddress gateway(192, 168, 1, 1);    
  //IPAddress subnet(255, 255, 255, 0);   
  
  // Variabele om de laatst gemeten tijd voor WiFi-herverbinding bij te houden
  unsigned long lastWifiAttempt = 0;
  const unsigned long WIFI_RETRY_INTERVAL = 10000; // elke 10 seconden opnieuw proberen


  // MQTT-configuratie
  //const char* mqtt_server = "192.168.1.250";     
  const char* mqtt_server = "";
  const int mqtt_port = 1883;                    
  const char* mqtt_user = "";        
  const char* mqtt_password = "";
  WiFiClient espClient;
  PubSubClient client(espClient);

  // MQTT-topics
  const char* motor_topic = "/kattenvoerbak/motor";         
  const char* status_topic = "/kattenvoerbak/status";       
  const char* dosing_topic = "/kattenvoerbak/dosage";       
  const char* reservoir_topic = "/kattenvoerbak/reservoir"; 
  const char* day_topic = "/kattenvoerbak/day";             
  const char* time_topic = "/kattenvoerbak/time";           
  const char* last_feed_topic = "/kattenvoerbak/last_feed"; 
  const char* feed_time_topic = "/kattenvoerbak/feeding_times";

  // Voerplanning en laatste voerbeurt
  String feedingTimes[3] = {"", "", ""};  
  String lastFeedingTime = "";

  // Variabelen voor statusscherm op LCD
  bool showStatusMessage = false;
  unsigned long statusMessageStartTime = 0;
  const unsigned long statusDisplayDuration = 5000UL;
  String lastStatusMessage = "";

  // Datum en tijd
  String dayString = "";  
  String timeString = "";

  // Variabelen voor weergavestanden en timing
  bool feedingTriggered[3] = {false, false, false};
  unsigned long lastStateSwitchTime = 0;
  const unsigned long STATE_SWITCH_INTERVAL = 10000;
  const unsigned long ULTRASONIC_UPDATE_INTERVAL = 10000;
  const unsigned long STATUS_PUBLISH_INTERVAL = 120000;
  const unsigned long MOTOR_MAX_RUNTIME = 12000;
  const unsigned long FLUSH_RUNTIME = 15000;
  int currentState = 0;

// SETUP
  void setup() {
    pinMode(motorEnablePin, OUTPUT);
    digitalWrite(motorEnablePin, HIGH);
    pinMode(motorStepPin, OUTPUT);
    pinMode(motorDirPin, OUTPUT);
    digitalWrite(motorStepPin, LOW);
    digitalWrite(motorDirPin, LOW);

    Serial.begin(115200);

    // Lees eerder opgeslagen waarden uit EEPROM
    readEEPROMValues(); 

    // Alleen een eerste poging om WiFi te verbinden
    setup_wifi();

    // Stel MQTT in
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // LCD initialiseren
    lcd.begin(16, 2);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Synchroniseren!");
    delay(2000); 

    // Als WiFi al verbonden is, publiceer direct de data, anders straks als WiFi beschikbaar komt.
    if (WiFi.status() == WL_CONNECTED && client.connected()) {
      client.publish(last_feed_topic, lastFeedingTime.c_str(), true);
      String dosingMessage = getDosingLevel(stepsPerDosis);
      client.publish(dosing_topic, dosingMessage.c_str(), true);
      sendStatusMessage("Kattenvoerbak is online");
    }
  }


// WIFI SETUP
  void setup_wifi() {
    Serial.println("Verbinding maken met WiFi...");
    WiFi.begin(ssid, password);

    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 5000) { 
      // Probeer max 5 seconden te verbinden in setup()
      delay(500);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi is verbonden!");
      Serial.print("IP-adres: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("\nWiFi niet verbonden tijdens setup. Zal later opnieuw proberen.");
    }
  }
// EEPROM FUNCTIES
  void writeStringToEEPROM(int addr, const String &data, int maxLen) {
      int len = data.length();
      if (len > maxLen - 1) {
          len = maxLen - 1; 
      }
      for (int i = 0; i < len; i++) {
          EEPROM.write(addr + i, data[i]);
      }
      EEPROM.write(addr + len, '\0');
    }
  String readStringFromEEPROM(int addr, int maxLen) {
      char buffer[100];
      if (maxLen > (int)sizeof(buffer)) maxLen = sizeof(buffer) - 1;
      for (int i = 0; i < maxLen; i++) {
          buffer[i] = EEPROM.read(addr + i);
          if (buffer[i] == '\0') {
              break;
          }
          if (i == maxLen - 1) buffer[i] = '\0';
      }
      return String(buffer);
  }
  void readEEPROMValues() {
      EEPROM.get(EEPROM_ADDR_STEP, stepsPerDosis);
      if (stepsPerDosis <= 0 || stepsPerDosis > 1000) {
          stepsPerDosis = 800; 
      }
      Serial.print("StepsPerDosis gelezen uit EEPROM: ");
      Serial.println(stepsPerDosis);

      lastFeedingTime = readStringFromEEPROM(EEPROM_ADDR_LAST_FEEDING, 20);
      Serial.print("Laatste voertijd: ");
      Serial.println(lastFeedingTime);

      // Voeg deze regel toe om voertijden uit de EEPROM te lezen
      readFeedingTimesFromEEPROM();
      Serial.println("Voertijden uit EEPROM gelezen:");
      for (int i = 0; i < 3; i++) {
          Serial.println(feedingTimes[i]);
      }
  }

  void saveStepsToEEPROM(int steps) {
      EEPROM.put(EEPROM_ADDR_STEP, steps);
      Serial.print("StepsPerDosis opgeslagen in EEPROM: ");
      Serial.println(steps);
  }
  void saveDayTimeToEEPROM() {
      writeStringToEEPROM(EEPROM_ADDR_DAY, dayString, 20);
      writeStringToEEPROM(EEPROM_ADDR_TIME, timeString, 20);
  }
  void saveFeedingTimesToEEPROM() {
      for (int i = 0; i < 3; i++) {
          writeStringToEEPROM(EEPROM_ADDR_FEED_TIMES + i * 20, feedingTimes[i], 20);
      }
  }

  void readFeedingTimesFromEEPROM() {
      for (int i = 0; i < 3; i++) {
          feedingTimes[i] = readStringFromEEPROM(EEPROM_ADDR_FEED_TIMES + i * 20, 20);
      }
  }

  void saveLastFeedingTimeToEEPROM() {
      writeStringToEEPROM(EEPROM_ADDR_LAST_FEEDING, lastFeedingTime, 20);
  }

  void readLastFeedingTimeFromEEPROM() {
      lastFeedingTime = readStringFromEEPROM(EEPROM_ADDR_LAST_FEEDING, 20);
  }
// VOERPLANNING
  void handleFeedingSchedule() {
      String currentTime = timeString;
      for (int i = 0; i < 3; i++) {
          // Controleer of er een geldige voertijd is ingesteld en deze overeenkomt met de huidige tijd
          if (!feedingTimes[i].isEmpty() && currentTime == feedingTimes[i] && !motorRunning && !feedingTriggered[i]) {
              startMotor();
              lastFeedingTime = currentTime; 
              saveLastFeedingTimeToEEPROM(); 
              client.publish(last_feed_topic, lastFeedingTime.c_str(), true);
              feedingTriggered[i] = true;
              Serial.print("Laatste voerbeurt: ");
              Serial.println(lastFeedingTime);
          } else if (currentTime != feedingTimes[i]) {
              // Als de huidige tijd niet overeenkomt, reset dan de trigger voor deze voertijd
              feedingTriggered[i] = false;
          }
      }
  }

// HELPER FUNCTIES
  String getDosingLevel(int steps) {
      if (steps == 160) return "Laag (10g)";
      else if (steps == 480) return "Gemiddeld (30g)";
      else if (steps == 800) return "Hoog (50g)";
      return "Onbekend";
  }

  void handleDayUpdate(const String &message) {
      dayString = message;
      saveDayTimeToEEPROM();
      Serial.print("Ontvangen dag: ");
      Serial.println(dayString);
  }

  void handleTimeUpdate(const String &message) {
      timeString = message;
      saveDayTimeToEEPROM();
      Serial.print("Ontvangen tijd: ");
      Serial.println(timeString);
  }

  void handleMotorCommand(const String &message) {
      if (message == "Flush" && !motorRunning) {
          flushMotor();
      } else if (message == "Motor gestart" && !motorRunning) {
          startMotor();
      } else if (message == "Motor gestopt" && motorRunning) {
          motorRunning = false;
      }
  }

  void handleDosingCommand(const String &message) {
      if (message.startsWith("Hoog")) {
          stepsPerDosis = 800;
      } else if (message.startsWith("Gemiddeld")) {
          stepsPerDosis = 480;
      } else if (message.startsWith("Laag")) {
          stepsPerDosis = 160;
      } else {
          Serial.println("Onbekend doseringsniveau ontvangen, geen update uitgevoerd.");
          return;
      }

      saveStepsToEEPROM(stepsPerDosis);
      Serial.println("Dosering is bijgewerkt en opgeslagen in EEPROM.");
  }

  void handleFeedTimesUpdate(const String &message) {
      StaticJsonDocument<128> doc;
      DeserializationError error = deserializeJson(doc, message);
      if (error) {
          Serial.println("Fout bij JSON-deserialisatie van voertijden, geen update uitgevoerd.");
          return;
      }

      for (int i = 0; i < 3; i++) {
          feedingTimes[i] = String(doc[i].as<const char*>());
      }
      saveFeedingTimesToEEPROM(); 
      Serial.println("Voertijden bijgewerkt en opgeslagen in EEPROM.");
      for (int a = 0; a < 3; a++) {
          Serial.println(feedingTimes[a]);
      }
  }


// MQTT CALLBACK
  void callback(char* topic, byte* payload, unsigned int length) {
      String message;
      for (unsigned int i = 0; i < length; i++) {
          message += (char)payload[i];
      }

      String topicStr = String(topic);

      if (topicStr == day_topic) {
          handleDayUpdate(message);
          return; 
      } 
      if (topicStr == time_topic) {
          handleTimeUpdate(message);
          return; 
      }
      if (topicStr == motor_topic) {
          handleMotorCommand(message);
      } else if (topicStr == dosing_topic) {
          handleDosingCommand(message);
      } else if (topicStr == feed_time_topic) {
          handleFeedTimesUpdate(message);
      }
      // Toon statusbericht op het LCD ongeacht het topic behalve bij day/time
      showStatusOnLCD(message);
  }

// STATUS & RECONNECT FUNCTIES
  void showStatusOnLCD(const String &status) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Status:");
      lcd.setCursor(0, 1);
      lcd.print(status.substring(0, 16));
  }

  void reconnect() {
    if (WiFi.status() != WL_CONNECTED) return; // Geen WiFi, dan ook niet proberen te verbinden met MQTT
    while (!client.connected() && WiFi.status() == WL_CONNECTED) {
      Serial.println("Verbinding maken met MQTT...");
      if (client.connect("Arduino_Client", mqtt_user, mqtt_password)) {
        client.subscribe(motor_topic);
        client.subscribe(status_topic);
        client.subscribe(dosing_topic);
        client.subscribe(day_topic);
        client.subscribe(time_topic);
        client.subscribe(feed_time_topic);
        client.subscribe(last_feed_topic);
      } else {
        Serial.print("Verbinding mislukt. Status: ");
        Serial.println(client.state());
        delay(5000);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("MQTT Verbinden:");
        lcd.setCursor(0, 1);
        lcd.print("mislukt");
        delay(5000);
        return;
      }
    }
  }

  void sendStatusMessage(const char* message) {
      if (client.connected()) {
          client.publish(status_topic, message);
          Serial.println("Statusbericht verzonden!");
      } else {
          Serial.println("Kan bericht niet verzenden: MQTT niet verbonden.");
      }
  }

// MOTOR FUNCTIES
  void startMotor() {
      if (!motorRunning) {
          motorRunning = true; 
          unsigned long startTime = millis(); 
          bool motorSuccess = true; 

          digitalWrite(motorEnablePin, LOW); 
          digitalWrite(motorSleepPin, HIGH); 
          delay(10);
          digitalWrite(motorDirPin, LOW); 
          delay(10);

          showStatusOnLCD("Motor gestart");

          for (int i = 0; i < stepsPerDosis; i++) {
              if (!motorRunning || (millis() - startTime > MOTOR_MAX_RUNTIME)) {
                  motorSuccess = false;
                  break;
              }
              digitalWrite(motorStepPin, HIGH);
              delayMicroseconds(2000);
              digitalWrite(motorStepPin, LOW);
              delayMicroseconds(2000);
          }

          motorRunning = false;
          digitalWrite(motorSleepPin, LOW);
          digitalWrite(motorEnablePin, HIGH);

          String currentTime = timeString;
          lastFeedingTime = currentTime;
          saveLastFeedingTimeToEEPROM();
          client.publish(last_feed_topic, lastFeedingTime.c_str(), true);

          if (motorSuccess) {
              client.publish(motor_topic, "Motor gestopt");
          } else {
              client.publish(motor_topic, "Motor fout");
          }
      }
  }

  void flushMotor() {
      if (!motorRunning) {
          motorRunning = true;
          unsigned long startTime = millis();

          digitalWrite(motorEnablePin, LOW);
          digitalWrite(motorSleepPin, HIGH);
          delay(10);
          digitalWrite(motorDirPin, LOW);
          delay(10);

          while (millis() - startTime < FLUSH_RUNTIME) {
              digitalWrite(motorStepPin, HIGH);
              delayMicroseconds(3000);
              digitalWrite(motorStepPin, LOW);
              delayMicroseconds(3000);
          }

          motorRunning = false;
          digitalWrite(motorSleepPin, LOW);
          digitalWrite(motorEnablePin, HIGH);
          client.publish(motor_topic, "Flush voltooid");
      }
  }

// ULTRASOON FUNCTIE
  void updateUltrasonicAndBargraph() {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);

      duration = pulseIn(echoPin, HIGH); 
      distance = duration * 0.0343 / 2;  

      int percentage = map(distance, maxDistance, minDistance, 0, 100);
      percentage = constrain(percentage, 0, 100);

      String message = String(percentage);
      client.publish(reservoir_topic, message.c_str());

      Serial.print("Afstand: ");
      Serial.print(distance);
      Serial.print(" cm, Reservoir: ");
      Serial.print(percentage);
      Serial.println("% gevuld");

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Reservoir niveau");
      lcd.setCursor(0, 1);
      lbg.drawValue(percentage, 100);
      lcd.setCursor(11, 1);
      lcd.print(String(percentage) + "%");
  }


// LOOP
  void loop() {
    // Als nog geen WiFi verbinding, probeer regelmatig opnieuw
    if (WiFi.status() != WL_CONNECTED) {
      // Laat op LCD zien dat WiFi niet verbonden is
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("WiFi niet");
      lcd.setCursor(0,1);
      lcd.print("verbonden!");

      // Check of het tijd is voor een nieuwe poging
      if (millis() - lastWifiAttempt >= WIFI_RETRY_INTERVAL) {
        lastWifiAttempt = millis();
        Serial.println("Opnieuw proberen met WiFi...");
        WiFi.disconnect();
        WiFi.begin(ssid, password);
      }
    } else {
      // Als WiFi verbonden is, check MQTT
      if (!client.connected()) {
        reconnect(); 
      }
      client.loop();
    }

    // Het voedingsschema moet gewoon blijven draaien ongeacht de WiFi-status
    handleFeedingSchedule();
    unsigned long now = millis();

    // Wissel tussen weergavestanden (datum/tijd en reservoir)
    if (now - lastStateSwitchTime > STATE_SWITCH_INTERVAL) {
      currentState = (currentState == 0) ? 1 : 0; 
      lastStateSwitchTime = now;
    }

    if (currentState == 0) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(dayString.substring(0, 16));
      lcd.setCursor(0, 1);
      lcd.print(timeString.substring(0, 16));
    } else {
      static unsigned long lastUltrasonicUpdate = 0;
      if (now - lastUltrasonicUpdate >= ULTRASONIC_UPDATE_INTERVAL) {
          lastUltrasonicUpdate = now;
          updateUltrasonicAndBargraph();
      }
    }

    static unsigned long lastPublish = 120000;
    if ((millis() - lastPublish > STATUS_PUBLISH_INTERVAL) && WiFi.status() == WL_CONNECTED && client.connected()) {
      sendStatusMessage("Feeder werkt!");
      lastPublish = millis();
    }
  }
