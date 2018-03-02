// WiFi and OTA libraries
#include <WiFi.h> // ESP32 WiFi drivers
#include <ESPmDNS.h> // ESP32 mDNS support
#include <WiFiUdp.h> // ESP32 WiFi UDP
#include <ArduinoOTA.h> // Arduino OTA updates
#include <Update.h> // Needed for OTA

// PID includes
#include <PID_v1.h> // PID library

// Identify which core has the Arduino stuff
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

// Function declarations. Definitions are at the bottom for readability
// Inlined to encourage compiler optimization
inline void start_WiFi();
inline void start_OTA();
void WiFi_loop(void * parameter);


// Global variables
// Set ssid and password here
const char* ssid = "NASA RMC Test Network";
const char* password = "StateSpaceRobotics";
const char* hostname = "esp32_OTA_PID_Example";

// PID variables
const int EN1 = 18, EN2 = 5;
const int PWM_PIN = 19;
int freq = 20000; // max frequency is 80,000,000 / 2^resolution
int mot_channel = 0;
int resolution = 8;
double cmd_vel=0, velocity=0, pwm=0;
double Ki=9, Kp=15, Kd=0;
PID PID_ex(&velocity, &pwm, &cmd_vel, Kp, Ki, Kd, DIRECT);
// PID function declarations
void setup_PID();
void update_vel();
void do_PID();

// Encoder variables
const uint32_t COUNTS_PER_MOTOR_REV = 12; // 12 paddles on the encoder
const uint32_t EVENTS_PER_MOTOR_REV = COUNTS_PER_MOTOR_REV*4; // 4 logic changes
const uint32_t EVENTS_PER_GEARBOX_REV = EVENTS_PER_MOTOR_REV*64; // Gearing
portMUX_TYPE enc_mux = portMUX_INITIALIZER_UNLOCKED;
#define A_PIN 34
#define B_PIN 35
volatile int32_t enc_count=0;
// Encoder ISR Declarations
void A_int();
void B_int();

// UDP Echo Client variables
const int udpEchoPort = 3333;
WiFiUDP udp_echo;
// UDP Declarations
void do_UDP_echo();

// UDP Setpoint Client variables
const int udpSetPointPort = 3334;
WiFiUDP udp_setpoint;
void do_UDP_setpoint();

// UDP Kp Client variables
const int udpKpPort = 3335;
WiFiUDP udp_kp;
void do_UDP_kp();

// UDP Kp Client variables
const int udpKiPort = 3336;
WiFiUDP udp_ki;
void do_UDP_ki();

// UDP Kp Client variables
const int udpKdPort = 3337;
WiFiUDP udp_kd;
void do_UDP_kd();

void setup() {
  // Serial Debug output
  Serial.begin(115200);
  Serial.println("Booting");
  // Start wifi
  start_WiFi();
  // Start OTA server
  start_OTA();

  // Spin up WiFi/etc. loop on Core 0
  xTaskCreatePinnedToCore(
    WiFi_loop, // Task function
    "WiFi_loop", // Task name
    10000, // Stack size for task (in words)
    NULL,  // Parameter passed as input
    2,     // Task priority
    NULL,  // Task handle
    0 // Run on Core 0
  );

  // Setup PID
  setup_PID();

  // UDP echo setup
  udp_echo.begin(udpEchoPort);
  udp_echo.flush();

  // UDP setpoint
  udp_setpoint.begin(udpSetPointPort);
  udp_setpoint.flush();

  // UDP Kp,Ki,Kd
  udp_kp.begin(udpKpPort);
  udp_kp.flush();
  udp_ki.begin(udpKiPort);
  udp_ki.flush();
  udp_kd.begin(udpKdPort);
  udp_kd.flush();
}

// WiFi and high level async task loop
// This loop shares Core 0 with the WiFi handling and contains all processes
//    which are not time-critical, leaving Core 1 for synchronous tasks
//    with strict timing requirements.
void WiFi_loop(void * parameter) {
  while (1){
    // Handles OTA updates
    ArduinoOTA.handle();

    // UDP echo stuff
    do_UDP_echo();

    // UDP setpoint
    do_UDP_setpoint();

    // UDP Kp,Ki,Kd
    do_UDP_kp();
    do_UDP_ki();
    do_UDP_kd();
    //Serial.print("WiFi_loop: "); Serial.println(xPortGetCoreID());
    yield(); // yield to let the WiFi drivers do their thing
    vTaskDelay(500/portTICK_PERIOD_MS);

  }
}

// Main loop
// The ESP32 has two cores (0 and 1). WiFi and other non-timing-critical
//    tasks should be done on core 0. Core 1 is dedicated to PID and timing
//    sensitive tasks.
// The Arduino loop always runs on Core 1
void loop() {
  static uint32_t start;
  start = millis();
  //Serial.print("main_loop: ");
  //Serial.println(xPortGetCoreID());
  //vTaskDelay(500/portTICK_PERIOD_MS);
  do_PID();
  //Serial.print("Vel: "); Serial.println(velocity);
  //Serial.print("Enc: "); Serial.println(enc_count);
  while (millis()-start < 100){
    delay(1);
  }
}

void start_WiFi() {
  // Start up wifi and connect
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Restart until successful connect
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  // Print the IP
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("BAM!!");
}

void start_OTA() {
  // Port for OTA server
  ArduinoOTA.setPort(3232);

  // Hostname for mDNS resolving
  ArduinoOTA.setHostname(hostname);

  // Password for performing OTA update
  ArduinoOTA.setPassword("OTApass");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  // Setup handler functions
  ArduinoOTA
    // Runs this function on start
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    // Runs this function on end
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    // Runs this function during update
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    // Runs this if there's an error
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  // Starts the OTA server
  ArduinoOTA.begin();

}

void do_UDP_echo(){
  static char buf[1024] = {'\0'};
  static int read_status = 0;
  int packet_size = udp_echo.parsePacket();
  // while stuff is in the receive buffer
  if (packet_size >= 1024){
    read_status = udp_echo.read(buf, 1024);
    Serial.println("Too big!");
    Serial.print(buf);
    read_status = udp_echo.read(buf, 1024);
    Serial.println(buf);
  } else if (packet_size) {
    read_status = udp_echo.read(buf, 1024);
    buf[packet_size] = '\0';
    Serial.print("Got: \""); Serial.print(buf); Serial.println("\"");
    // Echo back as ack
    udp_echo.beginPacket(udp_echo.remoteIP(), udp_echo.remotePort());
    udp_echo.printf(buf);
    udp_echo.endPacket();
  }
  udp_echo.flush();
}

void do_UDP_setpoint(){
  static char buf[1024] = {'\0'};
  static int read_status = 0;
  int packet_size = udp_setpoint.parsePacket();
  // while stuff is in the receive buffer
  if (packet_size >= 1024){
    read_status = udp_setpoint.read(buf, 1024);
    Serial.println("Too big!");
    Serial.print(buf);
    read_status = udp_setpoint.read(buf, 1024);
    Serial.println(buf);
  } else if (packet_size) {
    read_status = udp_setpoint.read(buf, 1024);
    buf[packet_size] = '\0';
    Serial.print("Got: \""); Serial.print(buf); Serial.println("\"");
    // Echo back as ack
    udp_setpoint.beginPacket(udp_setpoint.remoteIP(), udp_setpoint.remotePort());
    udp_setpoint.printf(buf);
    udp_setpoint.endPacket();
    cmd_vel = atof(buf);
  }
  udp_setpoint.flush();
}

void do_UDP_kp(){
  static char buf[1024] = {'\0'};
  static int read_status = 0;
  int packet_size = udp_kp.parsePacket();
  // while stuff is in the receive buffer
  if (packet_size >= 1024){
    read_status = udp_kp.read(buf, 1024);
    Serial.println("Too big!");
    Serial.print(buf);
    read_status = udp_kp.read(buf, 1024);
    Serial.println(buf);
  } else if (packet_size) {
    read_status = udp_kp.read(buf, 1024);
    buf[packet_size] = '\0';
    Serial.print("Got: \""); Serial.print(buf); Serial.println("\"");
    // Echo back as ack
    udp_kp.beginPacket(udp_kp.remoteIP(), udp_kp.remotePort());
    udp_kp.printf(buf);
    udp_kp.endPacket();
    Kp = atof(buf);
    PID_ex.SetTunings(Kp,Ki,Kd);
  }
  udp_kp.flush();
}

void do_UDP_ki(){
  static char buf[1024] = {'\0'};
  static int read_status = 0;
  int packet_size = udp_ki.parsePacket();
  // while stuff is in the receive buffer
  if (packet_size >= 1024){
    read_status = udp_ki.read(buf, 1024);
    Serial.println("Too big!");
    Serial.print(buf);
    read_status = udp_ki.read(buf, 1024);
    Serial.println(buf);
  } else if (packet_size) {
    read_status = udp_ki.read(buf, 1024);
    buf[packet_size] = '\0';
    Serial.print("Got: \""); Serial.print(buf); Serial.println("\"");
    // Echo back as ack
    udp_ki.beginPacket(udp_ki.remoteIP(), udp_ki.remotePort());
    udp_ki.printf(buf);
    udp_ki.endPacket();
    Ki = atof(buf);
    PID_ex.SetTunings(Kp,Ki,Kd);
  }
  udp_ki.flush();
}

void do_UDP_kd(){
  static char buf[1024] = {'\0'};
  static int read_status = 0;
  int packet_size = udp_kd.parsePacket();
  // while stuff is in the receive buffer
  if (packet_size >= 1024){
    read_status = udp_kd.read(buf, 1024);
    Serial.println("Too big!");
    Serial.print(buf);
    read_status = udp_kd.read(buf, 1024);
    Serial.println(buf);
  } else if (packet_size) {
    read_status = udp_kd.read(buf, 1024);
    buf[packet_size] = '\0';
    Serial.print("Got: \""); Serial.print(buf); Serial.println("\"");
    // Echo back as ack
    udp_kd.beginPacket(udp_kd.remoteIP(), udp_kd.remotePort());
    udp_kd.printf(buf);
    udp_kd.endPacket();
    Kd = atof(buf);
    PID_ex.SetTunings(Kp,Ki,Kd);
  }
  udp_ki.flush();
}

void setup_PID() {
  // Interrupt pins
  pinMode(A_PIN, INPUT);
  pinMode(B_PIN, INPUT);
  // Direction pins
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);

  // Encoder interrupts
  attachInterrupt(A_PIN, A_int, CHANGE);
  //attachInterrupt(B_PIN, B_int, CHANGE);

  // Initialize PWM Pin
  ledcSetup(mot_channel, freq, resolution);
  ledcAttachPin(PWM_PIN, mot_channel);

  PID_ex.SetMode(AUTOMATIC);
  PID_ex.SetOutputLimits(-255, 255);
  cmd_vel = 0;
}

void do_PID(){
  update_vel();

  PID_ex.Compute();

  // Forward or reverse
  if (pwm > 0){
    // Forward
    // Write to direction pins
    digitalWrite(EN1, HIGH);
    digitalWrite(EN2, LOW);
  }else {
    // Reverse
    // Write to direction pins
    digitalWrite(EN1, LOW);
    digitalWrite(EN2, HIGH);

  }
  // Write PWM
  ledcWrite(mot_channel, (uint8_t) round(abs(pwm)));

}

// Update motor velocity
void update_vel(){
  static uint32_t prev_time = millis()-1;
  static uint32_t this_time;

  static int32_t prev_count = 0;
  static int32_t this_count;

  static double vel_avg[5] = {0};
  static uint32_t i = 0;

  this_time = millis();

  portENTER_CRITICAL(&enc_mux);
  this_count = enc_count;
  portEXIT_CRITICAL(&enc_mux);

  // rad/s = events/duration / EVENTS_PER_GEARBOX_REV * 2*pi
  vel_avg[i] = (1000 * (double) (this_count-prev_count)) / (double) (this_time-prev_time) / EVENTS_PER_GEARBOX_REV * 2 * 3.14 / 5;
  velocity = vel_avg[0] + vel_avg[1] + vel_avg[2] + vel_avg[3] + vel_avg[4];
  i = (i+1)%5;
  // update prev_time to this_time
  prev_time = this_time;
  // Could optionally clear enc_count, but this way allows positioning wrt start
  prev_count = this_count;
}

// Encoder ISR's
// Need special parameters to place function in quick-access RAM
// See: https://techtutorialsx.com/2017/09/30/esp32-arduino-external-interrupts/
void IRAM_ATTR A_int(){
  //enc_count++;

  static uint32_t A_bit, B_bit, xor_bit;
  // Read A channel
  #if A_PIN < 32
    A_bit = (GPIO.in >> A_PIN) & 0x1;
  #else
    A_bit = (GPIO.in1.val >> (A_PIN - 32)) & 0x1;
  #endif
  #if B_PIN < 32
    B_bit = (GPIO.in >> B_PIN) & 0x1;
  #else
    B_bit = (GPIO.in1.val >> (B_PIN - 32)) & 0x1;
  #endif

  xor_bit = A_bit ^ B_bit;

  switch(xor_bit){
    case(1):
      portENTER_CRITICAL_ISR(&enc_mux);
      enc_count++;
      portEXIT_CRITICAL_ISR(&enc_mux);
      break;
    case(0):
      portENTER_CRITICAL_ISR(&enc_mux);
      enc_count--;
      portEXIT_CRITICAL_ISR(&enc_mux);
      break;
  }//*/
}

void IRAM_ATTR B_int(){
  //enc_count++;

  static uint32_t A_bit, B_bit, xor_bit;
  // Read A channel
  #if A_PIN < 32
    A_bit = (GPIO.in >> A_PIN) & 0x1;
  #else
    A_bit = (GPIO.in1.val >> (A_PIN - 32)) & 0x1;
  #endif
  #if B_PIN < 32
    B_bit = (GPIO.in >> B_PIN) & 0x1;
  #else
    B_bit = (GPIO.in1.val >> (B_PIN - 32)) & 0x1;
  #endif

  xor_bit = A_bit ^ B_bit;

  switch(xor_bit){
    case(1):
      portENTER_CRITICAL_ISR(&enc_mux);
      enc_count--;
      portEXIT_CRITICAL_ISR(&enc_mux);
      break;
    case(0):
      portENTER_CRITICAL_ISR(&enc_mux);
      enc_count++;
      portEXIT_CRITICAL_ISR(&enc_mux);
      break;
  }//*/
}
