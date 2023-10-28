#include <Arduino.h>
// Programm zur Auslesung und Entschlüsselung der Kundenschnittstelle eines SmartMeters der EVN (NÖ)
// Bibliothek: https://github.com/rweather/arduinolibs/tree/master/libraries/Crypto
// Aus dem Ordner "libraries" die Bibliothek "Crypto" installieren!
// Prg. funktioniert für einen Atmega2560 (Arduino Mega), Man könnte aber auch die Bibliothek SoftwareSerial benutzen um 
// das Programm auf einem anderen Mikrocontroller zu realisieren.

#include <Crypto.h>
#include <AES.h>
#include <GCM.h>
#include <WiFi.h>
#include <PubSubClient.h>
//#include  <BluetoothSerial.h>
#include <LiquidCrystal_I2C.h>
#include <esp_task_wdt.h>

#define I2C_SDA 21
#define I2C_SCL 22

LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

#define MAX_PLAINTEXT_LEN 300
#define RXD2 16 //Verbunden mit TX am MIKROE-4137; MBUS 1 auf RJ12 3, MBUS 2 auf RJ12 4
#define TXD2 17 ////Verbunden mit RX am MIKROE-4137


//BluetoothSerial SerialBT;

int byteNumber = 0;
unsigned long timeSinceLastData = 0;
bool processData = false;
int durchlauf=0;
int fehlerdurchlauf=0;
int startzeit=0;
int Umdrehungen = 0;
int Umdrehungenreal = 0;
bool flanke = 0;
bool flanke1 = 0;
bool buttonState = 0;
int sensorPin = 34; //Analog input von IR
const int zaehler = 23; //Inpulseingang von IR
bool debug =0; //Seriellen Monitor ein bzw. ausschalten
bool mqtt_send=1; //Senden der Daten ausschalten
int sensorValue = 0; //Anlalog Wert von sensorPin
bool autorestart =0; //Autorestart falls Serial read zu lange
 
 
//Deffinition WLAN und MQTT

const char* ssid = "your SSID";
const char* password = "your WIFI passwort";
const char* mqtt_server = "IP Adresse MQTT Server";
const int mqtt_port = 1887 //Port MQTT Server;
const char* mqtt_user = "MQTT User";
const char* mqtt_passwort = "MQTT Passwort";
const char* mqtt_name = "MQTT Name";




struct Vector_GCM {
  const char *name;
  byte keysize;
  unsigned int datasize;
  byte authsize;
  byte ivsize;
  byte tagsize;
  uint8_t key[16];
  byte plaintext[MAX_PLAINTEXT_LEN];
  byte ciphertext[MAX_PLAINTEXT_LEN];
  byte authdata[17];
  byte iv[12];
  byte tag[12];
};

struct IncommingData {
  byte year, month, day, hour, minutes, seconds;
  unsigned long wirkenergiePlus, wirkenergieMinus, momentanleistungPlus, momentanleistungMinus;
  float uL1, uL2, uL3, iL1, iL2, iL3, powerF;
};

IncommingData aktuelleDaten;

Vector_GCM datenMbus = {   //static
  .name        = "AES-128 GCM",
  .keysize     = 16,
  .datasize    = 297,
  .authsize    = 17,
  .ivsize      = 12,
  .tagsize     = 12,
  .key         = {0x99, 0xF9, 0x07, 0xC5, 0x3A, 0x99, 0x64, 0x60, 0x8B, 0x1A, 0x1A, 0x90, 0x1D, 0xEF, 0xB9, 0x29},
  .plaintext   = {},
  .ciphertext  = {},
  .authdata    = {},
  .iv          = {},
  .tag         = {},  
};



WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
unsigned long lastMsg1 = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
char rpn[MSG_BUFFER_SIZE];
char rpnreal[MSG_BUFFER_SIZE];
//char analogi2c[MSG_BUFFER_SIZE];

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    
    Serial.print(".");
    lcd.clear();
    lcd.setCursor(0,0);
	  lcd.print("!W");
    lcd.setCursor(1,1);
    lcd.print(aktuelleDaten.hour);
    lcd.print(":");
    lcd.print(aktuelleDaten.minutes);
    lcd.print(":");
    lcd.println(aktuelleDaten.seconds);
    delay(5000);
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    }
  
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    Umdrehungen = 0;
    Umdrehungenreal = 0;
  }
  //Serial Print für Plotter einschalter
  if ((char)payload[0] == 'd') {
    debug=!debug;
    Serial.println(debug);
  }
  //MQTT Senden aus bzw einschlaten
  
  if ((char)payload[0] == 'm') {
    mqtt_send=!mqtt_send;
    Serial.println(mqtt_send);
  }

  if ((char)payload[0] == 'r') {
    ESP.restart();
  }
  if ((char)payload[0] == 'a') {
    autorestart=!autorestart;
    if(autorestart){
      lcd.setCursor(0,0);
      lcd.print("Autorestart ON");
    }
  }
}
void mqtt_publish_float(const char *name, float value)
{
  char buffer[32];
  
  sprintf(buffer, "%0.2f", value);
  if(!client.publish(name, buffer))
  {
    Serial.print("Fehhler beim Daten senden");
  }
  Serial.printf("Published %s : %s\n", name, buffer);
}

void mqtt_publish_int(const char *name, uint32_t value)
{
  char buffer[32];
  sprintf(buffer, "%d", value);
  client.publish(name, buffer);
  Serial.printf("Published %s : %s\n", name, buffer);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    lcd.clear();
    lcd.setCursor(3,0);
	  lcd.print("!M");
    lcd.setCursor(1,1);
      lcd.print(aktuelleDaten.hour);
      lcd.print(":");
      lcd.print(aktuelleDaten.minutes);
      lcd.print(":");
      lcd.println(aktuelleDaten.seconds);

    if (client.connect(mqtt_name, mqtt_user, mqtt_passwort)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish("outTopic", "Time:");
      // ... and resubscribe
      client.subscribe("inTopic");
      //client.publish("outTopic", "2017-02-16T10:13:52");
      // ... and resubscribe
      //client.subscribe("inTopic");
      delay(100);
     
    } else {

      lcd.setCursor(8,0);
	  lcd.print(".");
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(10000);
    }
  }
}

void decrypt_text(Vector_GCM &vect) {
  GCM<AES128> *gcmaes128 = 0;
  gcmaes128 = new GCM<AES128>();
  gcmaes128->setKey(vect.key, gcmaes128->keySize());
  gcmaes128->setIV(vect.iv, vect.ivsize);
  gcmaes128->decrypt(vect.plaintext, vect.ciphertext, vect.datasize);
  delete gcmaes128;
}

void setup() {
  Serial.begin(115200);

  setup_wifi();
  lcd.init(I2C_SDA, I2C_SCL); // initialize the lcd to use user defined I2C pins
	lcd.backlight();
	lcd.setCursor(3,0);
	lcd.print("Hello");
	//lcd.setCursor(2,1);
	//lcd.print("Time is now");

   //SerialBT.begin("ESP_EVN_BT");
   
  esp_task_wdt_init(15, true); //Watchdog 15 sek akivieren
  esp_task_wdt_add(NULL); //Watchdog keine Erhöhung
  
  startzeit=millis();
  pinMode(zaehler, INPUT);
  
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  for (int i = 0; i < MAX_PLAINTEXT_LEN; i++) {
    datenMbus.plaintext[i] = 0x00;
    datenMbus.ciphertext[i]=0x00;
  }
  
  Serial2.begin(2400, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial Txd on pin: "+String(TXD2));
  Serial.println("Serial Rxd on pin: "+String(RXD2));
  delay(500);
}

void loop() {
  buttonState = digitalRead(zaehler);
  // (SerialBT.available()) {
  //SerialBT.write(buttonState);
  //}
  //delay(10);
  if (buttonState == 0 && flanke1==0){
    flanke1 = 1;
    Umdrehungenreal = Umdrehungenreal + 1;
  }
  sensorValue = analogRead(sensorPin);
  //SerialBT.println(" , ");//.write(" , ");
  //SerialBT.write(sensorValue);
  
  if (sensorValue<3800&&flanke==0){
    Umdrehungen = Umdrehungen + 1;
    flanke = 1;
  }

  

  unsigned long now1 = millis();
  if (now1 - lastMsg1 > 50) {
      lastMsg1 = now1;

      if(buttonState == 1 && flanke1==1){

        delay(10);
        flanke1 = 0;
      
    }
    if (sensorValue>4090 && flanke==1){
      flanke = 0;
    }
  }
  
if(debug){
  Serial.print("Analog Wert: ");
  Serial.print(sensorValue);
  Serial.print("Impuls:: ");
  Serial.print(buttonState);//für Plotter
  Serial.print("Umdrehungen von Analogwert: ");
  Serial.print(Umdrehungen);
  Serial.print("Umdrehungen von Impuls ");
  Serial.println(Umdrehungenreal);
  //delay(5000);
}

  if (!client.connected()) {
    Serial.print("Fehler MQTT");
    reconnect();
    
  }
  client.loop();

  if (millis() > timeSinceLastData + 3000) {                // Das SmartMeter sendet alle 5Sek neue Daten-> 
      byteNumber = 0;                                       // Position im Speicher Array nach 3Sek wieder auf null setzen um neue Daten empfangen zu können
    }
//Serial.println(Serial2.available());
  while (Serial2.available() > 0 ){                         // Wenn Daten im Buffer der Ser. SChnittstelle sind....
    
    
    if (byteNumber < MAX_PLAINTEXT_LEN) {
      datenMbus.ciphertext[byteNumber] = Serial2.read();    // Daten speichern
      byteNumber++;                                         // Zählvariable erhöhen
    }
    else{
      fehlerdurchlauf++;
      lcd.clear();
      Serial.print("Error Buffer overview");
      lcd.setCursor(9,0);
      lcd.print("Error:");
      lcd.print(durchlauf);
      lcd.print(" ");
      lcd.setCursor(0,1);
      lcd.print(fehlerdurchlauf);
      lcd.setCursor(2,1);
      lcd.print(aktuelleDaten.hour);
      lcd.print(":");
      lcd.print(aktuelleDaten.minutes);
      lcd.print(":");
      lcd.println(aktuelleDaten.seconds);

      mqtt_publish_int("feeds/integer/Smartmeter/Fehlerdurchlauf", durchlauf);
      mqtt_publish_int("feeds/integer/Smartmeter/Fehleranzahl", fehlerdurchlauf);
      durchlauf=0;

      if(fehlerdurchlauf>30){
        delay(5000);
         ESP.restart();
      }
      return;
    }
    timeSinceLastData = millis();
  }
  if (millis() > timeSinceLastData + 3000) {                // Sind mehr als 3 Sekunden vergangen-> Daten entschlüsseln
    if (processData) {
     /*Serial.println("Daten vom Smart Meter: ");          // Ausgabe der eingelesenen Rohdaten(verschlüsselt)
      for (int i = 0; i < MAX_PLAINTEXT_LEN; i++) {   //
        if (datenMbus.ciphertext[i] < 0x10)Serial.print("0");
        Serial.print(datenMbus.ciphertext[i], HEX);
      }
      Serial.println("");
      */
      for (int i = 0; i < 8; i++) {                          // Initialisation Vektor (IV) bilden (8Byte System Title + 4Byte Frame Counter) ...befinden sich immer an der selben stelle im Datensatz
        datenMbus.iv[i] = datenMbus.ciphertext[i + 11];
      }
      for (int i = 0; i < 4; i++) {
        datenMbus.iv[i + 8] = datenMbus.ciphertext[i + 22];  // FrameCounter anhängen...
      }
      
      for (unsigned int i = 0; i < datenMbus.datasize - 26; i++) { // Anfang der Nachricht "löschen", sodass nur mehr die verschlüsselten Daten in dem Array bleiben
        datenMbus.ciphertext[i] = datenMbus.ciphertext[i + 26];
      }
      for(int i = 256;i<MAX_PLAINTEXT_LEN;i++){
      datenMbus.ciphertext[i]=0x00;
      }
      decrypt_text(datenMbus);
     /* 
      Serial.print("Iv: ");
      for (int i = 0; i < 12; i++) {
        if (datenMbus.iv[i] < 0x10)Serial.print("0");
        Serial.print(datenMbus.iv[i], HEX);
      }
      Serial.println();
      Serial.println("Entschluesselte Daten: ");
      for (unsigned int i = 0; i < datenMbus.datasize; i++) {
        if (datenMbus.plaintext[i] < 16)Serial.print("0");
        Serial.print(datenMbus.plaintext[i], HEX);
      }
      
      Serial.println(" ");

      */

     if(durchlauf>2){
      aktuelleDaten.year = ((datenMbus.plaintext[6] << 8) | datenMbus.plaintext[7]) - 2000;
      aktuelleDaten.month = datenMbus.plaintext[8];
      aktuelleDaten.day = datenMbus.plaintext[9];
      aktuelleDaten.hour = datenMbus.plaintext[11];
      aktuelleDaten.minutes = datenMbus.plaintext[12];
      aktuelleDaten.seconds = datenMbus.plaintext[13];
      aktuelleDaten.wirkenergiePlus=((unsigned long)datenMbus.plaintext[43]<<24)|((unsigned long)datenMbus.plaintext[44]<<16)|((unsigned long)datenMbus.plaintext[45]<<8)|(unsigned long)datenMbus.plaintext[46];
      aktuelleDaten.wirkenergieMinus=((unsigned long)datenMbus.plaintext[62]<<24)|((unsigned long)datenMbus.plaintext[63]<<16)|((unsigned long)datenMbus.plaintext[64]<<8)|(unsigned long)datenMbus.plaintext[65];
      aktuelleDaten.momentanleistungPlus=((unsigned long)datenMbus.plaintext[81]<<24)|((unsigned long)datenMbus.plaintext[82]<<16)|((unsigned long)datenMbus.plaintext[83]<<8)|(unsigned long)datenMbus.plaintext[84];
      aktuelleDaten.momentanleistungMinus=((unsigned long)datenMbus.plaintext[100]<<24)|((unsigned long)datenMbus.plaintext[101]<<16)|((unsigned long)datenMbus.plaintext[102]<<8)|(unsigned long)datenMbus.plaintext[103];
      aktuelleDaten.uL1=float((datenMbus.plaintext[119]<<8)|datenMbus.plaintext[120])/10.0;
      aktuelleDaten.uL2=float((datenMbus.plaintext[136]<<8)|datenMbus.plaintext[137])/10.0;
      aktuelleDaten.uL3=float((datenMbus.plaintext[153]<<8)|datenMbus.plaintext[154])/10.0;
      aktuelleDaten.iL1=float((datenMbus.plaintext[170]<<8)|datenMbus.plaintext[171])/100.0;
      aktuelleDaten.iL2=float((datenMbus.plaintext[187]<<8)|datenMbus.plaintext[188])/100.0;
      aktuelleDaten.iL3=float((datenMbus.plaintext[204]<<8)|datenMbus.plaintext[205])/100.0;
      aktuelleDaten.powerF=float((datenMbus.plaintext[221]<<8)|datenMbus.plaintext[222])/1000.0;
      Serial.print(aktuelleDaten.day);
      Serial.print(".");
      Serial.print(aktuelleDaten.month);
      Serial.print(".");
      Serial.print(aktuelleDaten.year);
      Serial.print("  ");
      Serial.print(aktuelleDaten.hour);
      Serial.print(":");
      Serial.print(aktuelleDaten.minutes);
      Serial.print(":");
      Serial.println(aktuelleDaten.seconds);
      lcd.setCursor(0,0);
      lcd.print(aktuelleDaten.hour);
      lcd.print(":");
      lcd.print(aktuelleDaten.minutes);
      lcd.print(":");
      lcd.println(aktuelleDaten.seconds);
      Serial.print("A+: ");
      Serial.print(aktuelleDaten.wirkenergiePlus);
      lcd.setCursor(0,1);
      lcd.print("A+:");
      lcd.print(aktuelleDaten.wirkenergiePlus);
      lcd.print(" ");
      lcd.print(durchlauf);

      Serial.print("Wh | A-: ");
      Serial.print(aktuelleDaten.wirkenergieMinus);
      Serial.println("Wh");
      Serial.print("P+: ");
      Serial.print(aktuelleDaten.momentanleistungPlus);
      Serial.print("W | P- (einsp.): ");
      Serial.print(aktuelleDaten.momentanleistungMinus);
      Serial.print("W  ");
      Serial.print("Saldo: ");
      Serial.print(aktuelleDaten.momentanleistungPlus-aktuelleDaten.momentanleistungMinus);  
      Serial.println(" W");
      Serial.println("U1: " + String(aktuelleDaten.uL1) + "V  U2: " + String(aktuelleDaten.uL2)+ "V  U3: " + String(aktuelleDaten.uL3)+"V");
      Serial.println("I1: " + String(aktuelleDaten.iL1) + "A  I2: " + String(aktuelleDaten.iL2)+ "A  I3: " + String(aktuelleDaten.iL3)+"A");
      Serial.print("PowerFactor: ");
      Serial.println(aktuelleDaten.powerF);
      
      Serial.println("");

    if(mqtt_send==1){
      if(millis()>startzeit+2000){ //Startverzögerung für Senden
      //Daten von Smartmeter mittels MQTT senden
      mqtt_publish_int("feeds/integer/Smartmeter/WirkenergiePlus", aktuelleDaten.wirkenergiePlus);
      mqtt_publish_int("feeds/integer/Smartmeter/WirkenergieMinus", aktuelleDaten.wirkenergieMinus);
      mqtt_publish_int("feeds/integer/Smartmeter/MomenanleistungPlus", aktuelleDaten.momentanleistungPlus);
      mqtt_publish_int("feeds/integer/Smartmeter/MomenanleistungMinus", aktuelleDaten.momentanleistungMinus);
      mqtt_publish_int("feeds/integer/Smartmeter/MomenanleistungSaldo", (aktuelleDaten.momentanleistungPlus - aktuelleDaten.momentanleistungMinus)); 
      mqtt_publish_float("feeds/float/Smartmeter/UL1", aktuelleDaten.uL1);
      mqtt_publish_float("feeds/float/Smartmeter/UL2", aktuelleDaten.uL2);
      mqtt_publish_float("feeds/float/Smartmeter/UL3", aktuelleDaten.uL3);
      mqtt_publish_float("feeds/float/Smartmeter/IL1", aktuelleDaten.iL1);
      mqtt_publish_float("feeds/float/Smartmeter/IL2", aktuelleDaten.iL2);
      mqtt_publish_float("feeds/float/Smartmeter/IL3", aktuelleDaten.iL3);
      mqtt_publish_float("feeds/float/Smartmeter/Powerfactor", aktuelleDaten.powerF);
      snprintf (msg, MSG_BUFFER_SIZE, "%i", sensorValue);
      snprintf (rpn, MSG_BUFFER_SIZE, "%i", Umdrehungen);
      snprintf (rpnreal, MSG_BUFFER_SIZE, "%i", Umdrehungenreal);
    
      //IR Signele senden
      client.publish("Analog", msg);
      delay(1);
  
      client.publish("RPN", rpn);
      delay(1);
      client.publish("Umdrehungenreal", rpnreal);

      
          Serial.println(durchlauf);
     }
     
    }

     }
      for (int i = 0; i < MAX_PLAINTEXT_LEN; i++) {
        datenMbus.plaintext[i] = 0x00;
        datenMbus.ciphertext[i] = 0x00;
      }
      processData = false;
      durchlauf++;
      Serial.println(durchlauf);
    }
  } else {
    processData = true;
  
  }



esp_task_wdt_reset();  //Watchdog rücksetzen  
}



