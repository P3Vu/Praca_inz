//  Praca Inżynierska Paweł Wojciechowkski 2019
// Cyfrowa Poziomica z Interfesem Bezprzewodowym
// Digital Inclinometer with Wireless Interface

// Biblioteki od Sensora i Wyswietacza OLED
#include "SSD1306.h"                                        // Biblioteka do sterownika wyswietlacza OLED
#include "Wire.h"
#include "MPU6050.h"
#include "math.h"

//BLE
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

BLECharacteristic *characteristicTX;

// Biblioteki od LoRa
#include <SPI.h>
#include <LoRa.h>
#include <Arduino.h>

//            Podlaczenie pinow OLED ( wbudowany ; 128x64 ) do ESP32 GPIO  

// OLED_SDA -- GPIO4
// OLED_SCL -- GPIO15
// OLED_RST -- GPIO16

//            Podlaczenie pinow MPU6050 do ESP32 
//MPU6050_SDA -- GPIO4
//MPU6050_SCL -- GPIO15

//            Podlaczenie pinow SX1278 ( wbudowany ) do ESP32 - LoRa
// GPIO5  -- SX1278's SCK
// GPIO19 -- SX1278's MISO
// GPIO27 -- SX1278's MOSI
// GPIO18 -- SX1278's CS
// GPIO14 -- SX1278's RESET
// GPIO26 -- SX1278's IRQ(Interrupt Request)


SSD1306 display( 0x3c , 4 , 15 );                             // Z biblioteki SSD1306Wire.h ( ADRES , SDA, SCL ) 
MPU6050 czujnik( 0x68 );   // podaje mu domyslnie adres 0x68 sprawdzic , mozna tez zerknac na faktyczny adres oleda 

#define interruptPin_Gora 32 
#define interruptPin_Dol 33 
#define interruptPin_Enter 12 
#define interruptPin_Esc 13 

#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex - od Sleepa

// od SX1278
#define SS      18
#define RST     14
#define DI0     26
#define BAND    868E6  //915E6 

#define SERVICE_UUID   "ab0828b1-198e-4351-b779-901fa0e0371e"
#define CHARACTERISTIC_UUID_TX  "0972EF8C-7613-4075-AD52-756F33D4DA91"  

#define CHARACTERISTIC_UUID_RX  "4ac8a682-9736-4e5d-932b-e9b31405049c"

void przycisk_gora();
void przycisk_dol();
void przycisk_Enter();
void przycisk_Esc();
void Analiza_pomiarow();
void Transmisja_LoRa();
void Sprawdz_Interfejsy();
void Transmisja_WiFi();

long debounce_time = 200;             // debouncing w milisec
volatile unsigned long last_micros;
int x_kursor = 110;
int y_kursor = 0;
volatile bool Enter = false;
volatile  bool Esc = false;
int l_menu = 0; // 0 - menu glowne ; 1 - pod menu 1 itd.
int l_menu_pre = 0; // zmienna by zapamiętać poprzedni stan l_menu 

// zmienne MPU6050
const float pi = 3.141592;
const float sample_no = 25;    // liczba próbek dla aproksymacji
  
int16_t ax, ay, az;  // define accel as ax,ay,az
float x , y , z;
int sample;

float angle_x , angle_y , angle_z = 0 ;
long ax_sum , ay_sum , az_sum = 0;

int16_t gx, gy, gz;  // define gyro as gx,gy,gz

// zmienne LoRa
int counter = 0;
bool Lora_ON = false;

// BLE
bool deviceConnected = false;
bool BLE_ON = false;
//float pomiar = 10.50;


/* *********************** Klasa do callbacku BLE  *************** */

class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
 deviceConnected = true;
    };
 
    void onDisconnect(BLEServer* pServer) {
 deviceConnected = false;
    }
};

/*  ************************ SETUP ********************* */

void setup() {
  Wire.begin();     // join I2C bus
  Serial.begin(115200); // wylacznie pod testowanie na PC
  Serial.println();
  
  while(!Serial);

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_34,1); //1 = High, 0 = Low

  pinMode(16 , OUTPUT );
  digitalWrite(16, LOW);                                    // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(16,HIGH);                                    // while OLED is running , must set GPIO16 in high ( reset na wysokim podczas pracy )

  Serial.println("Initialazing I2C devices...");
  display.init();
  czujnik.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(czujnik.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  pinMode(interruptPin_Gora, INPUT_PULLUP); //GPIO 32 jako we
  pinMode(interruptPin_Dol, INPUT_PULLUP); //GPIO 33 jako we
  pinMode(interruptPin_Enter, INPUT_PULLUP); //GPIO 12 jako we
  pinMode(interruptPin_Esc, INPUT_PULLUP); //GPIO 13 jako we

  attachInterrupt(digitalPinToInterrupt(interruptPin_Gora) , przycisk_gora , RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPin_Dol) , przycisk_dol , RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPin_Enter) , przycisk_Enter , RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPin_Esc) , przycisk_Esc , RISING);      

  display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_LEFT); 

  pinMode( 22 , OUTPUT );  // Led dla sygnalizowania transmisji interfejsem LoRa
  digitalWrite(22 , LOW); // Inicjalizuj niskim stanem
  
  LoRa.setPins(SS,RST,DI0);
  SPI.begin(5,19,27,18);  // SCK, MISO , MOSI, CS
  
  Serial.println("LoRa Sender Initializing..");

  if(!LoRa.begin(BAND)){
    Serial.println("Starting LoRa failed!");
    while(1); 
  }

  Serial.println("LoRa Initial Successfull!");

// BLE
  pinMode( 17, OUTPUT);
  pinMode(17, LOW);
  BLEDevice::init("ESP32-BLE");   // Nazwa urzadzenia BLE
  BLEServer *server = BLEDevice::createServer(); // tworzenie serwera BLE
  server->setCallbacks(new ServerCallbacks());  
  BLEService *service = server->createService(SERVICE_UUID);
  
     characteristicTX = service->createCharacteristic(
                       CHARACTERISTIC_UUID_TX,
                       //BLECharacteristic::PROPERTY_NOTIFY |
                       BLECharacteristic::PROPERTY_WRITE
                     );
                        
  //characteristicTX->addDescriptor(new BLE2902());

  BLECharacteristic *characteristic = service->createCharacteristic(
               CHARACTERISTIC_UUID_RX,
               BLECharacteristic::PROPERTY_WRITE
              );

  service->start();
  server->getAdvertising()->start();
  
}
// -----------------------------------------------------------

/*  ************************ PĘTLA GŁÓWNA ********************* */

void loop() {

    // obsluga przyciskow
    if ( y_kursor < 0 ) y_kursor = 50;
    else if ( y_kursor > 50 ) y_kursor = 0;


    if ( Enter == true ){

      if ( l_menu == 0 ){
        if ( y_kursor == 0 ) l_menu = 1;
        if ( y_kursor == 10 ) l_menu = 2;
        if( y_kursor == 30 ) l_menu = 4;        
      }

      else if ( l_menu == 1 ){
        
      }

      else if ( l_menu == 2){
      
        if ( y_kursor == 0 && Lora_ON == false ){
          Serial.println("Wlaczono LoRa.");
          Lora_ON = true;
        }
        else if ( y_kursor == 0 && Lora_ON == true ){
          Serial.println("Wylaczono LoRa.");
          Lora_ON = false;
        }

        if ( y_kursor == 20 && BLE_ON == false ){
          Serial.println("Wlaczono BLE.");
          BLE_ON = true;
        }
        else if ( y_kursor == 20 && BLE_ON == true ){
          Serial.println("Wylaczono BLE.");
          BLE_ON = false;
        }
        
      }

      else if ( l_menu == 4 ){
        if ( y_kursor == 30 ){
          l_menu = 41;
        }
      }

      else if ( l_menu == 41 ){
        l_menu = 42;
      }
      
      Enter = false;
    }

    if( Esc == true ){
      l_menu = 0;
      Esc = false;
    }
    
    Analiza_pomiarow();
    wyswietl_menu(l_menu);
    
}

// ------------------------------------------------------------

void wyswietl_menu( int l_menu ){

  display.clear();

  switch ( l_menu ){
    
    case 0:     display.drawString( 15 , 0 , "1. Podglad na zywo");
                display.drawString( 15, 10 ,  "2. Transmisja");
                display.drawString( 15 , 20 , "3. Opcje");
                display.drawString( 15 , 30 , "4. Uspij");
                
                display.drawString( x_kursor , y_kursor , "<-");    // rysuj kursor 
                
                break;
      
    case 1:     display.drawString( 35 , 0 , "X : " + (String)angle_x);
                display.drawString( 35 , 10, "Y : " + (String)angle_y);
                display.drawString( 35 , 20, "Z : " + (String)angle_z);

                display.drawString( 15 , 50, "Esc aby wrocic");
                break;
    
    

    case 2:     display.drawString(15 , 0 , "LoRa :");
                display.drawString(15, 20 , "BLE :");
                Sprawdz_Interfejsy();

                display.drawString( x_kursor , y_kursor , "<-");    // rysuj kursor 
                display.drawString(15,50, "Esc aby wrocic");
                break;

    case 4:     display.drawString( 15 , 30 , "Przejdz w tryb uspienia" );
                break;

    case 41:    display.drawString( 15 , 20 , "Jestes pewien ?");
                display.drawString( 15, 30 , " Enter - Tak / Esc - Nie ");
                break;

    case 42:    display.drawString( 15, 30 , "No to ide spac.");
                delay(2000);
                esp_deep_sleep_start();
                break;            
  }
  display.display();
}


void Analiza_pomiarow(){
  czujnik.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  // read measurements from device    
  
  ax_sum = ax_sum + ax;
  ay_sum = ay_sum + ay;
  az_sum = az_sum + az;
  sample++;

  if ( sample == sample_no )
  {
    // Uśredniamy
  x = ax_sum /sample_no;
  y = ay_sum /sample_no;
  z = az_sum /sample_no;

  // Przeliczenie na katy
  angle_x = atan2(x , sqrt((y*y) + (z*z))) / (pi/180);
  angle_y = atan2(y , sqrt((x*x) + (z*z))) / (pi/180);
  angle_z = atan2(sqrt((x*x) + (y*y)) , z )/ (pi/180);

  // Reset dla nastepnego obiegu

  sample = 0;
  ax_sum = 0;
  ay_sum = 0;
  az_sum = 0;

  }  
}


void Transmisja_LoRa(){

  digitalWrite( 22 , HIGH);   // dioda sygnalizujaca wlaczony interfejs LoRa
 
  Serial.print("Sending packet: ");
  Serial.println(counter);

  // send packet
  LoRa.beginPacket();

  if(counter == 0){       // nastepne przeslanie bedzie X-em
    LoRa.print("X ");
    LoRa.println(angle_x);
    counter++;
  }

  else if(counter == 1){
    LoRa.print("Y ");
    LoRa.println(angle_y);
    counter++;
  }
  
  else if(counter == 2){
    LoRa.print("Z ");
    LoRa.println(angle_z);
    counter = 0;
  }
  
  LoRa.endPacket();

  // ogarnac pozniej jakos ten delay
  delay(50);

}

void Sprawdz_Interfejsy(){
              
   if ( Lora_ON == true ){
      display.drawString(70, 0 , "ON");
      Transmisja_LoRa(); 
   }
                
   else {
      display.drawString(70, 0, "OFF");
      //LoRa.end();    end chyba wylacza caly interfejs wiec potrzebny begin 
      digitalWrite( 22 , LOW);   
   } 

   if ( BLE_ON == true ){
    display.drawString(70 , 20 , "ON");
    Transmisja_BLE();
   }
   else{
    display.drawString(70 ,  20 , "OFF");
    digitalWrite(17, LOW);
   }
}

void Transmisja_BLE(){

  digitalWrite(17 , HIGH );

    if ( deviceConnected ) {
            String pomiarx = String( angle_x ) + " " + String( angle_y )+  " " + String( angle_z );
            char txString[50];
            pomiarx.toCharArray( txString , 50 );

            //dtostrf( pomiarx , 2 , 2 , txString ); // float_val, min_width, digits_after_decimal, char_buffer
            characteristicTX->setValue(txString); 
            characteristicTX->notify(); 
  }
}
/*  ************************ Przerwania ********************* */

void przycisk_gora(){

  if((long)(micros() - last_micros) >= debounce_time * 1000){
    Serial.println("Gora");
    y_kursor -= 10;
    last_micros = micros();
  }
}

void przycisk_dol(){
  if((long)(micros() - last_micros) >= debounce_time * 1000){
    Serial.println("Dol");
    y_kursor += 10;
    last_micros = micros();
  }
}

void przycisk_Enter(){
  if((long)(micros() - last_micros) >= debounce_time * 1000){
    Serial.println("Enter");
    Enter = true;
    last_micros = micros();
  }
}

void przycisk_Esc(){
  if((long)(micros() - last_micros) >= debounce_time * 1000){
    Serial.println("Esc");
    Esc = true;
    last_micros = micros();
  }
}
