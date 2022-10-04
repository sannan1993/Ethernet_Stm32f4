/*
   gpio 14

   merge two codes in a single:-

   when logic is high run slave code.
   and when logic is low run configurator code.


   Connect D6 to 3v3 for opening the page of printer configurator.
   Connect the D5 to get the following operations:-

   D5 = GND  ( Gms Configurator mode)
   D5 = HIGH ( Uploading mode)
*/







#include <Arduino.h>
#include <ESP8266WiFi.h>

// GPIO14 is a livecon
#define BUTTON1   14

//according to nodemcu
#define LED_ESP12  2    //gpio 2
#define LED_BOARD  16   //gpio 16


/*===============================================================================================*/
//SLAVE PART
/*===============================================================================================*/
// 0 for alnnomate ,12 for new atg_device, 16 for bulit in
#define WiFi_LED  12

// define debuge level 1 meanes ON 0 means OFF
#define Debug     0

#if Debug==1

#define Debug_print(X)    Serial.print(X)
#define Debug_println(X)  Serial.println(X)

#define DEBUG_ESP_PORT

#else

#define Debug_print(X)
#define Debug_println(X)

#endif

// meta-data
#define VERSION "ESP_07_multi_server_no_fb_V3.3.3"
#define S_DATE "08-Oct-2018"
#define R_DATE "15-March-2018"


// Wifi SSID and Password
const char* ssid      = "AlnnoMate";
const char* password  = "@lsons_t@tsuno";

byte mac[6];
char mac_add[18];

int wifi_status = WL_IDLE_STATUS;

// The String recived form alnno slave
String controller_received_string = "";

// The String recived from master
String received_string = "";

// Force Reconnection and GPIO Control
int Forced_Led_Low = 0;
int Forced_Reconn = 0;

unsigned long lastReconnTime = 0;                        // last time you reconnected to the servers
unsigned long lastConnectPrinter = 0;                    // last time you connected to the printer
unsigned long lastConnectionTime = 0;                    // last time you connected to the server, in milliseconds
unsigned long lastWifiConnection = 0;                    // last time you try to reconnect the wifi
unsigned long lastCheckWebSock = 0;                      // last time the websocket was checked.

const unsigned long ConnectionIntervalWifi = 10000L;     // the time after which wifi reconnectes(if not connected)
const unsigned long ConnectionIntervalPrinter = 5000L;   // delay between reconn, printer, this is to insure connection as the printer terminates connection inproperly // not used
const unsigned long ConnectionIntervalForce = 10000L ;   // Force connection interval delay between updates, in milliseconds, cool down time
const unsigned long ConnectionIntervalServer = 10000L;   // reconnection cool down time, after this time the system will connect to the servers is not connected
const unsigned long ConnectionIntervalWebSocket = 5000L; // checking of websocket status


void printWifiStatus();

// Main Server IP //
IPAddress master_IP(192, 168, 69, 69); // Master
const int master_Port = 8080;
WiFiClient master_Client;
// End of Main Server //

// control code ESP
char ETX = '\x03';                                        // end of text , used for identify id
char EOT = '\x04';                                        // end of transmition, used as end of string

extern uint8_t MySocketWrite; // see websockets.cpp line number 725

/*===============================================================================================*/
// PRINTER //
/*===============================================================================================*/

//printer configurator
#define STM32_CONT_PIN    12


/*   Configurator packet len      */
#define packlen     500

void send_DataToServer(void); //Prototyping
void load_Params      (void);
void webServerHandler (void);
void debugVariables   (void);

class WiFi_Configurator
{
  public:
    char RightDispenserID[30], LeftDispenserID[30], device[12], email[40], MobileNumber[15], server[6], battery_type[3], generator_battery[7], extend[3], pump_name[60], date[20], Time[20], printer_mode[3], dual_mode[3], deci_point[3];
  char prod[6][20];
} params;

WiFiClient client;
WiFiServer server(80);
IPAddress AP_IP    (192, 168, 0, 10);
IPAddress gateway  (192, 168, 4, 9);
IPAddress subnet   (255, 255, 255, 0);

String serverString;
/*===============================================================================================*/
uint16_t buttonCount0 = 0, buttonCount1 = 0;
uint8_t buttonPressed = 0;
/*===============================================================================================*/


void slave_init(void);




/*

   ======================
  Timer 1 Handler (500ms)
  =======================

  programStatus = 0   // run slave code..
  programStatus = 1   // run configurator code..

*/
void ICACHE_RAM_ATTR onTimerISR() {

  digitalWrite(LED_BOARD, !(digitalRead(LED_BOARD))); //read opposite logic and write in on the gpio.

  //scan for high logic
  if ( (digitalRead (BUTTON1) == 0) && (buttonPressed == 0)) {

    if (buttonCount0 >= 2) {            //1-sec

      //clear the counts
      buttonCount0 = 0;
      buttonCount1 = 0;

      //button is pressed
      buttonPressed = 1;

    }
    else {
      ++buttonCount0;
    }
  }
  else {
    buttonCount0 = 0;
  }

  //scan for low logic
  if ((digitalRead (BUTTON1) == 1) && (buttonPressed == 1)) {

    if (buttonCount1 >= 2) {

      buttonCount0 = 0 ;
      buttonCount1 = 0 ;

      //button is released
      buttonPressed = 0;
    }

    else {
      ++buttonCount1;
    }
  }
  else {
    buttonCount1 = 0;
  }
  timer1_write(2500000);//12us
}

/************************************************ SETUP LOOP ********************************************************************************/
void setup() {


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
  pinMode (BUTTON1,   INPUT);      // button1 is sensed

  //Configure the LEDs
  pinMode (LED_ESP12, OUTPUT);
  pinMode (LED_BOARD, OUTPUT);

  //Initialize Ticker every 0.5s
  timer1_attachInterrupt(onTimerISR);


  //80Mhz/16 = 5-Mhz =
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write(2500000);                                    // 2500000 / 5 ticks per us from TIM_DIV16 == 500,000 us interval

  //Setting up the leds
  digitalWrite (LED_BOARD, HIGH);
  digitalWrite (LED_ESP12, HIGH);
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.setTimeout(10);
  Serial1.setTimeout(10);
  delay(2);
}
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/************************************************************************ MAIN LOOP ********************************************************************************/
void loop()
{

  // configuration mode.....
  if ( buttonPressed == 1 )
  {
    Serial1.print ("\r\n \tPRINTER CONTROLLER.....");

    /*=================================================================================*/
    int WiFiStatus = 0;
    short Flag = 0;
    bool SoftAP_Flag = false;
    unsigned long previousMillis = 0;
    const long interval = 1000;
    /* Set these to your desired credentials. */
    const char *ssid       = "GMS_CONFIGURATOR";
    //Note: unable to use PROGMEM to stor data
    const char *password   = "@lsons_t@tsuno";
    unsigned long currentMillis;
    /*=================================================================================*/
    //initialize
    pinMode(STM32_CONT_PIN, INPUT_PULLUP); //to turn on the GMS configurator....

    while (buttonPressed) {
      /****************** ESTABLISH WIFI AS A ROUTER ******************************/
      if ((digitalRead (STM32_CONT_PIN) == 1) && (WiFiStatus == 0)) {

        Serial.print ("WiFi started!");
        WiFi.softAPConfig(AP_IP, gateway, subnet);

        Flag = WiFi.softAP(ssid, password, 1, false, 1);
        IPAddress myIP = WiFi.softAPIP();
        server.begin();

        //Update the WIFI Status.....
        WiFiStatus = 1;
        SoftAP_Flag = false;
      }


      /****************** SERVER THREAD ******************************/
      else if ((digitalRead (STM32_CONT_PIN) == 1) && (WiFiStatus == 1))
      {

        //Handle web server
        webServerHandler();
        delay (500);

      }

      /****************** DISCONNECT ROUTER *************************/
      else
      {
        //   WiFiStatus = 0;

        // Serial1.print ("\r\n router is getting turned off.....");

        if ((SoftAP_Flag == false) && (WiFiStatus == 1))
        {

          Serial1.print ("\r\n \tdisconnect AP mode......");

          WiFi.disconnect(); // disconnect the station mode.
          WiFi.mode(WIFI_OFF); //turn off the wifi.
          SoftAP_Flag = true;
          WiFiStatus = 0; //added inside

        }
        //delay (10);
        // store current ms
        currentMillis = millis();
        if (currentMillis - previousMillis >= interval)
        {
          previousMillis = currentMillis;
          Serial.print ("\r\n Router is OFF!");
        }

      }
    }
  }

  else
  {
    Serial1.print ("\r\n \t SLAVE MODE......");
    slave_init();  //initialize

    while (!buttonPressed) {

      //-------------------------------//
      // Server Data Handler //
      //-------------------------------//
      if (master_Client.available())
      {
        // Local Variable
        String data = "";
        int index;
        data = master_Client.readStringUntil('#'); // the '#' is not included int the data variable
        data += '#';
        index = received_string.indexOf(data);
        if ( index == -1) // if data is unique
        {
          received_string += data; // The data Recived form master
        }
      }

      // IF DATA IS RECEIVED FROM ALNNOMATE SLAVE.....
      if (Serial.available())
      {
        controller_received_string = "" ;                                 // empty the string
        controller_received_string = Serial.readStringUntil(EOT);         // read command or data
        Serial1.print("controller_received_string=");
        Serial1.println(controller_received_string);
        // -------- DECODE ALNNOMATE SLAVE COMMANDS----------------//
        // The command any_data is recived //
        if (controller_received_string == "any_data#")
        {
          // Print the Master String on terminal....
          Serial1.println(received_string);

          //if string received from master then send to alnnomate slave
          if (received_string.length() > 0) {
            Serial.println(received_string + '$'); // data ram
          }
          //otherwise send empty
          else {
            Serial.println("EMPTY$"); // FEED_BACK
          }
        }
        //if delete command is received then delete the receive buffer
        else if (controller_received_string == "del#")
        {
          received_string = ""; // clear ram buffer
          Serial.println("DEL_OK$");
        }

        // if no_resp is received then reconnect the wifi and send low to alnnomate slave.....
        else if (controller_received_string == "no_resp#")
        {
          digitalWrite(WiFi_LED, LOW);
          Forced_Led_Low = 1;
          Serial.println("RECONN_OK$"); // FEED_BACK
        }

        //Debug commands for esp on putty
        //-----------------------------------------------------------------------//
        else if (controller_received_string == "version#")
        {
          Serial.print(VERSION);
          Serial.println("$");
        }
        else if (controller_received_string == "s_date#")
        {
          Serial.print(S_DATE);
          Serial.println("$");
        }
        else if (controller_received_string == "r_date#")
        {
          Serial.print(R_DATE);
          Serial.println("$");
        }
        else if (controller_received_string == "wifi#")
        {
          int wifi_status = WiFi.status();
          Serial.print(wifi_status); // FEED_BACK
          Serial.print(":");
          switch (wifi_status)
          {
            case WL_CONNECTED:
              Serial.println("WL_CONNECTED$");
              break;
            case WL_NO_SHIELD:
              Serial.println("WL_NO_SHIELD$");
              break;
            case WL_IDLE_STATUS:
              Serial.println("WL_IDLE_STATUS$");
              break;
            case WL_NO_SSID_AVAIL:
              Serial.println("WL_NO_SSID_AVAIL$");
              break;
            case WL_CONNECT_FAILED:
              Serial.println("WL_CONNECT_FAILED$");
              break;
            case WL_CONNECTION_LOST:
              Serial.println("WL_CONNECTION_LOST$");
              break;
            case WL_DISCONNECTED:
              Serial.println("WL_DISCONNECTED$");
              break;
            case WL_SCAN_COMPLETED:
              Serial.println("WL_SCAN_COMPLETED$");
              break;
            default:
              Serial.println("UNKNOWN$");
              break;
          }
        }
        else if (controller_received_string == "serv#")
        {
          int conn = master_Client.connected();
          if (conn)
          {
            Serial.println("CONN_OK$");
          }
          else
          {
            Serial.println("CONN_NO$");
          }
        }
        else if (controller_received_string == "mac#")
        {
          Serial.print(mac_add);
          Serial.println("$");
        }
        else if (controller_received_string == "localIP#")
        {
          IPAddress ip = WiFi.localIP();
          //Serial.print("IP Addre ss: ");
          Serial.print(ipToString(ip));
          Serial.println("$");
        }
        //End of debug commands....
        //---------------------------------------------------------------------------//
        else
        {
          String id_str;
          long id;
          // check the alnnomate slave packet
          if (controller_received_string.length() > 0)
          {
            //receive till ETX
            int index = controller_received_string.indexOf(ETX);
            Debug_print("index=");
            Debug_println(index);
            if (index >= 0)  // if the character found,it is non negative
            {
              id_str = controller_received_string.substring(index + 1, controller_received_string.length());
              // The ID of the Server //
              // Used to indicate that at which server the data is to be send //
              // 1 = Master, 2=Thermal Printer(not Used), 3=LiveSocket(not Used)
              id = id_str.toInt();
              Debug_print("id=");
              Debug_println(id);

              controller_received_string = controller_received_string.substring(0, index);
              Debug_print("controller_received_string:");
              Debug_println(controller_received_string);

              // ID=1 means master //
              if (id == 1)
              {
                // Send data to master if wifi connected and connected to master server //
                if ((WiFi.status() == 3) &&  master_Client.connected())
                {
                  master_Client.print(controller_received_string);
                  Serial.println("SEND_OK$");// FEED_BACK
                  digitalWrite(WiFi_LED, HIGH);
                }
                else
                {
                  digitalWrite(WiFi_LED, LOW);
                  Serial.println("FAIL$");// FEED_BACK
                  Forced_Led_Low = 1; // goto reconnection logic
                }
              }
              else
              {
                Serial.println("NO_ID$");
                // Through Error
              } // Inner ifelse
            }
            else
            {
              Serial.println("NO_ETX$");
              // ETX not found
            }
          } // end of if
        }   // end of main ifelse
      }    // end of Serial.available()


      if (WiFi.status() == 3) // wifi Avaliable
      {
        // Forced Reconn main server //
        // 10 sec Reconnection Interval can be changed //
        if (millis() - lastConnectionTime > ConnectionIntervalForce && (Forced_Led_Low == 1))
        {
          digitalWrite(WiFi_LED, LOW);
          master_Client.stop();
          Debug_println("RC_SERV$");// FEED_BACK
          Serial1.println("Connection With Server Down and Forced Connection");

          if (master_Client.connect(master_IP, master_Port))
          {
            digitalWrite(WiFi_LED, HIGH);
            Forced_Led_Low = 0;
            Debug_println("OK_SERV$");// FEED_BACK
          }
          else
          {
            Debug_println("NO_SERV$");// FEED_BACK
          }
          lastConnectionTime = millis();
        }
        if (millis() - lastReconnTime > ConnectionIntervalServer)
        {
          // Reconnection logic main Server
          // if not connected to the main server
          if (!master_Client.connected())
          {
            digitalWrite(WiFi_LED, LOW);
            Serial1.println("Connection With Server Down");
            Debug_println("RC_SERV$"); // FEED_BACK

            if (master_Client.connect(master_IP, master_Port))
            {
              digitalWrite(WiFi_LED, HIGH);

              Forced_Led_Low = 0 ;
              Debug_println("OK_SERV$"); // FEED_BACK
            }
            else
            {
              // Do Nothing
              Debug_println("NO_SERV$");// FEED_BACK
            }
          }
          lastReconnTime = millis();

        }
      } // end of main if Wifi==3

      if (WiFi.status() != 3)
      {
        digitalWrite(WiFi_LED, LOW);
        if (millis() - lastWifiConnection > ConnectionIntervalWifi)
        {

          WiFi.begin(ssid, password);
          Debug_println("RECONN_WIFI$");
          lastWifiConnection = millis();
        }
        //delay(1000);
        //Serial1.println("Wifi Disconnected");
        //Serial.println("NO_WIFI$"); // FEED_BACK
      }
    }
  }

}
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*
   ========================================================================================================================================================

                                                                        Initialize slave

   ========================================================================================================================================================
*/
void slave_init(void) {
  int x = 0, y = 0, z = 0;
  unsigned long timeOut = 10000;      // wifi connection timeout
  unsigned long startTime;

  // ----- Initialization -----------//
  pinMode(WiFi_LED, OUTPUT);
  digitalWrite(WiFi_LED, LOW);

  // This is the ESP debugging information //
  // ESP will only output Dubug info if it is configured beform build //
  // dont worry about it //
  Serial1.setDebugOutput(true);
  Debug_print("Connecting to ");
  Debug_println(ssid);

  //turn off from AP mode
  server.close ();
  WiFi.mode(WIFI_OFF);

  //Set wifi mode
  WiFi.mode(WIFI_STA);

  // If the ESP cannot initilize //
  // This sould never happen //
  if (WiFi.status() == WL_NO_SHIELD)
  {
    Debug_println("WiFi shield not present");
    Debug_println("WiFi status==");
    Debug_print(WiFi.status());
    Serial.print("Fatal Error:");
    Serial.println(WiFi.status());
    // infinite loop
    while (true);
    digitalWrite(WiFi_LED, LOW);
  }

  // Connect to Rounter //
  WiFi.begin(ssid, password);

  // Wait for connection to router //
  // maximium 10seconds //
  startTime = millis(); // note start time
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Debug_print(".");

    // if timeout reach, 10 second timeout //
    if (millis() - startTime > timeOut)
    {
      // break out of the loop
      break;
      // TIMEOUT
    }
  }
  // Display status if connection status //
  if (wifi_status == WL_CONNECTED)
  {
    //Serial.println("OK_WIFI$");// FEED_BACK
    Debug_println("You're connected to the network");
    printWifiStatus();
  }
  else if (wifi_status != WL_CONNECTED)
  {
    //pull gpio low for alnnomate slave...
    digitalWrite(WiFi_LED, LOW);

    //Serial.println("NO_WIFI$");  // FEED_BACK
    Debug_println("You're not connected to the network");
    //get mac
    WiFi.macAddress(mac);
    sprintf(mac_add, "%02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
  }

  Debug_print("MAC Address = ");
  Debug_println(mac_add);

  // connect to the main server
  Debug_print("Server:");

  // timout for master server //
  master_Client.setTimeout(200);

  // check connection to master and wifi status //
  if (WiFi.status() == 3 && master_Client.connect(master_IP, master_Port))
  {
    Debug_println("Connected");

    //turn on LED for alnnomate slave...
    digitalWrite(WiFi_LED, HIGH);
  }
  else
  {
    Debug_println("Failed");
  }

  Debug_println("ESP READY");
  // Flush any data in the Serial buffer //
  serialFlush();          // empty Serial0 buffer
  received_string = "";   // empty the master_Client buffer
}

/**************************************************************** SLAVE USER DEFINED FUNCTIONS**************************************************************************************************/
// Empty Serail buffer form serial0
void serialFlush()
{
  while (Serial.available() > 0)
  {
    char t = Serial.read();
  }
}
void printWifiStatus()
{
  Debug_print("SSID: ");
  Debug_println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Debug_print("IP Address: ");
  Debug_println(ip);

  long rssi = WiFi.RSSI();

  Debug_print("Signal strength (RSSI):");
  Debug_print(rssi);
  Debug_println(" dBm");
  WiFi.macAddress(mac);
  sprintf(mac_add, "%02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
}
/*
   Convert the IP address object in to a sting like 192.168.0.105
*/
String ipToString(IPAddress IP)
{
  String ipString = "";
  ipString += IP[0];
  ipString += ".";
  ipString += IP[1];
  ipString += ".";
  ipString += IP[2];
  ipString += ".";
  ipString += IP[3];

  return ipString;
}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/


/******************************************************************* PRINTER CONTROLLER USER DEFINED FUNCTIONS **************************************************************************************************/
/* =================================================================

                   Web Browser Parsing......

   =================================================================*/
void webServerHandler (void)
{
  int iter = 0, k = 0, loop1 = 0;
  signed int index = 0;
  long i = 0;
  static char parseStr[500];

  //Check whether server is available or not
  client = server.available();

  /******************************************************************************************/
  //client is availablenpage.php?var_pump1=+0&var_pump2=+0&extend=0&prod1=&prod2=&prod3=&prod4=&prod5=&prod6=&pump_name=0&pump_add=0&date=&time=&printer_mode=1 HTTP/1.1
  if (client)
  {
    //read the data util the \r
    serverString = client.readStringUntil('\r');
    client.flush();
  }
  //First Page
  /******************************************************************************************/
  index = serverString.indexOf ("GET / HTTP/1.1");                                                   // Tell the starting position of that string...
  if (index >= 0)
  {
    //Get Parameters stored in USB.....
    receiveParameters_STM32();

    //Send the parameter page........
    send_DataToServer();

    serverString = "";               //Clears the string
    index = -1;
  }

  //Second Page
  /******************************************************************************************/
  index = serverString.indexOf ("GET /actionpage.php?"); // if this is in the string
  if (index >= 0)
  {
    String tempVal;
    int tempIndex;

    //Copies the content to the temp buf
    serverString.toCharArray(parseStr,  serverString.length() + 1);
    //Print the data on console.
    //Serial.print ("\r\nparseStr:"); Serial.println (parseStr);
    //Serial.print("\x04#");
    //GET /actionpage.php?var_pump1=7&var_pump2=8&extend=1&prod1=hobc&prod2=hi-super&prod3=diesel&prod4=kerosene&prod5=leaded+high+octane&prod6=Leaded+Regular&pump_name=ROSHAN+Service+Station&pump_add=Plot+A-625+Block-L+North+Nazimabad%2C+Karachi&date=2019-04-10&time=05%3A57&printer_mode=0 HTTP/1.1

    /*************          device id                  *****************************/
    for (i = 0; parseStr[i] != '='; i++); i++;

    for (k = 0; parseStr[i] != '&'; k++)
    {
      params.device[k] = parseStr[i++];
    }
    params.device[k] = 0;

    /*************          Parse Right Dispenser ID    *****************************/

    for (i = i ; parseStr[i] != '='; i++); i++;

    for (k = 0; parseStr[i] != '&'; k++)
    {
      params.RightDispenserID[k] = parseStr[i++];
    }
    params.RightDispenserID[k] = 0;

    /*************          Parse Left Dispenser ID    *****************************/
    for (i = i; parseStr[i] != '=' ; i++);  i++;

    for (k = 0; parseStr[i] != '&' ; k++)
    {
      params.LeftDispenserID[k] = parseStr[i++];
    }
    params.LeftDispenserID[k]  = 0;

    /*************          Parse Mobile Number    *****************************/
    for (i = i; parseStr[i] != '=' ; i++);  i++;

    for (k = 0; parseStr[i] != '&' ; k++)
    {
      params.MobileNumber[k] = parseStr[i++];
    }
    params.MobileNumber[k]  = 0;

    /*************          Parse server    *****************************/
    for (i = i; parseStr[i] != '=' ; i++);  i++;

    for (k = 0; parseStr[i] != '&' ; k++)
    {
      params.server[k] = parseStr[i++];
    }
    params.server[k]  = 0;
    /*************          Parse email   *****************************
      for (i=i; parseStr[i] != '=' ; i++);  i++;

      for (k=0; parseStr[i] != '&' ; k++)
      {
       params.email[k] = parseStr[i++];
      }
      params.email[k]  = 0;

      /*************          battry type  *****************************/

    for (i = i; parseStr[i] != '=' ; i++);  i++;

    for (k = 0; parseStr[i] != '&' ; k++)
    {
      params.battery_type[k] = parseStr[i++];
    }
    params.battery_type[k]  = 0;
    /*************          generator battery *****************************/

    for (i = i; parseStr[i] != '=' ; i++);  i++;

    for (k = 0; parseStr[i] != '&' ; k++)
    {
      params.generator_battery[k] = parseStr[i++];
    }
    params.generator_battery[k]  = 0;

    /*************          Parse Extend              *****************************
    for (i = i; parseStr[i] != '=' ; i++);  i++;

    for (k = 0; parseStr[i] != '&' ; k++)
    {
      params.extend[k] = parseStr[i++];
    }
    params.extend[k]  = 0;
    k = 0;

    /* ************           Parse prod              **************************** */
  /*  for (loop1 = 0; loop1 <= 5; loop1++)
    {
      for (i = i; (parseStr[i] != '='); i++);  i++;
      for (i = i; (parseStr[i] != '&'); i++)
      {
        //Max Limit is product limit is 15....
        if (k <= 15)
        {
          params.prod[loop1][k++] = parseStr[i];
        }
      }
      params.prod[loop1][k]  = 0;
      k = 0;
    }*/

    /*************           Pump name              *****************************/
    for (i = i; parseStr[i] != '=' ; i++);  i++;
    for (k = 0; parseStr[i] != '&'; k++)
    {
      params.pump_name[k] = parseStr[i++];
    }

    params.pump_name[k] = 0;

    /*************           Pump address           *****************************/
    // HTTP Request will not give the packet.....

    //for (i=i; parseStr[i] != '=' ; i++);  i++;
    //for (k=0; parseStr[i] != '&'; k++)
    //{
    //params.pump_address[k] = parseStr[i++];
    //}
    //params.pump_address[k] = 0;

    /*************           Date                 *****************************/
    for (i = i; parseStr[i] != '=' ; i++);  i++;
    for (k = 0; parseStr[i] != '&' ; k++)
    {
      params.date[k] = parseStr[i++];
    }
    params.date[k] = 0;

    /*************           Time                 *****************************/

    for (i = i; parseStr[i] != '=' ; i++);  i++;
    for (k = 0; parseStr[i] != '&' ; k++)
    {
      params.Time[k] = parseStr[i++];
    }

    params.Time[k] = 0;

    /*************           Printer mode           ***************************/

    for (i = i; parseStr[i] != '=' ; i++);  i++;
    for (k = 0; parseStr[i] != '&' ; k++)
    {
      params.printer_mode[k] = parseStr[i++];
    }
    params.printer_mode[k] = 0;

    /*************           Dual mode           *****************************/

    for (i = i; parseStr[i] != '=' ; i++);  i++;
    for (k = 0; parseStr[i] != '&' ; k++)
    {
      params.dual_mode[k] = parseStr[i++];
    }
    params.dual_mode[k] = 0;
    /*************           Decimal Point           *****************************/
    for (i = i; parseStr[i] != '=' ; i++);  i++;
    for (k = 0;  ((parseStr[i] != 'H') && (parseStr[i + 1] != 'T') && (parseStr[i + 2] != 'T') && (parseStr[i + 3] != 'P')) ; k++)
    {
      params.deci_point[k] = parseStr[i++];
    }
    params.deci_point[k] = 0;
    /*
                     |=======================================|
                     |Decode parameters of Printer controller|
                     |=======================================|
    */
    urldecode2(params.device, params.device);
    //urldecode2(params.email,params.email);
    urldecode2(params.RightDispenserID, params.RightDispenserID);
    urldecode2(params.LeftDispenserID, params.LeftDispenserID);
    urldecode2(params.MobileNumber, params.MobileNumber);
    urldecode2(params.server, params.server);
    //urldecode2(params.battery_type,params.battery_type);
    urldecode2(params.generator_battery, params.generator_battery);
  //  urldecode2(params.extend, params.extend);
    urldecode2(params.pump_name, params.pump_name);
    //urldecode2(params.pump_address,params.pump_address);
    urldecode2(params.date, params.date);
    urldecode2(params.Time, params.Time);
   /* for (k = 0; k <= 5; k++)
    {
      //  Serial.print ("\r\nBD_Prod");Serial.print(k);Serial.print(":");Serial.println (&params.prod[k][0]);
      urldecode2(&params.prod[k][0], &params.prod[k][0]);
      // Serial.print ("\r\nAD_Prod");Serial.print(k);Serial.print(":");Serial.println (&params.prod[k][0]);

    }*/
    urldecode2(params.printer_mode, params.printer_mode);

   /* Serial.print("\r\nDevice");           Serial.print(params.device);
    Serial.print("\r\nRightDispenserID"); Serial.print(params.RightDispenserID);
    Serial.print("\r\nLeftDispenserID");  Serial.print(params.LeftDispenserID);
    Serial.print("\r\nMobileNumber");     Serial.print(params.MobileNumber);*/


    //Send Packet to STM32
    buildConfiguratorPacket();
    //??? Read acknowledgement...
    /*^^^^^^^^^^^^^^^^^^^^^^^Server Response^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
    String s = "HTTP/1.1 200 OK\r\n";
    s += "Content-Type: text/html\r\n\r\n";
    s += "<!DOCTYPE HTML>\r\n<html>\r\n";
    s += "<html><head><title>ALnoMate Configurator</title></head><body>";
    s += "<h5><center>PARAMETER ARE SET</h5>";
    client.print(s);
    index = -1;                       //To prevent from sending the data
    serverString = "";                //Clears the string
  }
  // These are not php files
  index = serverString.indexOf ("GET /pump1_resp.php?"); // if this is in the string
  if (index >= 0)
  {
    char respo[20];
    strcpy (respo, "10");
    strcat (respo, "\x03");
    strcat (respo, "\x04");
    strcat (respo, "#");
    Serial.print(respo);
    /*^^^^^^^^^^^^^^^^^^^^^^^Server Response^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
    String s = "HTTP/1.1 200 OK\r\n";
    //s += "Content-Type: text/html\r\n\r\n";
    //s += "<!DOCTYPE HTML>\r\n<html>\r\n";
    client.print(s);
    index = -1;                       //To prevent from sending the data
    serverString = "";                //Clears the string
  }
  index = serverString.indexOf ("GET /pump2_resp.php?"); // if this is in the string
  if (index >= 0)
  {
    char respo[20];
    strcpy (respo, "11");
    strcat (respo, "\x03");
    strcat (respo, "\x04");
    strcat (respo, "#");
    Serial.print(respo);
    /*^^^^^^^^^^^^^^^^^^^^^^^Server Response^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
    String s = "HTTP/1.1 200 OK\r\n";
    //s += "Content-Type: text/html\r\n\r\n";
    //s += "<!DOCTYPE HTML>\r\n<html>\r\n";
    client.print(s);
    index = -1;                       //To prevent from sending the data
    serverString = "";                //Clears the string
  }
  index = serverString.indexOf ("GET /clear_resp1.php?"); // if this is in the string
  if (index >= 0)
  {
    char respo[20];
    strcpy (respo, "12");
    strcat (respo, "\x03");
    strcat (respo, "\x04");
    strcat (respo, "#");
    Serial.print(respo);
    /*^^^^^^^^^^^^^^^^^^^^^^^Server Response^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
    String s = "HTTP/1.1 200 OK\r\n";
    s += "Content-Type: text/html\r\n\r\n";
    //s += "<!DOCTYPE HTML>\r\n<html>\r\n";
    client.print(s);
    /*
      String long_resp = "This is longresponse"; // 20 in length
      for(long int i=0;i<100000;i++)
      {
      client.print(long_resp);
      }
    */
    index = -1;                       //To prevent from sending the data
    serverString = "";                //Clears the string
  }
  index = serverString.indexOf ("GET /clear_resp2.php?"); // if this is in the string
  if (index >= 0)
  {
    char respo[20];
    strcpy (respo, "13");
    strcat (respo, "\x03");
    strcat (respo, "\x04");
    strcat (respo, "#");
    Serial.print(respo);
    /*^^^^^^^^^^^^^^^^^^^^^^^Server Response^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
    String s = "HTTP/1.1 200 OK\r\n";
    s += "Content-Type: text/html\r\n\r\n";
    //s += "<!DOCTYPE HTML>\r\n<html>\r\n";
    client.print(s);
    /*
      String long_resp = "This is longresponse"; // 20 in length
      for(long int i=0;i<100000;i++)
      {
      client.print(long_resp);
      }
    */
    index = -1;                       //To prevent from sending the data
    serverString = "";                //Clears the string
  }
}
/**************************************************************************************************************************************************************************************************/

/*
  |=================================================================================|
  |                                                                                 |
  |                         Send Data To Server                                     |
  |                                                                                 |
  |================================================================================ |
  class WiFi_Configurator
{
  public:
    char RightDispenserID[30], LeftDispenserID[30], device[12], email[40], MobileNumber[15], battery_type[3], generator_battery[7], extend[3], prod[6][20], pump_name[60], date[20], Time[20], printer_mode[3], dual_mode[3], deci_point[3];

} params;
  
  
  
  */
void send_DataToServer(void)
{
  String ptr = "";
  int  k = 0, j = 0;

  ptr +=  "<html><head><body>";
  //ptr +=  "<h1 style=\"background-color: #00134d; color:White;\"><center>Alnno Printer Configurator</center></h1>";
  ptr +=  "<h1><center><u>Generator Monitoring System Configurator</u></center></h1>";
  ptr +=  "<form action=\"actionpage.php\" method=\"get\" target=\"blank\">";
  ptr +=  "<body>\n";

  ptr += "<b>Device ID:<input maxlength = \"12\" type=\"text\" value=\"";
  for (k = 0; params.device[k] != 0; k++)
  {
    ptr +=  params.device[k];
  }

  ptr +=  "\" name=\"device\"><br><br>";


  ptr += "Router SSID:<input maxlength = \"30\" type=\"text\" value=\"";

  for (k = 0; params.RightDispenserID[k] != 0; k++)
  {
    ptr +=  params.RightDispenserID[k];
  }

  ptr +=  "\" name=\"var_pump1\"><br><br>";

  ptr += "Router Password:<input maxlength = \"30\" type=\"text\" value=\"";
  for (k = 0; params.LeftDispenserID[k] != 0; k++)
  {
    ptr +=  params.LeftDispenserID[k];
  }

  ptr +=  "\" name=\"var_pump2\"><br><br>";
  /****************************mobile number for send sms**************************************************************/

  ptr += "Server Mobile Number:<input maxlength = \"15\" type=\"text\" value=\"";
  for (k = 0; params.MobileNumber[k] != 0; k++)
  {
    ptr +=  params.MobileNumber[k];
  }

  ptr +=  "\" name=\"MobileNumber\"><br><br>";
/*******************************server Type*****************************************************/

   ptr += "Server:<select style=\"color=red\" name=\"server\">";
  ptr += "<option value=\"0\"";
  if (strcmp(params.server, "0") == 0)
    ptr += " selected=\"select\"";
  ptr += ">Wifi</option>";
  ptr += "<option value=\"1\"";
  if (strcmp(params.server, "1") == 0)
    ptr += " selected=\"select\"";
  ptr += ">GSM</option>";
  ptr += "</select><br><br>";

  /****************************mobile number for send sms**************************************************************

    ptr += "E-Mail ID:<input maxlength = \"40\" type=\"text\" value=\"";
    for (k=0; params.email[k] != 0; k++)
    {
    ptr +=  params.email[k];
    }

    ptr +=  "\" name=\"email\"><br><br>";


    /****************************battery type******************************************************/
  ptr += "Battery Type:<select style=\"color=red\" name=\"battery_type\">";
  ptr += "<option value=\"0\"";
  if (strcmp(params.battery_type, "0") == 0)
    ptr += " selected=\"select\"";
  ptr += ">12V</option>";
  ptr += "<option value=\"1\"";
  if (strcmp(params.battery_type, "1") == 0)
    ptr += " selected=\"select\"";
  ptr += ">24V</option>";
  ptr += "</select><br><br>";

  /****************************generator battery******************************************************/
  ptr += "Generator Battery:<select style=\"color=red\" name=\"generator_battery\">";
  ptr += "<option value=\"1\"";
  if (strcmp(params.generator_battery, "1") == 0)
    ptr += " selected=\"select\"";
  ptr += ">Enable</option>";
  ptr += "<option value=\"0\"";
  if (strcmp(params.generator_battery, "0") == 0)
    ptr += " selected=\"select\"";
  ptr += ">Disable</option>";
  ptr += "</select><br><br>";

  /*********************************************************************************************

  ptr +=  "Extend:<select name=\"extend\"> <option value = \"";

  //  if (strcmp (params.dual_mode,"1") == 0)
  if (strcmp (params.extend, "0") == 0)
  {
    ptr += "0\">No</option> <option value = \"1\">Yes</option></select><br><br>";
  }
  else
  {
    ptr += "1\">Yes</option> <option value = \"0\">No</option></select><br><br>";
  }

/*  for (int iter = 0 ; iter <= 5 ; iter++ )
  {
    ptr += "Product";
    ptr += iter + 1;
    ptr += ":<input maxlength = \"15\" type=\"text\" pattern=\"[^\x21-\x2A,\x2E,\x2F,^\x7B-\x7E]+\" title=\"Invalid Input\" value=\"";

    for (k = 0; params.prod[iter][k] != 0; k++)
    {
      ptr += params.prod[iter][k];
    }

    ptr += "\" name=\"prod";
    ptr += iter + 1;
    ptr += "\"><br><br>";
  }*/

  ptr += "Pump Name:<input maxlength = \"80\" type=\"text\" value=\"";
  for (k = 0 ; params.pump_name[k] != 0; k++)
  {
    ptr += params.pump_name[k];
  }
  ptr += "\" name=\"pump_name\"><br><br>";

  /************  Pump address ************************/
  //Don't take Pump Address from User

  // ptr += "Pump Address:<input maxlength = \"50\" type=\"text\" value=\"";
  // for (k=0; params.pump_address[k] !=0; k++)
  //{
  //   ptr += params.pump_address[k];
  // }
  // ptr += "\" name=\"pump_add\"><br><br>";

  /************  Date ************************/
  ptr += "Date:<input type=\"date\" value=\"";
  for (k = 0 ; params.date[k] != 0; k++)
  {
    ptr += params.date[k];
  }
  ptr += "\" name=\"date\"><br><br>";

  /************  Time ************************/
  ptr += "Time:<input type=\"time\" value=\"";

  for (k = 0; params.Time[k] != 0; k++)
  {
    ptr += params.Time[k];
  }
  ptr += "\" name=\"time\"><br><br>";


  /************  Printer Mode ************************/
  ptr += "Printer_mode:<select name=\"printer_mode\">";
  // option with selection logic
  // standalone->1
  ptr += "<option value = \"1\"";
  ptr += ">Standalone</option>";
  // otherFCC->2
  ptr += "<option value = \"2\"";
  if (strcmp (params.printer_mode, "2") == 0)
    ptr += "selected=\"selected\"";
  ptr += ">otherFCC</option>";
  // Slave->0
  ptr += "<option value = \"0\"";
  if (strcmp (params.printer_mode, "0") == 0)
    ptr += "selected=\"selected\"";
  ptr += ">Slave</option>";
  ptr += "</select><br>";
  /************  Dual Mode ************************/
  ptr += "<br>Dual Mode:<select style=\"color=red\" name=\"dual_printer\"> ";
  ptr += "<option value=\"1\"";
  if (strcmp (params.dual_mode, "1") == 0)
    ptr += " selected=\"select\"";
  ptr += ">Enable</option>";

  ptr += "<option value=\"0\"";
  if (strcmp (params.dual_mode, "0") == 0)
    ptr += " selected=\"select\"";
  ptr += ">Disable</option>";
  ptr += "</select><br>";

  ptr += "<br>Decimal Point:<select style=\"color=red\" name=\"deci_point\">";
  ptr += "<option value=\"0\"";
  if (strcmp(params.deci_point, "0") == 0)
    ptr += " selected=\"select\"";
  ptr += ">1</option>";
  ptr += "<option value=\"1\"";
  if (strcmp(params.deci_point, "1") == 0)
    ptr += " selected=\"select\"";
  ptr += ">2</option>";
  ptr += "</select>";
  /*----------------------------------------------------------------------------*/

  ptr += "<center><input type=\"submit\" value=\"Submit\"></center></b>";
  ptr +=  "</form>\n";
  // End of Form 1
  ptr +=  "<form action=\"pump1_resp.php\" method=\"get\" target=\"Pump 1\">"; // start of second form
  ptr += "<center><input type=\"submit\" value=\"pump1\"></center></b>";
  ptr +=  "</form>";
  ptr +=  "<form action=\"pump2_resp.php\" method=\"get\" target=\"Pump 2\">"; // start of second form
  ptr += "<center><input type=\"submit\" value=\"pump2\"></center></b>";
  ptr +=  "</form>";
  ptr +=  "<form action=\"clear_resp1.php\" method=\"get\" target=\"Clear1\">"; // start of second form
  ptr += "<center><input type=\"submit\" value=\"Clear1\"></center></b>";
  ptr +=  "</form>";
  ptr +=  "<form action=\"clear_resp2.php\" method=\"get\" target=\"Clear2\">"; // start of second form
  ptr += "<center><input type=\"submit\" value=\"Clear2\"></center></b>";
  ptr +=  "</form>";
  // End of page
  ptr +=  "</body>\n";
  ptr +=  "</html>\n";
  client.print (ptr);
}


/* ====================================================|
  |                                                     |
  |                Acquire Parameters from STM          |
  |                                                     |
  | ====================================================*/
void receiveParameters_STM32(void)
{
  String rcv_str_mcu;
  static char STM32USB_Buffer[500];
  int loop1 = 0;
  bool validityStat;

  while (loop1 <= 4)
  {
    loop1++;

    //Send get_usbData[04]#
    Serial.print ("get_usbData\x04\x23");
    rcv_str_mcu = Serial.readStringUntil ('#');//Read till #  1000ms timeout default
    if (rcv_str_mcu != "") // not empty
    {
      //Serial.println ("\r\nStringUntil:"); Serial.print (rcv_str_mcu);
      rcv_str_mcu.toCharArray(STM32USB_Buffer, rcv_str_mcu.length() + 1);     //Convert String type to array
      Serial.flush();                                                         //Flush my buffer
      //Serial.print ("\r\n Data:"); Serial.print (STM32USB_Buffer);

      validityStat = packetValidation (STM32USB_Buffer);
      if (validityStat ==  true)
      {
        parseUSB_Data(STM32USB_Buffer);                                     //Got the string from USB
        rcv_str_mcu = "";
        break;
      }

      rcv_str_mcu = "";
      delay (200);
    }
    else
    {
      delay (120);
    }

  }
}

/*
   ===================================================


       This function check the protocol of the packet send
       by STM32 . It checks
       for the number of ETX in the packet....

       if equal to 15 then return true
       otherwise false.

       in false case ESP must reinquire STM32.




  ====================================================   */

bool packetValidation (char str[])
{
  int len, count = 0;
  bool flag = false;

  len = strlen (str);

  //Scan of number of ETX must be = 15
  /*********************************/
  for (int i = 0; i <= len ; i++)
  {
    if (str[i] == '\x03')
    {
      count++;
    }
  }
  /********************************/
  if (count == 15)
  {
    flag = true;
  }
  else
  {
    flag = false;
  }

  //  Serial.print ("\r\nCount:"); Serial.print (count); Serial.print("\r\n");
  return flag;
}

/*| ==================================== |
  |                                      |
  |        PARSE USB DATA OF STM32       |
  |                                      |
  | ==================================== |*/
// char RightDispenserID[5],LeftDispenserID[5],extend[3],prod[6][20],pump_name[60],pump_address[80],date[20],Time[20],printer_mode[3];
// 02[03]0[03]1[03]2[03]1[03]hobc[03]hi-super[03]diesel[03]kerosene[03]leaded high octane[03]Leaded Regular[03]11:17[03]2019-04-19[03]0[03]ROSHAN Service Station[03]Plot A-625 Block-L North Nazimabad, Karachi[04]
void parseUSB_Data (char stm32usbString[])
{
  int j = 0, k = 0, loop1 = 0;

  for (j = 0; stm32usbString[j] != '\x03'; j++); j++;

  //Get PRINTER MODE
  for (j = j; stm32usbString[j] != '\x03'; j++) {
    params.printer_mode[k++] = stm32usbString[j];
  }
  params.printer_mode[k] = 0; j++; k = 0;

  //get deivce id

  for (j = j; stm32usbString[j] != '\x03'; j++) {
    params.device[k++] = stm32usbString[j];
  }
  params.device[k] = 0; j++; k = 0;


  //Get Right dispenser ID
  for (j = j; stm32usbString[j] != '\x03'; j++) {
    params.RightDispenserID[k++] = stm32usbString[j];
  }
  params.RightDispenserID[k] = 0; j++; k = 0;

  //Get Left dispenser ID
  for (j = j; stm32usbString[j] != '\x03'; j++) {
    params.LeftDispenserID[k++] = stm32usbString[j];
  }
  params.LeftDispenserID[k] = 0; j++; k = 0;
  /*FOR MOBILE NUMBER*********************************************************************************/
  for (j = j; stm32usbString[j] != '\x03'; j++) {
    params.MobileNumber[k++] = stm32usbString[j];
  }
  params.MobileNumber[k] = 0; j++; k = 0;

 /*FOR server*********************************************************************************/
  for (j = j; stm32usbString[j] != '\x03'; j++) {
    params.server[k++] = stm32usbString[j];
  }
  params.server[k] = 0; j++; k = 0;


  //get deivce id

  /*for (j=j; stm32usbString[j]!='\x03';j++){
        params.email[k++] = stm32usbString[j];
    }
    params.email[k] = 0; j++; k=0;*/

  /*FOR BATTERY TYPE*********************************************************************************/
  for (j = j; stm32usbString[j] != '\x03'; j++) {
    params.battery_type[k++] = stm32usbString[j];
  }
  params.battery_type[k] = 0; j++; k = 0;

  /*FOR Generator battery *********************************************************************************/
  for (j = j; stm32usbString[j] != '\x03'; j++) {
    params.generator_battery[k++] = stm32usbString[j];
  }
  params.generator_battery[k] = 0; j++; k = 0;


  /************************************************************************************
  //Get Extend
  for (j = j; stm32usbString[j] != '\x03'; j++) {
    params.extend[k++] = stm32usbString[j];
  }
  params.extend[k] = 0; j++; k = 0;

  //Get Products
  /*for (loop1 = 0; loop1 <= 5; loop1++)
  {
    //Get Products
    for (j = j; stm32usbString[j] != '\x03'; j++)
    {
      params.prod[loop1][k++] = stm32usbString[j];
    }
    params.prod[loop1][k] = 0; j++; k = 0;
  }*/
   //Get pump name
  for (j = j; stm32usbString[j] != '\x04'; j++) {
    params.pump_name[k++] = stm32usbString[j];
  }
  params.pump_name[k] = 0; j++; k = 0;

  //Get Time
  for (j = j; stm32usbString[j] != '\x03'; j++) {
    params.Time[k++] = stm32usbString[j];
  }
  params.Time[k] = 0; j++; k = 0;

  //Get date
  for (j = j; stm32usbString[j] != '\x03'; j++) {
    params.date[k++] = stm32usbString[j];
  }
  params.date[k] = 0; j++; k = 0;

  //Get dual mode
  for (j = j; stm32usbString[j] != '\x03'; j++) {
    params.dual_mode[k++] = stm32usbString[j];
  }
  params.dual_mode[k] = 0; j++; k = 0;

  //Get deci_point
  for (j = j; stm32usbString[j] != '\x03'; j++) {
    params.deci_point[k++] = stm32usbString[j];
  }
  params.deci_point[k] = 0; j++; k = 0;

 
}

/*
  |=================================================================================|
  |                                                                                 |
  |                         Send Packet To STM32                                    |
  |                                                                                 |
  |================================================================================ |

  0121Hi-SuperHOBC ExcelliumDieselkerosenelead keroseneLeaded Regular11:422019-04-130Roshan Service Center752 Blocù
*/
//<Printer_Mode>,<RightDispenserID>,<LeftDispenserID>,<Extended_mode>,<Prod1>,<Prod2>,<Prod3>,<Prod4>,<Prod5>,<Prod6>,<time>,<date>,<dualmode>,<deci_point>,<Pump_Name>
void buildConfiguratorPacket (void)
{
  char packet[packlen + 1];
  int i = 0;

  strcpy (packet, "02");
  strcat (packet, "\x03");
  strcat (packet, params.printer_mode);
  strcat (packet, "\x03");
  strcat (packet, params.device);
  strcat (packet, "\x03");
  strcat (packet, params.RightDispenserID);
  strcat (packet, "\x03");
  strcat (packet, params.LeftDispenserID);
  strcat (packet, "\x03");
  strcat (packet, params.MobileNumber);
  strcat (packet, "\x03");
  strcat (packet, params.server);
  strcat (packet, "\x03");
  /* strcat (packet, params.email);
    strcat (packet, "\x03");*/
  strcat (packet, params.battery_type);     //Dual Mode
  strcat (packet, "\x03");
  strcat (packet, params.generator_battery);     //Dual Mode
  strcat (packet, "\x03");
   strcat (packet, params.pump_name);
  strcat (packet, "\x03");
/*  strcat (packet, params.extend);
  strcat (packet, "\x03");

  /*for (i = 0; i <= 5 ; i++)
  {
    strcat (packet, &params.prod[i][0]);
    strcat (packet, "\x03");
  }*/
  
  strcat (packet, params.date);
  strcat (packet, "\x03");
  strcat (packet, params.Time);
  strcat (packet, "\x03");
  strcat (packet, params.dual_mode);     //Dual Mode
  strcat (packet, "\x03");
  strcat (packet, params.deci_point);     //Dual Mode
  strcat (packet, "\x03");
  strcat (packet, "0");
  strcat (packet, "\x04"); // end
  strcat (packet, "#");
  Serial.print (packet);
}


/*====================================================

           URL  DECODE 2

   ==================================================*/
void urldecode2(char* dst, char* src)
{
  char a, b;
  while (*src != '\0')
  {
    //percent condition
    if ( (*src == '%') && ((a = src[1]) && (b = src[2])) && (isxdigit(a) && isxdigit(b)) )
    {
      if (a >= 'a')
        a -= 'a' - 'A';
      if (a >= 'A')
        a -= ('A' - 10);
      else
        a -= '0';
      if (b >= 'a')
        b -= 'a' - 'A';
      if (b >= 'A')
        b -= ('A' - 10);
      else
        b -= '0';
      *dst++ = 16 * a + b;
      src += 3;
    }
    //space condition
    else if (*src == '+')
    {
      *dst++ = ' ';
      src++;
    }
    else
    {
      *dst++ = *src++;
    }
  }
  *dst++ = '\0';
}
