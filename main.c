
#include "stm32f4xx.h"
#include "my_arduino.h"
#include "myStringLib.h"
#include "tm_stm32f4_spi.h"
#include "defines.h"
#include "tm_stm32f4_gpio.h"
#include "ENC28J602.h"
#include "EtherShield.h"
#include "test_file.h"
#include "stdbool.h"


#define WEBSERVER_VHOST "somewebsit.com"
//#define WEBSERVER_VHOST "facebook.com" // 157.240.227.35
// API URL to send request to
#define API_URL "http://somewebsit.com/datasend.php/"
// aditional header
#define BLOGGACCOUNT "Authorization: Basic xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
// LED define
#define LED_PORT GPIOB
#define LED_PIN GPIO_Pin_9
//global variables
uint8_t spiData[2];
uint8_t data8;
uint16_t data16;
uint16_t plen, dat_p;
uint8_t start_web_client=1;
uint16_t posPing;
uint32_t lastPingtime;
typedef enum Dns_States_en{
  DNS_IDLE,
  DNS_REQUEST,
  DNS_RECEIVED} Dns_States_en;
Dns_States_en Dns_States = DNS_IDLE;
int8_t dns_state=-1;

uint8_t ARP_req[42]={
  0xff,0xff,0xff,0xff,0xff,0xff, // Destination MAC: ff:ff:ff:ff:ff:ff
  0x74,0x69,0x69,0x2d,0x30,0x36, // Source MAC: 74:69:69:2d:30:36
  0x08,0x06, // Ethertype
  0x00,0x01, // HTYPE
  0x08,0x00, // PTYPE
  0x06, // HLEN
  0x04, // PLEN
  0x00,0x01, // OPER
  0x74,0x69,0x69,0x2d,0x30,0x36, // Sender Mac: 74:69:69:2d:30:36
  0xc0,0xa8,0x45,0x6E, // Sender IP: 192.168.69.110
  0x00,0x00,0x00,0x00,0x00,0x00, // Targer MAC: 00:00:00:00:00:00
  0xc0,0xa8,0x45,0x01 // Targer IP: 192.168.69.1
};

// please modify the following two lines. mac and ip have to be unique
// in your local area network. You can not have the same numbers in
// two devices:

static uint8_t MacAddress[6] = {0x54,0x55,0x58,0x10,0x00,0x25};

static uint8_t myip[4] = {
  192,168,69,110};

static uint8_t destip[4] = {8,8,8,8};

// The mac address of the PC you want to wake u
static uint8_t wolmac[6] = {
  0x48,0x0F,0xCF,0x4B,0x60,0x15}; // 48-0F-CF-4B-60-15
// gate way ip
static uint8_t gwip[4] = { 192,168,69,1};
// This ip is updated via dsn
static uint8_t serverIp[4] = {0,0,0,0}; //computer IP //162.214.184.38
// select the demo that you want to run
enum demo_n{
  Server,
  Client_viaDNS,
  Client_viaIP,
  Ping,
  WakeOnLan
};
const enum demo_n demo=Ping;
#define MYWWWPORT 80
#define BUFFER_SIZE 1500
static uint8_t buf[BUFFER_SIZE+1];
// global string buffer for twitter message:
static char statusstr[150];

uint16_t http200ok(void)
{
  return(ES_fill_tcp_data(buf,0,"HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nPragma: no-cache\r\n\r\n"));
}



int count=0;
uint32_t millis_counter=0;
uint32_t lastUpdate = 0;
uint32_t lastBlink=0;
uint32_t millis()
{
  return millis_counter;
}
void delay(uint32_t ms)
{
  uint32_t last_ms;
  last_ms = millis();
  while(millis()-last_ms < ms)
  {
    // do nothng
  }
}
void SysTick_Handler(void) {
  millis_counter++;
}

void browserresult_callback(uint8_t statuscode,uint16_t datapos,uint16_t datalen)
{
  if (statuscode==0){
    //    LEDOFF;
    // copy the "Date: ...." as returned by the server
    asm("NOP");
  }
  // clear pending state at sucessful contact with the
  if (start_web_client==2){
    start_web_client=3;
  }
}

void browserresult_callback2(uint8_t statuscode,uint16_t a,uint16_t b)
{
  if (statuscode==0){
    //    LEDOFF;
    // copy the "Date: ...." as returned by the server
    asm("NOP");
  }
  
  asm("NOP");
}

int main()
{
  SystemInit();
  SysTick_Config(SystemCoreClock / 1000);
  
  delay(100);
  TM_GPIO_Init(LED_PORT,LED_PIN,TM_GPIO_Mode_OUT ,TM_GPIO_OType_OD,TM_GPIO_PuPd_DOWN ,TM_GPIO_Speed_Low);
  // ethernet module initilization
  ENC28J60_GPIO_Configuration();	
  ENC28J60_SPI1_Configuration();
  enc28j60Init(MacAddress);															//open source api
  enc28j60clkout(2);	
  // init the ethernet/ip layer:
  ES_init_ip_arp_udp_tcp(MacAddress,myip, MYWWWPORT);
  
  
  if(demo == Client_viaDNS)
  {
    // init the web client:
    ES_client_set_gwip(gwip);  // e.g internal IP of dsl router
    //ES_client_tcp_set_serverip(serverIp);
  }
  if(demo==Client_viaIP)
  {
    serverIp[0] = 162;
    serverIp[1] =214;
    serverIp[2] = 184;
    serverIp[3] =   38; //computer IP //162.214.184.38
    // init the web client:
    ES_client_set_gwip(gwip);  // e.g internal IP of dsl router
    ES_client_tcp_set_serverip(serverIp);
  }
  if(demo==Ping)
  {
    ES_client_set_gwip(gwip);  // e.g internal IP of dsl router
  }
  delay(1000);
  // using funciton
  while (1)
  {
    // blink
    if( millis() > lastBlink+150)
    {
      lastBlink = millis();
      TM_GPIO_TogglePinValue(LED_PORT,LED_PIN);
    }
    if(demo == Server)
    {
      /* user code here */
      //ENC28_packetSend(42, ARP_req);
      //Delayms(2000);
      //data8 = enc28j60linkup();
      //TM_GPIO_TogglePinValue(GPIOB,GPIO_Pin_9);
      
      dat_p=ES_packetloop_icmp_tcp(buf,ES_enc28j60PacketReceive(BUFFER_SIZE, buf));
      
      /* dat_p will be unequal to zero if there is a valid 
      * http get */
      if(dat_p==0){
        // no http request
      }
      else
      {
        count++;
        // tcp port 80 begin
        if (strncmp("GET ",(char *)&(buf[dat_p]),4)!=0){
          // head, post and other methods:
          dat_p=http200ok();
          dat_p=ES_fill_tcp_data(buf,dat_p,"<h1>200 OK</h1>");
          ES_www_server_reply(buf,dat_p); // send web page data
        }
        // just one web page in the "root directory" of the web server
        else if (strncmp("/ ",(char *)&(buf[dat_p+4]),2)==0){
          dat_p=http200ok();
          dat_p=ES_fill_tcp_data(buf,dat_p,test_file_html_1);
          ES_www_server_reply(buf,dat_p); // send web page data
        }
        else if (strncmp("/mystyle.css",(char *)&(buf[dat_p+4]),12)==0)
        {
          dat_p=http200ok();
          dat_p=ES_fill_tcp_data(buf,dat_p,test_file_css);
          ES_www_server_reply(buf,dat_p); // send web page data
        }
        else{
          dat_p=ES_fill_tcp_data(buf,0,"HTTP/1.0 401 Unauthorized\r\nContent-Type: text/html\r\n\r\n<h1>401 Unauthorized</h1>");
          ES_www_server_reply(buf,dat_p); // send web page data
        }
        // tcp port 80 end
      }
    }
    else if(demo == WakeOnLan)
    {
      // handle ping and wait for a tcp packet
      plen = ES_enc28j60PacketReceive(BUFFER_SIZE, buf);
      dat_p=ES_packetloop_icmp_tcp(buf,plen);
      if( lastUpdate + 10000 < millis() ) {
        ES_send_wol( buf, wolmac );
        lastUpdate = millis();
      }
    }
    else if(demo == Client_viaDNS)
    {
      plen = ES_enc28j60PacketReceive(BUFFER_SIZE, buf);
      dat_p = ES_packetloop_icmp_tcp(buf,plen);
      if(dat_p==0)
      {
        // we are idle here
        if (ES_client_waiting_gw() ){ //0 means have gateway
          asm("NOP");
        }
        else if (Dns_States==DNS_IDLE){
          Dns_States=DNS_REQUEST;
          lastUpdate = millis();
          ES_dnslkup_request(buf, (uint8_t*)WEBSERVER_VHOST );
        }
        else if (Dns_States!=DNS_RECEIVED){
          // retry every 5 seconds if dns-lookup failed:
          if (millis() > (lastUpdate + 5000L) ){
            Dns_States=DNS_IDLE;
            lastUpdate = millis();
          }
        }
        else if (start_web_client==1){
          start_web_client=2;
          strcpy(statusstr,"[ {\"name\":\"TestSalman\"");
          strncat(statusstr,",\"age\":200}]",14);
          ES_client_http_post(API_URL,WEBSERVER_VHOST,BLOGGACCOUNT,NULL,statusstr,&browserresult_callback);
          //ES_client_browse_url(API_URL,NULL,WEBSERVER_VHOST,&browserresult_callback2);
        }
      } 
      else {
        if (Dns_States==DNS_REQUEST && ES_udp_client_check_for_dns_answer( buf, plen ) ){
          uint8_t *ipTemp;
          ipTemp = ES_dnslkup_getip();
          for(int i=0;i<4;i++)
          {
            serverIp[i] = ipTemp[i];
          }
          Dns_States=DNS_RECEIVED;
          ES_client_tcp_set_serverip(serverIp);
        }
      }
      // stuff
    }
    else if(demo == Client_viaIP)
    {
      plen = ES_enc28j60PacketReceive(BUFFER_SIZE, buf);
      dat_p = ES_packetloop_icmp_tcp(buf,plen);
      if(dat_p==0)
      {
        if (start_web_client==1)
        {
          start_web_client=2;
          strcpy(statusstr,"[ {\"name\":\"Client via IP testing\"");
          strncat(statusstr,",\"age\":200}]",14);
          ES_client_http_post(API_URL,WEBSERVER_VHOST,BLOGGACCOUNT,NULL,statusstr,&browserresult_callback);
        }
      }
    }
    else if(demo==Ping)
    {
      plen = ES_enc28j60PacketReceive(BUFFER_SIZE, buf);
      dat_p = ES_packetloop_icmp_tcp(buf,plen);
      if(plen > 0 && ES_packetloop_icmp_checkreply(buf,destip))
      {
        //Internet check
        asm("NOP");
      }
      if(millis() > lastPingtime+5000)
      {
        lastPingtime=millis();
        ES_client_icmp_request(buf,destip);
      }
    }
  }// end of while
}// end of main
