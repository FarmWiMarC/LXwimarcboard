//0106 timedelay depend on loracount

#include <SPI.h>
#include <LoRa.h>
#include <SoftwareSerial.h> //Include the library
SoftwareSerial nextion(7,8); // RX, TX //create a Nextion object named myNextion using the nextion serial port @ 9600bps

int counter = 0;
int loopcount=0;
int packetSize=0;
char rfID='A';
int timedelay=180;
int BitcheckA=0xFF;
int BitcheckB=0xFF;
char my_packet[40];
int e;
int loracount=0;
byte data[12];
byte addr[8];
char netpiepublish[50];
char timestate[30];
int minState=0;
boolean flagSend = true;
// define spi bus pins
//#define SLAVESELECT 10
//#define SPICLOCK 13
//#define DATAOUT 11  //MOSI
//#define DATAIN 12  //MISO
//#define UBLB(a,b)  ( ( (a) << 8) | (b) )
//#define UBLB19(a,b) ( ( (a) << 16 ) | (b) )
int i,j;
int Ptcount=0;
#define rxPin 12
#define txPin 11
#define LORA_RESET 9
#define SENSOR_supplypin 6
char InputRS232[40];
int ValueMT1,ValueMT2,ValueMT3,ValueMT4;
long Tempvoltage,Luxvoltage,Levelvoltage1,Levelvoltage2,winddirectionrika,Levelvoltage3;
long tempT,Lux,Level1,Level2,Level3,pressure;
int v1,v2,v3,v4,v5,v6,v7,v8,d1,d2,WindD;
char ID;
char myVariable;
int bitIwantToCheck;
boolean isBitSet = (myVariable & (1 << bitIwantToCheck)) ? true : false;
//Addresses

//mois
float M;


int angle;
float OldTime;
int voltref=503;
int voltoffset=0;
float timecount =0;
float reftime =0;
int WindReading,intwind;
float WindChill;
char InputRPI[30];
char ValueSensor[50];

int indexchar;
//Anometer
volatile unsigned int windRotation = 0;
//Used for timing
float windTimer = 0;
float windDtime = 0;

float Calculation_temp;
int Calculation_temp1;

//Humidity sensor HIH5030
//Tempe sensor MCP9701 
//Humidity to A0
// Temperature to A1
unsigned long lastvalue;
unsigned long Supvoltage,RHvoltage,Windvoltage;
unsigned long RH = 0;
int sensorPin = 1; 
//int loopcount;
int inputcount=0;
//Rain gauge
unsigned long RainMeasurement = 0;
unsigned long LastRainReset = 0;
volatile byte Hit = 1;

byte CurrentDisplayPage = 0;
int relayPin = 4; //output pin 13

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
boolean FlagShutdown=false;
boolean FlagStart=false;
int RHint;
long temp;

/////////////////////////////////////////////////////////////
/////////////Serial
////////////////////////////////////////////////////////
void serialEvent() {
int i,j;     
      for (i=1;i<500;i++){ 
  char inChar = (char)Serial.read(); 
   
     if ((inChar == 'P')) 
     { InputRPI[0]=inChar;
    for (j=1;j<30;j++){  InputRPI[j] = Serial.read(); }  

  if(( ((InputRPI[0] == 'P')&&(InputRPI[1] == 't'))&&(InputRPI[4] == '-')&&(InputRPI[7] == '-') )||((InputRPI[0] == 'P')&&((InputRPI[1] == 'T')||(InputRPI[1] == 'S'))))
  {
     stringComplete=true;
     
  
  i=1000;
  
   
// for (j=0;j<30;j++){  InputRPI[j]= '0';  }


      } // if 
     }  //if 
     }//for 
  

while (Serial.available()) 
     Serial.read();


  
}

//////////////////////////////////////////////////////////////////

boolean ConvertData_CheckModeID(int measPoint)
{char i,data[3],checkchar;
 boolean FlagSearch;
 int tmp;
 char  Data1,Data2,Data3,Data4,Data5;
 int tmp1,tmp2;

 
 FlagSearch =1;i=0;//k=0;

//char x=InputRS232[1]+InputRS232[4]+InputRS232[7]+InputRS232[10]+InputRS232[13]+InputRS232[16]+InputRS232[19]+InputRS232[22]+InputRS232[25]+InputRS232[28]+InputRS232[31];
char x=InputRS232[1]+InputRS232[4]+InputRS232[7]+InputRS232[10]+InputRS232[13]+InputRS232[16]+InputRS232[19]+InputRS232[22]+InputRS232[25];

   if (InputRS232[32] != x)    {
//    char x =InputRS232[5]+InputRS232[8]+InputRS232[1]+InputRS232[2];
    Serial.print("Input ="); Serial.println(InputRS232[26]);
    Serial.print("check ="); Serial.println(x);
    delay(1000);
    
  //  return false;  
  } 


 /////////////////v1  
  i=InputRS232[1];
  Data1=InputRS232[2];
  Data2=InputRS232[3];
  Data3=InputRS232[4];

  checkchar=i+Data1+Data2;
 if (Data3 == checkchar )
    {  v1 =0;
  tmp =Data1;
  tmp <<= 8;
  tmp &= 0xff00;//
  v1=Data2;
 
  v1 &= 0x00ff;
  v1 |= tmp;   
 
//v1=InputRS232[3]+InputRS232[4];
  v1-=1000;
tmp1 = v1/10000;                tmp2 = v1%10000; 
       Data1 = tmp1;
       tmp1 = tmp2/1000;                     tmp2 %= 1000;               
       Data2 = tmp1;
       tmp1 = tmp2/100;                     tmp2 %= 100;               
       Data3 = tmp1;
       tmp1 = tmp2/10;                     tmp2 %= 10;               
       Data4 = tmp1;
              
       Data5 = tmp2;
       
       Data1 += 0x30;  Data2 += 0x30;  Data3 += 0x30; Data4 += 0x30;  Data5 += 0x30;    
//ValueSensor[2] = 0x41; // "A"
netpiepublish[0] = Data1;
netpiepublish[1] = Data2;
netpiepublish[2] = Data3;
netpiepublish[3] = Data4;
netpiepublish[4] = Data5;
netpiepublish[5] = ';';

}

 // if (data[2] == (InputRS232[3]&&InputRS232[4]) )
 else { return false;}



 



/////////////////v2  
   Data1=InputRS232[5];
   Data2=InputRS232[6];
   Data3=InputRS232[7];

  checkchar=i+Data1+Data2;
 if (Data3 == checkchar )
 {
  v2 =0;
  tmp = Data1;
  tmp <<= 8;
  tmp &= 0xff00;//
  v2=Data2;
 
  v2 &= 0x00ff;
  v2 |= tmp;   
 
 v2-=1000;

tmp1 = v2/10000;                tmp2 = v2%10000; 
       Data1 = tmp1;
       tmp1 = tmp2/1000;                     tmp2 %= 1000;               
       Data2 = tmp1;
       tmp1 = tmp2/100;                     tmp2 %= 100;               
       Data3 = tmp1;
       tmp1 = tmp2/10;                     tmp2 %= 10;               
       Data4 = tmp1;
              
       Data5 = tmp2;
       
       Data1 += 0x30;  Data2 += 0x30;  Data3 += 0x30; Data4 += 0x30;  Data5 += 0x30;    
//ValueSensor[2] = 0x41; // "A"
netpiepublish[6] = Data1;
netpiepublish[7] = Data2;
netpiepublish[8] = Data3;
netpiepublish[9] = Data4;
netpiepublish[10] = Data5;
netpiepublish[11] = ';';

 }
 // if (data[2] == (InputRS232[3]&&InputRS232[4]) )
 else { return false;}

 
/////////////////v3  
    Data1=InputRS232[8];
    Data2=InputRS232[9];
    Data3=InputRS232[10];
   checkchar=i+Data1+Data2;
 if (Data3 == checkchar )
 {v3 =0;
  tmp =  Data1;
  tmp <<= 8;
  tmp &= 0xff00;//
  v3= Data2;
 
  v3 &= 0x00ff;
  v3 |= tmp;   
v3-=1000;
tmp1 = v3/10000;                tmp2 = v3%10000; 
       Data1 = tmp1;
       tmp1 = tmp2/1000;                     tmp2 %= 1000;               
       Data2 = tmp1;
       tmp1 = tmp2/100;                     tmp2 %= 100;               
       Data3 = tmp1;
       tmp1 = tmp2/10;                     tmp2 %= 10;               
       Data4 = tmp1;
              
       Data5 = tmp2;
       
       Data1 += 0x30;  Data2 += 0x30;  Data3 += 0x30; Data4 += 0x30;  Data5 += 0x30;    
//ValueSensor[2] = 0x41; // "A"
netpiepublish[12] = Data1;
netpiepublish[13] = Data2;
netpiepublish[14] = Data3;
netpiepublish[15] = Data4;
netpiepublish[16] = Data5;
netpiepublish[17] = ';';

 }
 // if (data[2] == (InputRS232[3]&&InputRS232[4]) )
 else { return false;}

/////////////////v4  
    Data1=InputRS232[11];
    Data2=InputRS232[12];
    Data3=InputRS232[13];
   checkchar=i+Data1+Data2;
 if (Data3 == checkchar )
 {v4 =0;
  tmp =  Data1;
  tmp <<= 8;
  tmp &= 0xff00;//
  v4= Data2;
 
  v4 &= 0x00ff;
  v4 |= tmp;   

 v4-=1000;

tmp1 = v4/10000;                tmp2 = v4%10000; 
       Data1 = tmp1;
       tmp1 = tmp2/1000;                     tmp2 %= 1000;               
       Data2 = tmp1;
       tmp1 = tmp2/100;                     tmp2 %= 100;               
       Data3 = tmp1;
       tmp1 = tmp2/10;                     tmp2 %= 10;               
       Data4 = tmp1;
              
       Data5 = tmp2;
       
       Data1 += 0x30;  Data2 += 0x30;  Data3 += 0x30; Data4 += 0x30;  Data5 += 0x30;    
//ValueSensor[2] = 0x41; // "A"
netpiepublish[18] = Data1;
netpiepublish[19] = Data2;
netpiepublish[20] = Data3;
netpiepublish[21] = Data4;
netpiepublish[22] = Data5;
netpiepublish[23] = ';';

 }
 // if (data[2] == (InputRS232[3]&&InputRS232[4]) )
 else { return false;}


/////////////////v5  
    Data1=InputRS232[14];
    Data2=InputRS232[15];
    Data3=InputRS232[16];
    checkchar=i+Data1+Data2;
 if (Data3 == checkchar )
 {v5 =0;
  tmp =  Data1;
  tmp <<= 8;
  tmp &= 0xff00;//
  v5= Data2;
 
  v5 &= 0x00ff;
  v5 |= tmp;   
 v5-=1000;
tmp1 = v5/10000;                tmp2 = v5%10000; 
       Data1 = tmp1;
       tmp1 = tmp2/1000;                     tmp2 %= 1000;               
       Data2 = tmp1;
       tmp1 = tmp2/100;                     tmp2 %= 100;               
       Data3 = tmp1;
       tmp1 = tmp2/10;                     tmp2 %= 10;               
       Data4 = tmp1;
              
       Data5 = tmp2;
       
       Data1 += 0x30;  Data2 += 0x30;  Data3 += 0x30; Data4 += 0x30;  Data5 += 0x30;    
//ValueSensor[2] = 0x41; // "A"
netpiepublish[24] = Data1;
netpiepublish[25] = Data2;
netpiepublish[26] = Data3;
netpiepublish[27] = Data4;
netpiepublish[28] = Data5;
netpiepublish[29] = ';';

 }
 // if (data[2] == (InputRS232[3]&&InputRS232[4]) )
 else { return false;}

/*
 v5= InputRS232[16];
tmp1 = v5/10000;                tmp2 = v5%10000; 
       Data1 = tmp1;
       tmp1 = tmp2/1000;                     tmp2 %= 1000;               
       Data2 = tmp1;
       tmp1 = tmp2/100;                     tmp2 %= 100;               
       Data3 = tmp1;
       tmp1 = tmp2/10;                     tmp2 %= 10;               
       Data4 = tmp1;
              
       Data5 = tmp2;
       
       Data1 += 0x30;  Data2 += 0x30;  Data3 += 0x30; Data4 += 0x30;  Data5 += 0x30;    
//ValueSensor[2] = 0x41; // "A"
netpiepublish[24] = Data1;
netpiepublish[25] = Data2;
netpiepublish[26] = Data3;
netpiepublish[27] = Data4;
netpiepublish[28] = Data5;
netpiepublish[29] = ';';

 Data1=InputRS232[14];
    Data2=InputRS232[15];
 Data3=i+Data1+Data2;

 v6=Data3;
tmp1 = v6/10000;                tmp2 = v6%10000; 
       Data1 = tmp1;
       tmp1 = tmp2/1000;                     tmp2 %= 1000;               
       Data2 = tmp1;
       tmp1 = tmp2/100;                     tmp2 %= 100;               
       Data3 = tmp1;
       tmp1 = tmp2/10;                     tmp2 %= 10;               
       Data4 = tmp1;
              
       Data5 = tmp2;
       
       Data1 += 0x30;  Data2 += 0x30;  Data3 += 0x30; Data4 += 0x30;  Data5 += 0x30;    
//ValueSensor[2] = 0x41; // "A"
netpiepublish[30] = Data1;
netpiepublish[31] = Data2;
netpiepublish[32] = Data3;
netpiepublish[33] = Data4;
netpiepublish[34] = Data5;
netpiepublish[35] = ';';
 


*/

/////////////////v6  
    Data1=InputRS232[17];
    Data2=InputRS232[18];
    Data3=InputRS232[19];
    checkchar=i+Data1+Data2;
 if (Data3 == checkchar )
 {v6 =0;
  tmp =  Data1;
  tmp <<= 8;
  tmp &= 0xff00;//
  v6= Data2;
 
  v6 &= 0x00ff;
  v6 |= tmp;   
 v6-=1000;
tmp1 = v6/10000;                tmp2 = v6%10000; 
       Data1 = tmp1;
       tmp1 = tmp2/1000;                     tmp2 %= 1000;               
       Data2 = tmp1;
       tmp1 = tmp2/100;                     tmp2 %= 100;               
       Data3 = tmp1;
       tmp1 = tmp2/10;                     tmp2 %= 10;               
       Data4 = tmp1;
              
       Data5 = tmp2;
       
       Data1 += 0x30;  Data2 += 0x30;  Data3 += 0x30; Data4 += 0x30;  Data5 += 0x30;    
//ValueSensor[2] = 0x41; // "A"
netpiepublish[30] = Data1;
netpiepublish[31] = Data2;
netpiepublish[32] = Data3;
netpiepublish[33] = Data4;
netpiepublish[34] = Data5;
netpiepublish[35] = ';';

 }
 // if (data[2] == (InputRS232[3]&&InputRS232[4]) )
 else { return false;}



/////////////////v7  
    Data1=InputRS232[20];
    Data2=InputRS232[21];
    Data3=InputRS232[22];
 checkchar=i+Data1+Data2;
 if (Data3 == checkchar )
 {v7 =0;
  tmp =  Data1;
  tmp <<= 8;
  tmp &= 0xff00;//
  v7= Data2;
 
  v7 &= 0x00ff;
  v7 |= tmp;   
 v7-=1000;
tmp1 = v7/10000;                tmp2 = v7%10000; 
       Data1 = tmp1;
       tmp1 = tmp2/1000;                     tmp2 %= 1000;               
       Data2 = tmp1;
       tmp1 = tmp2/100;                     tmp2 %= 100;               
       Data3 = tmp1;
       tmp1 = tmp2/10;                     tmp2 %= 10;               
       Data4 = tmp1;
              
       Data5 = tmp2;
       
       Data1 += 0x30;  Data2 += 0x30;  Data3 += 0x30; Data4 += 0x30;  Data5 += 0x30;    
//ValueSensor[2] = 0x41; // "A"
netpiepublish[36] = Data1;
netpiepublish[37] = Data2;
netpiepublish[38] = Data3;
netpiepublish[39] = Data4;
netpiepublish[40] = Data5;
netpiepublish[41] = ';';

 }
 // if (data[2] == (InputRS232[3]&&InputRS232[4]) )
 else { return false;}

/////////////////v8  
    Data1=InputRS232[23];
    Data2=InputRS232[24];
    Data3=InputRS232[25];
checkchar=i+Data1+Data2;
 if (Data3 == checkchar )
 {v8 =0;
  tmp =  Data1;
  tmp <<= 8;
  tmp &= 0xff00;//
  v8= Data2;
 
  v8 &= 0x00ff;
  v8 |= tmp;   
 v8-=1000;
tmp1 = v8/10000;                tmp2 = v8%10000; 
       Data1 = tmp1;
       tmp1 = tmp2/1000;                     tmp2 %= 1000;               
       Data2 = tmp1;
       tmp1 = tmp2/100;                     tmp2 %= 100;               
       Data3 = tmp1;
       tmp1 = tmp2/10;                     tmp2 %= 10;               
       Data4 = tmp1;
              
       Data5 = tmp2;
       
       Data1 += 0x30;  Data2 += 0x30;  Data3 += 0x30; Data4 += 0x30;  Data5 += 0x30;    
//ValueSensor[2] = 0x41; // "A"
netpiepublish[42] = Data1;
netpiepublish[43] = Data2;
netpiepublish[44] = Data3;
netpiepublish[45] = Data4;
netpiepublish[46] = Data5;
netpiepublish[47] = ';';

 }
/*
/////////////////d1  
    Data1=InputRS232[26];
    Data2=InputRS232[27];
    Data3=InputRS232[28];
checkchar=i+Data1+Data2;
 if (Data3 == checkchar )
 {d1 =0;
  tmp =  Data1;
  tmp <<= 8;
  tmp &= 0xff00;//
  d1= Data2;
 
  d1 &= 0x00ff;
  d1 |= tmp;   
 d1-=1000;
tmp1 = d1/10000;                tmp2 = d1%10000; 
       Data1 = tmp1;
       tmp1 = tmp2/1000;                     tmp2 %= 1000;               
       Data2 = tmp1;
       tmp1 = tmp2/100;                     tmp2 %= 100;               
       Data3 = tmp1;
       tmp1 = tmp2/10;                     tmp2 %= 10;               
       Data4 = tmp1;
              
       Data5 = tmp2;
       
       Data1 += 0x30;  Data2 += 0x30;  Data3 += 0x30; Data4 += 0x30;  Data5 += 0x30;    
//ValueSensor[2] = 0x41; // "A"
netpiepublish[48] = Data1;
netpiepublish[49] = Data2;
netpiepublish[50] = Data3;
netpiepublish[51] = Data4;
netpiepublish[52] = Data5;
netpiepublish[53] = ';';
 }
/////////////////d2  
    Data1=InputRS232[29];
    Data2=InputRS232[30];
    Data3=InputRS232[31];
checkchar=i+Data1+Data2;
 if (Data3 == checkchar )
 {d2 =0;
  tmp =  Data1;
  tmp <<= 8;
  tmp &= 0xff00;//
  d2= Data2;
 
  d2 &= 0x00ff;
  d2 |= tmp;   
  d2-=1000;
tmp1 = d2/10000;                tmp2 = d2%10000; 
       Data1 = tmp1;
       tmp1 = tmp2/1000;                     tmp2 %= 1000;               
       Data2 = tmp1;
       tmp1 = tmp2/100;                     tmp2 %= 100;               
       Data3 = tmp1;
       tmp1 = tmp2/10;                     tmp2 %= 10;               
       Data4 = tmp1;
              
       Data5 = tmp2;
       
       Data1 += 0x30;  Data2 += 0x30;  Data3 += 0x30; Data4 += 0x30;  Data5 += 0x30;    
//ValueSensor[2] = 0x41; // "A"
netpiepublish[54] = Data1;
netpiepublish[55] = Data2;
netpiepublish[56] = Data3;
netpiepublish[57] = Data4;
netpiepublish[58] = Data5;
netpiepublish[59] = ';';





 
 
 
 
 }
 
 // if (data[2] == (InputRS232[3]&&InputRS232[4]) )
 else { return false;}
*/





 
 
 
 
 
}


void Wind_Init(void)
{
  pinMode(3, INPUT);
  attachInterrupt(1, windSpeed, RISING);
  windTimer=millis();//start timing  
  
}

int Wind_GetSpeed(void)
{
  /*
  The cup-type anemometer measures wind speed by closing a contact as 
  a magnet moves past a switch.  A wind speed of 1.492 MPH (2.4 km/h) 
  causes the switch to close once per second.
  */ 
  
  //Check using Interrupt
  float windSpeed = 0;
  
  windDtime =  millis()-windTimer;
  windTimer = millis();
  windDtime = windDtime/1000;
 //  Serial.print("windDtime =              ");
  //    Serial.print(windDtime);    
  //    Serial.println("  "); 
  
  windSpeed = windRotation*100/windDtime;//rotation per second
 //  Serial.print("windRotation =           ");
  //    Serial.print(windRotation);    
 //     Serial.println("  ");   
  windRotation = 0;  
  windSpeed = windSpeed*2/3;//1 rotation per second equals 2.4 km/h = 2/3 m/s
  return int(windSpeed); 
}

void windSpeed()
{
  windRotation++;
}

void WindDirection(byte y)
{ int adc;
    // read the analog input into a variable:
   
 Windvoltage = analogRead(2);
 //adc = get_wind_direction(Windvoltage);
 
 
 
  Windvoltage=  Windvoltage*voltref;
  Windvoltage=Windvoltage/102.3-voltoffset;
   WindD=360*Windvoltage/5000;
  
    Serial.print("Windvoltage =");
    Serial.print(Windvoltage);//print the result
    Serial.println(" mV");//print the result 
 
 
   //ProcessConvertValueToRPI('W','A',adc,Windvoltage);
   //Serial.println(ValueSensor);
   //  Serial.println();
   
 
  
}  




void Rain_Init(void)
{
  attachInterrupt(0, Rain_Measure, RISING);
  //LastRainReset = millis();
 // LCD_CenterText("* Regn", 54);
}    

void Rain_Measure(void)
{
  RainMeasurement = RainMeasurement + 20; 
 
}

////////////////////////////////////////////////
/////////////Convert data send with LORA
///////////////////


void ProcessSendLORA(char idex)
{ 
int i;
int Data1,Data2,Data3;
   int tmp;
char sum1,sum2,sum3,sum4,sum5,sum6,sum7,sum8,sum9,sum10;
ValueSensor[0] = 'P';
ValueSensor[1] = idex;
//////////////////v1 Send
 

 
 tmp =v1+1000; 
  tmp = tmp & 0xff00;
  tmp >>= 8; 
  Data1 = tmp;

  
 tmp =v1+1000; 
 
  tmp = tmp & 0x00ff;
  Data2 = tmp;



  
sum1=idex+Data1+Data2;
Data3=Data1+Data2;


//Data1='A';

ValueSensor[2] = Data1;
ValueSensor[3] = Data2;
ValueSensor[4] = sum1;

//////////////////v2 Send
  tmp =v2+1000; 
  tmp = tmp & 0xff00;
  tmp >>= 8; 
  Data1 = tmp;

  
 tmp =v2+1000; 
 
  tmp = tmp & 0x00ff;
  Data2 = tmp;
  
sum2=idex+Data1+Data2;

Data3=Data1+Data2;
ValueSensor[5] = Data1;
ValueSensor[6] = Data2;
ValueSensor[7] = sum2;
//////////////////v3 Send
  tmp =v3+1000; 
  tmp = tmp & 0xff00;
  tmp >>= 8; 
  Data1 = tmp;

  
 tmp =v3+1000; 
 
  tmp = tmp & 0x00ff;
  Data2 = tmp;
sum3=idex+Data1+Data2;


Data3=Data1+Data2;

ValueSensor[8] = Data1;
ValueSensor[9] = Data2;
ValueSensor[10] = sum3;
//////////////////v4 Send
 tmp =v4+1000; 
  tmp = tmp & 0xff00;
  tmp >>= 8; 
  Data1 = tmp;

  
 tmp =v4+1000; 
 
  tmp = tmp & 0x00ff;
  Data2 = tmp;
sum4=idex+Data1+Data2;
Data3=Data1+Data2;
ValueSensor[11] = Data1;
ValueSensor[12] = Data2;
ValueSensor[13] =sum4;
//////////////////v5 Send
  tmp =v5+1000; 
  tmp = tmp & 0xff00;
  tmp >>= 8; 
  Data1 = tmp;

  
 tmp =v5+1000; 
 
  tmp = tmp & 0x00ff;
  Data2 = tmp;
sum5=idex+Data1+Data2;
Data3=Data1+Data2;
ValueSensor[14] = Data1;
ValueSensor[15] = Data2;
ValueSensor[16] = sum5;
//////////////////v6 Send
 tmp =v6+1000; 
  tmp = tmp & 0xff00;
  tmp >>= 8; 
  Data1 = tmp;

  
 tmp =v6+1000; 
 
  tmp = tmp & 0x00ff;
  Data2 = tmp;
sum6=idex+Data1+Data2;
Data3=Data1+Data2;
ValueSensor[17] = Data1;
ValueSensor[18] = Data2;
ValueSensor[19] =sum6;
//////////////////v7 Send
 tmp =v7+1000; 
  tmp = tmp & 0xff00;
  tmp >>= 8; 
  Data1 = tmp;

  
 tmp =v7+1000; 
 
  tmp = tmp & 0x00ff;
  Data2 = tmp;
sum7=idex+Data1+Data2;
Data3=Data1+Data2;
ValueSensor[20] = Data1;
ValueSensor[21] = Data2;
ValueSensor[22] = sum7;


//////////////////v8 Send
 tmp =v8+1000; 
  tmp = tmp & 0xff00;
  tmp >>= 8; 
  Data1 = tmp;

  
 tmp =v8+1000; 
 
  tmp = tmp & 0x00ff;
  Data2 = tmp;
sum8=idex+Data1+Data2;

Data3=Data1+Data2;
ValueSensor[23] = Data1;
ValueSensor[24] = Data2;
ValueSensor[25] = sum8;




//////////////////d1 Send
 tmp =d1+1000; 
  tmp = tmp & 0xff00;
  tmp >>= 8; 
  Data1 = tmp;

  
 tmp =v8+1000; 
 
  tmp = tmp & 0x00ff;
  Data2 = tmp;
sum9=idex+Data1+Data2;

Data3=Data1+Data2;
ValueSensor[27] = Data1;
ValueSensor[28] = Data2;
ValueSensor[29] = sum9;

//////////////////d2 Send
 tmp =d2+1000; 
  tmp = tmp & 0xff00;
  tmp >>= 8; 
  Data1 = tmp;

  
 tmp =d2+1000; 
 
  tmp = tmp & 0x00ff;
  Data2 = tmp;
sum10=idex+Data1+Data2;

Data3=Data1+Data2;
ValueSensor[30] = Data1;
ValueSensor[31] = Data2;
ValueSensor[32] = sum10;


ValueSensor[26]=ValueSensor[1]+ValueSensor[4]+ValueSensor[7]+ValueSensor[10]+ValueSensor[13]+ValueSensor[16]+ValueSensor[19]+ValueSensor[22]+ValueSensor[25];
//ValueSensor[32]=idex+sum1+sum2+sum3+sum4+sum5+sum6+sum7+sum8+sum9+sum10;//
 
//ValueSensor[27]='\n';

//for (i=1;i<30;i++) { Serial.print(ValueSensor[i],HEX); Serial.print(','); }   
for (i=0;i<5;i++){
  for (int j=0;j<30;j++)
  Serial.print(ValueSensor[j]); 
  Serial.println();
  delay(500);
}



}

boolean checkID()
{    Serial.println("..............................................................check ID");
     if (InputRS232[0] == 'I')
     {
      if (InputRS232[3]==(InputRS232[1]+InputRS232[2]))
        { Serial.print("1<<3:");Serial.println((1<<3),BIN);
          Serial.print("InputRS232[1]:");Serial.println(InputRS232[1],BIN);
          Serial.print("InputRS232[2]:");Serial.println(InputRS232[2],BIN);
          if (isBitSet) Serial.println ("isBitSet: TRUE"); else Serial.println ("isBitSet: false");
          if( isBitSet = (InputRS232[1] & (1 << 3))) {ID='A';Serial.print("ID:A send"); if (ID==rfID) {return true;exit;}}
          //else {return false;exit;}  
          bitIwantToCheck=4;
          if( isBitSet = (InputRS232[1] & (1 << 2))) {ID='B';Serial.print("ID:B send"); if (ID==rfID) {return true;exit;}}
          //else {return false;exit;}
          bitIwantToCheck=4;
          if( isBitSet = (InputRS232[1] & (1 << 1))) {ID='C';Serial.print("ID:C send"); if (ID==rfID) {return true;exit;}}
          //else {return false;exit;}
          bitIwantToCheck=4;
          if( isBitSet = (InputRS232[1] & (1 << 0))) {ID='D';Serial.print("ID:D send"); if (ID==rfID) {return true;exit;}}
          //else {return false;exit;}
          if( isBitSet = (InputRS232[2] & (1 << 3))) {ID='E';Serial.print("ID:E send"); if (ID==rfID) {return true;exit;}}
          //else {return false;exit;}  
          bitIwantToCheck=4;
          if( isBitSet = (InputRS232[2] & (1 << 2))) {ID='F';Serial.print("ID:F send"); if (ID==rfID) {return true;exit;}}
          //else {return false;exit;}
          bitIwantToCheck=4;
          if( isBitSet = (InputRS232[2] & (1 << 1))) {ID='G';Serial.print("ID:G send"); if (ID==rfID) {return true;exit;}}
          //else {return false;exit;}
          bitIwantToCheck=4;
          if( isBitSet = (InputRS232[2] & (1 << 0))) {ID='H';Serial.print("ID:H send"); if (ID==rfID) {return true;exit;}}
        
        }
        
       Serial.print(rfID); Serial.println("...already sent" );

       
       return false;  
     }  
   else return false;
}





void setup() {
  delay (3000);
  Serial.begin(9600);
   pinMode(relayPin, OUTPUT); // output
  pinMode(5, OUTPUT);
   pinMode(LORA_RESET, OUTPUT);digitalWrite(LORA_RESET, HIGH);//LORA RESET
   pinMode(SENSOR_supplypin, OUTPUT);digitalWrite(SENSOR_supplypin, LOW);//sensor supply
  while (!Serial);
 
  Serial.println("LoRa Reciever.................");

  if (!LoRa.begin(925E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  
  
  }

  LoRa.setTxPower(20);
/////////////////////////////////

 Wind_Init();
  Rain_Init();
  delay(1000);
   
 //  digitalWrite(relayPin, HIGH); // output pin 13
  loopcount=0;
   WindReading=0;
   RainMeasurement = 0;
   RHint=0;
   temp=0;
 //WindReading = Wind_GetSpeed();
    
   RainMeasurement = 0;
 //  RHint=0;
 RHvoltage = analogRead(0);
 RHvoltage = RHvoltage*voltref;
 RHvoltage = RHvoltage/102.3-voltoffset;

Tempvoltage = analogRead(1); 
 Tempvoltage = Tempvoltage*voltref;
Tempvoltage = Tempvoltage/102.3-voltoffset;

Windvoltage = analogRead(2); 
 Windvoltage = Windvoltage*voltref;
Windvoltage = Windvoltage/102.3-voltoffset;
 

 
Luxvoltage = analogRead(3); 
 Luxvoltage = Luxvoltage*voltref;
Luxvoltage = Luxvoltage/102.3-voltoffset;

Levelvoltage1 = analogRead(4); 
 Levelvoltage1 = Levelvoltage1*voltref;
Levelvoltage1 = Levelvoltage1/102.3-voltoffset;

Levelvoltage2 = analogRead(5); 
 Levelvoltage2 = Levelvoltage2*voltref;
Levelvoltage2 = Levelvoltage2/102.3-voltoffset;

Levelvoltage3 = analogRead(6); 
 Levelvoltage3 = Levelvoltage3*voltref;
Levelvoltage3 = Levelvoltage3/102.3-voltoffset;


  Supvoltage = analogRead(7);
 Supvoltage = Supvoltage*voltref;
Supvoltage = Supvoltage/102.3-voltoffset;
Supvoltage = Supvoltage*11+700;

 reftime =millis();
/////////////////





  /////////////////////////////////////////////////////////
  
  





  
}

void loop() {

if (stringComplete)
{
  stringComplete=false;
  Serial.println(InputRPI);
  for (j=0;j<30;j++){  InputRPI[j]= '0';
  }
}
//////////////read input///////////////////



timecount = (millis()-reftime)/1000; 
Serial.print("timecount = "); Serial.println(timecount);
if (loracount > 5) {timedelay=300-loracount*10;if (timedelay<0) timedelay=300;}
if (timecount > timedelay)
 {  for (int j=0;j<5;j++)
 {
 Serial.print("...........send rfID:");
 Serial.println(rfID);
  LoRa.beginPacket();
  LoRa.print(ValueSensor);
 // LoRa.print(counter);
  LoRa.endPacket();
  int randomdelay = random(500, 2000);
 Serial.println(randomdelay);
  delay(randomdelay);
 
 }
 reftime= millis();
 }
  /////////////////////////////////////////////////////////
 
 // Wind direction measurement 
 //WindDirection(36); 

 
 /////////////////////////////////////////////////////////
 
 
 
 ////////////////////////////////////////////////////////////
 // Wind direction measurement 
 
 ///////////////////////////////////////////////////////////
   
  
  lastvalue= WindReading;
  Serial.print("last Windspeed =");
  Serial.println(WindReading);  

  delay(500);
  WindReading = Wind_GetSpeed();
    
  WindReading = (lastvalue+WindReading)/2;
  
      delay(50);
      Serial.print("Windspeed =  ");
      Serial.print(WindReading);    
      Serial.println(" cm/s");   
  
 //  Serial.println(ValueSensor);    
    Serial.println(); 
     // delay(3000);
 //  Print_WindDirection(36); 
  
  
  delay(50); 
        
///////////////////////////////////////////////////////////////   
         
// Humidity measurement       
  
///////////////////////////////////////////////////////////////  

 lastvalue=RHvoltage;


  RHvoltage = analogRead(0);
 RHvoltage = RHvoltage*voltref;
 RHvoltage = RHvoltage/102.3-voltoffset;
 
 RHvoltage = (RHvoltage+lastvalue)/2;
 
 RH= (1250*RHvoltage)/voltref-1250;
if (RH > 10000)
RH=10000;


 
 

 
 Serial.print("Humid ="); 
 Serial.print(RHvoltage);
 
  
 Serial.println(" %"); 


// Serial.println(ValueSensor);
Serial.println(); 

delay(50);
 
/////////////////////////////////////////////////////////////////////

// ambient temperature measurement

//////////////////////////////////////////////////////////////////// 

 lastvalue=Tempvoltage;
 Tempvoltage = analogRead(1); 
 Tempvoltage = Tempvoltage*voltref;
Tempvoltage = Tempvoltage/102.3-voltoffset;
 
  
   Tempvoltage = (Tempvoltage+lastvalue)/2;
  tempT = (Tempvoltage*2187.5)/voltref-6687.5;
 
  Serial.print("Temp  ="); 
 
 
 Serial.print(Tempvoltage);
 Serial.println(" C");
 
 

// Serial.println(ValueSensor);

 delay(50);
 Serial.println();


/////////////////////////////////////////////////////////////////////

// Lux measurement

//////////////////////////////////////////////////////////////////// 

 lastvalue=Luxvoltage;
Luxvoltage = analogRead(3); 
 Luxvoltage = Luxvoltage*voltref;
Luxvoltage = Luxvoltage/102.3-voltoffset;
 
 
   Luxvoltage = (Luxvoltage+lastvalue)/2;
     Lux = (Luxvoltage*100) /440;
  Serial.print("Lux  ="); 
 
 
 Serial.print(Luxvoltage);
 Serial.println(" lLux");
 
 

// Serial.println(ValueSensor);

 delay(50);
 Serial.println();

/////////////////////////////////////////////////////////////////////

//
//Lux 100k
//////////////////////////////////////////////////////////////////// 



lastvalue=Levelvoltage1;

 
Levelvoltage1 = analogRead(4); 
 Levelvoltage1 = Levelvoltage1*voltref;
Levelvoltage1 = Levelvoltage1/102.3-voltoffset;
 
 
  //int temp=(int)(temperatureC*10);
 
   Levelvoltage1 = (Levelvoltage1+lastvalue)/2;
  // temp = (Levelvoltage1 - 500) /8.0;
  //  Level1=temp;
  //winddirectionrika = 360*Levelvoltage1/5000;
  //Serial.print("Level1 ="); Serial.print(Level1,1); Serial.println("");
 pressure = 500+(Levelvoltage1-500)*0.25;
  Serial.print(" pressure ="); Serial.print(pressure); Serial.println("");
 
 delay(500);

// Serial.println(ValueSensor);

  delay(50);
 Serial.println();

/////////////////////////////////////////////////////////////////////

//
//Level2
//////////////////////////////////////////////////////////////////// 



 lastvalue=Levelvoltage2;

 
Levelvoltage2 = analogRead(5); 

Serial.print("Level2  ="); 
 
 
 Serial.print(Levelvoltage2);
 Serial.println(" C");
 Levelvoltage2 = Levelvoltage2*voltref;
Levelvoltage2 = Levelvoltage2/102.3-voltoffset;
 
  Serial.print("Level2  ="); 
 
 
 Serial.print(Levelvoltage2);
 Serial.println(" C");
 
  //int temp=(int)(temperatureC*10);
 
   Levelvoltage2 = (Levelvoltage2+lastvalue)/2;
  // temp = (Levelvoltage2 - 500) /8.0;
 
 

 //Serial.println(ValueSensor);

  delay(50);
 Serial.println();

 
 //////////////////////////////////////////////////////////////

 /////////////////////////////////////////////////////////////////////

//
//Level3
//////////////////////////////////////////////////////////////////// 



 lastvalue=Levelvoltage3;

 
Levelvoltage3 = analogRead(6); 
Serial.print("Level3  ="); 
 
 
 Serial.print(Levelvoltage3);
 Serial.println(" C");
 
 Levelvoltage3 = Levelvoltage3*voltref;
Levelvoltage3 = Levelvoltage3/102.3-voltoffset;
 Serial.print("Level3  ="); 
 
 
 Serial.print(Levelvoltage3);
 Serial.println(" C");
  
 
 
  //int temp=(int)(temperatureC*10);
 
   Levelvoltage3 = (Levelvoltage3+lastvalue)/2;
 // temp = (Levelvoltage - 500) /8.0;
 
 

 //Serial.println(ValueSensor);

  delay(50);
// Serial.println();

 
 //////////////////////////////////////////////////////////////


 
 //////////////////////////////////////////////////////////////
 
 // Rain measurement ....................

///////////////////////////////////////////////////////////////// 
      Serial.print("Rain gauge = ");
      Serial.println(RainMeasurement);
  
 //  Serial.println(ValueSensor);
   Serial.println(); 

   //////////////////////////////////////////////////////////////
   
  // Supply measurement....................
 
 //////////////////////////////////////////////////////////////////
 
 
  lastvalue=Supvoltage;

  Supvoltage = analogRead(7);
 Supvoltage = Supvoltage*voltref;
Supvoltage = Supvoltage/102.3-voltoffset;
Supvoltage = Supvoltage*11+700;
 if (Supvoltage < 9000)
 {pinMode(SENSOR_supplypin, OUTPUT);digitalWrite(SENSOR_supplypin, HIGH);
v1=0;
v2=0;
v3=0;
v4=0;
v5=0;
v6=0;
v8=0;
 
 }
else
{pinMode(SENSOR_supplypin, OUTPUT);digitalWrite(SENSOR_supplypin, LOW);


} 
   Supvoltage = (Supvoltage+lastvalue)/2;
   
  Serial.print("Supply  ="); 
 
 
 Serial.print(Supvoltage,1);
 Serial.println("V");

 lastvalue=Windvoltage;
//////////////////////////////////////////////////
  Windvoltage = analogRead(2);
 Windvoltage = Windvoltage*voltref;
Windvoltage = Windvoltage/102.3-voltoffset;
//Supvoltage = Supvoltage*11+700;
 

 
   Windvoltage = (Windvoltage+lastvalue)/2;
   
  Serial.print("Windvoltage/a2  ="); 
 
 
 Serial.print(Windvoltage,1);
 Serial.println("V");
 


 
  delay(100);
// Serial.println();
   //   delay(3000);
v1= RHvoltage;    
v2=Tempvoltage;
v3=Luxvoltage ; 
v4=Levelvoltage1;
v5=Levelvoltage2;
v6=Levelvoltage3;
v8=Windvoltage; 
v7=Supvoltage;


 d1=RainMeasurement; d2=windRotation;
 
// v1=100;v2=200;v1=100;v2=200;v1=100;v4=200;v5=100;v6=200;v7=100;
// d1=11111;d2=2222;
 ProcessSendLORA(rfID);
  delay(1500);

Serial.print("Header:");Serial.println("V");
Serial.print("v1:");Serial.println(v1);
Serial.print("v2:");Serial.println(v2);
Serial.print("v3:");Serial.println(v3);
Serial.print("v4:");Serial.println(v4);
Serial.print("v5:");Serial.println(v5);
Serial.print("v6:");Serial.println(v6);
Serial.print("v7:");Serial.println(v7);
Serial.print("v8:");Serial.println(v8);

//int packetSize = LoRa.parsePacket();
  Serial.print("waiting for Input packet loracount ");
  
    Serial.println(loracount);
    int loopnumber = 1000;
    if (loracount > 3) loopnumber = loracount*500;
    if (loopnumber > 5000) loopnumber = 5000;
  while ((packetSize == 0)&&(loopcount < loopnumber)) 
  {
 
  loopcount++;
    if(loopcount%100 == 0) Serial.print(".");
  
     packetSize = LoRa.parsePacket();
    // received a packet
  if ((packetSize > 0)&&(packetSize < 40))
  {
    Serial.print("Received packet '");

    // read packet
    int i=0;
    
    while (LoRa.available()) {
      InputRS232[i]=((char)LoRa.read());
      i++;
    }
   Serial.println(InputRS232);
   
    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
    Serial.print(" loopcount ");
    Serial.println(loopcount);
    Serial.print(" packetSize  ");
    Serial.println(packetSize);
    //loracount =0;
nextion.begin(9600); // set the data rate for the SoftwareSerial port
 
  // We change the image of image box p0
  nextion.print(ValueSensor);

nextion.end();

  if (checkID())
    {
    
    for (int j=0;j<10;j++)
 {
 Serial.print("...........send rfID:");
 Serial.println(rfID);
  LoRa.beginPacket();
  LoRa.print(ValueSensor);
 // LoRa.print(counter);
  LoRa.endPacket();
  int randomdelay = random(500, 2000);
 Serial.println(randomdelay);
  delay(randomdelay);
 }
  loracount=0;   
    }


else{
    if (ConvertData_CheckModeID(0))
{
  
Serial.print(F("netpie: "));
Serial.println(netpiepublish);
 Serial.println(InputRS232);
 if (loracount >1) loracount--;
 }

 // loracount=0;  
  }
  }
  delay(10);
  }//while
  loracount++;
  Serial.print("loracounnt: ");Serial.println(loracount);
loopcount=0;
packetSize=0;
LoRa.sleep();
delay(random(3000, 5000));

if (loracount%6 == 0)
  {
   digitalWrite(LORA_RESET, LOW); // output pin 13
   delay(5000);
   digitalWrite(LORA_RESET, HIGH); // output pin 13
    if (!LoRa.begin(925E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  
  
  }

  LoRa.setTxPower(20);
Serial.println("RESET LORA");
  
  }
if (loracount > 100)
  { digitalWrite(5, LOW); // output pin 13
    delay(5000);
    digitalWrite(5, HIGH);
     
  }
LoRa.sleep();
}



  


 
