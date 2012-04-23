#include "DHT.h"

#define ir_in 2		//Sensor pin 1 wired through a 220 ohm resistor
#define ir_out 8
#define dht_vpin 6
#define dht_dpin 7
#define led_pin 13
#define DEBUG 0

//byte dht_data[5];//Array to hold the bytes sent from sensor.
DHT dht;

int start_bit_S = 2000;	//Start bit threshold (Microseconds)
int bin_1_S = 1000;	//Binary 1 threshold (Microseconds) 1000
int bin_0_S = 400;	//Binary 0 threshold (Microseconds) 400

int start_bit_N = 2400;			 //Start bit threshold (Microseconds)
int bin_1_N = 1200;				   //Binary 1 threshold (Microseconds)
int bin_0_N = 600;				     //Binary 0 threshold (Microseconds)
int dataOut = 0;
int guardTime = 300;

char buffSerial[24];
int iSerial=0;


volatile byte remoteOn = 0;  // ==1, means remote has been pressed
volatile unsigned int t1_High; // t1 of the start bit
void remoting()  // The ISR
{
    remoteOn=1;
    t1_High=TCNT1;
}

void setup()
{
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, LOW);	 //not ready yet

    pinMode(ir_in, INPUT);
    digitalWrite(ir_in, HIGH);	//We are not using a resistor in IR detector. Pull up

    pinMode(ir_out, OUTPUT);
    digitalWrite(ir_out, LOW);	  //not ready yet

    // start timer
    TCCR1A = 0x00;
    TCCR1B = 0x03;          // 16MHz clock with prescaler means TCNT1 increments every 4uS
    TIMSK1 = 0x00;

    attachInterrupt (0, remoting, RISING);

    // Init DHT
	pinMode(dht_dpin,OUTPUT);
    pinMode(dht_vpin,OUTPUT);
	digitalWrite(dht_dpin,HIGH);
	digitalWrite(dht_vpin,HIGH); 

    Serial.begin(9600);
    digitalWrite(led_pin, HIGH);
}

void loop()
{
    // receiver
    if (remoteOn==1)
    {
        int key = getIRKey();		    //Fetch the key
        if (DEBUG)
        {
            Serial.print("Key Recieved: ");
            Serial.println(key);
        }

        if (key==-1)
        {
            remoteOn=0;  //reset flag
            return;
        }

        // send dummy key to wake receiver
        sendIRKey(0xFFFFFFFF);
        delay(50);

        switch(key)
        {
        case 2336: //Volume up
            sendIRKey(0xE13E11EE);
            delay(200);
            break;
        case 6432: //Volume down
            sendIRKey(0xE13E31CE);
            delay(200);
            break;
        case 1648: //Red
            sendIRKey(0xE13E13EC);
            break;
        case 5744: //Green
            sendIRKey(0xE13EA45B);
            break;
        case 3696: //Yellow
            sendIRKey(0xE13E43BC);
            break;
        case 7792: //Blue
            sendIRKey(0xE13EA15E);
            break;
        default:
            break;
        }
        Serial.print("Key:");
        Serial.println(key);

        remoteOn=0;  //reset flag
    }

    //read serial data
    while (Serial.available()) 
	{
        ReadSerialData(Serial.read());
    }
}

void ReadSerialData(char inChar) 
{
    if (iSerial==25) iSerial=0;
    buffSerial[iSerial++]=inChar;
    if (inChar == '@')
    {
        if (iSerial > 1) 
        {
            iSerial -= 1; //Decrement by 1 to get string size
            ParseSerialCommand();

        }
        iSerial = 0;
    }
}

void ParseSerialCommand() {
    if (iSerial < 1) return;

    int i=0;
    buffSerial[iSerial]=0;
    // process command
    char *strCommand = strtok (buffSerial, " ");
    if (strncmp(buffSerial, "SendIR", 6)==0)
    {
        char *strType = strtok (NULL, " ");
        char *strCode = strtok (NULL, " ");
        // convert code to integer
        long code=strtoul(strCode, NULL , 16);
        if (*strType=='N')
        {
            sendIRKey(code);
        }
    }	
    else if (strncmp(buffSerial, "OK?", 3)==0)
    {
        Serial.println(F("OK!"));
    }

    else if (strncmp(buffSerial, "DHT_On", 6)==0)
    {
        digitalWrite(dht_dpin,HIGH);
        digitalWrite(dht_vpin,HIGH);
    }
    else if (strncmp(buffSerial, "DHT_Read", 8)==0)
    {
		int chk = dht.read11(dht_dpin);
		if (chk==0)
        {
            Serial.print(F("Temperature:\""));
            Serial.print(dht.temperature, DEC);
            Serial.print(F("\". Humidity:\""));
            Serial.print(dht.humidity, DEC);
            Serial.println("\".");
        }
        else
		{
			Serial.print(F("DHT11 Read failed:"));
			Serial.println(chk);
		}            
        //digitalWrite(dht_vpin,LOW); 
    }
}

// for Sony protocol
int getIRKey()
{
    int data[12];
    int longpulse=1;
    digitalWrite(led_pin, LOW);
    // measure the length of start bit
    while (digitalRead(ir_in)==HIGH) //measure start bit;
        if ( (TCNT1-t1_High) > 2500   )  //10 ms timeout
        {
            return -1;
        }

    int t2_High=TCNT1;
    int t_Start=(t2_High-t1_High)*4; // length of start pulse i uS

    if (t_Start>2500 || t_Start<2300 ) // NEC protocol
    {
        return -1;
    }

    // get data
    data[0] = pulseIn(ir_in, HIGH, 2000);      //Start measuring bits
    data[1] = pulseIn(ir_in, HIGH, 2000);
    data[2] = pulseIn(ir_in, HIGH, 2000);
    data[3] = pulseIn(ir_in, HIGH, 2000);
    data[4] = pulseIn(ir_in, HIGH, 2000);
    data[5] = pulseIn(ir_in, HIGH, 2000);
    data[6] = pulseIn(ir_in, HIGH, 2000);
    data[7] = pulseIn(ir_in, HIGH, 2000);
    data[8] = pulseIn(ir_in, HIGH, 2000);
    data[9] = pulseIn(ir_in, HIGH, 2000);
    data[10] = pulseIn(ir_in, HIGH, 2000);
    data[11] = pulseIn(ir_in, HIGH, 2000);

    delay(50); // to slow down the loop if needed

    if (DEBUG)
    {
        Serial.println("-----");
        for(int i=0;i<12;i++)
            Serial.println(data[i]);
    }

    // get data stream
    for(int i=0;i<12;i++)
    {
        if(data[i] > bin_1_S)  //is it a 1?
            data[i] = 1;
        else if(data[i] > bin_0_S) //is it a 0?
            data[i] = 0;
        else
            return -1;			 //I don't know what it is!
    }

    int result = 0;
    int seed = 1;

    for(int i=0;i<12;i++)
    {
        if(data[i])
            result |= 1;
        result=result << 1;
    }
    digitalWrite(led_pin, HIGH);
    return result;			     //Return key number
}

//SENDER
void sendIRKey(long dataOut) {
    int data[32];
    digitalWrite(led_pin, HIGH);     //Ok, i'm ready to send
    for (int i=0; i<32; i++)
    {
        data[i] = dataOut>>i & B1;   //encode data as '1' or '0'
    }

    // send startbit
    digitalWrite(ir_out, HIGH);
    delayMicroseconds(9000);
    // send separation bit
    digitalWrite(ir_out, LOW);
    delayMicroseconds(4500);
    // send the whole string of data
    for (int i=31; i>=0; i--)
    {
        if (data[i] == 1)
        {
            digitalWrite(ir_out, HIGH);
            delayMicroseconds(560);
            digitalWrite(ir_out, LOW);
            delayMicroseconds(1690);
        }
        else
        {
            digitalWrite(ir_out, HIGH);
            delayMicroseconds(560);
            digitalWrite(ir_out, LOW);
            delayMicroseconds(560);
        }
    }
    digitalWrite(ir_out, HIGH);
    delayMicroseconds(560);
    digitalWrite(ir_out, LOW);
    delay(20);
}

// this will write an oscillation at 38KHz for a certain time in useconds
void oscillationWrite(int pin, int time)
{
    for(int i = 0; i <= time/26; i++)
    {
        digitalWrite(pin, HIGH);
        delayMicroseconds(13);
        digitalWrite(pin, LOW);
        delayMicroseconds(13);
    }
}



