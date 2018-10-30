// cfaed coupled PLLs demonstrator
// copyright @Alexandros Pollakis, 2015, Lucas Wetzel 2018

// inculde libraries
#include "ClickButton.h"
#include "Wire.h"
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

// set the baud rate of the serial port
#define BAUDRATE 115200

// set the buffer size (number of samples and maximal possible delay)
#define BUFFER_SIZE 51200

// divider to set the sampling rate
#define SAMPLERATE 100 //300
// i.e. with samplerate 100/ms we have a sampleperiod of Tsample = 1/100E3 = 10E-6 seconds

// scale for transmission delay (choose according to SAMPLERATE)
#define DELAY_SCALE 1  //3

// system pin definitions
#define BOARD_LED 13

// pins for buttons
#define PIN_BUTTON_A 80
#define PIN_BUTTON_B 78
#define PIN_BUTTON_C 79

// available memory size = 2^17 =  128 KB
// Memory for Datacollection 120 KB = 122880 B
// 3x3 = 9 PLLs: 122880 Byte / 9 = 13653,33..
//            -> 13653 Byte pro PLL -> Mem = 122877 Byte
// 2x2 = 4 PLLs: 13653 Byte pro PLL -> Mem =  54612 Byte
// 1x2 = 2 PLLs: 13653 Byte pro PLL -> Mem =  27306 Byte

// number of collectable samples: 13653 Byte/PLL * 8samples/Byte = 109224 samples/PLL
// assumption: open frequency 1kHz
// sample rate: 10us -> 100 samples/periods -> ca. 1100 periods collectable
//               5us -> 200 samples/periods -> ca.  550 periods collectable [* good choice]

// this is the global trigger variable, it will start a measurement, i.e.,
// 1) free running PLLs for a few periods (Tp =~ 1ms)
// 2) turn on coupling, system is at random initial state, EXTENT: use buffer that looks as if the system was in a synched state
// 3) measure the transients after coupling is turned on, until the buffer is full, then flush it to the Serial port
int triggerRun=0;

// operation mode
uint8_t mode;            // mode = 0: free running PLLs (no coupling)
                         // mode = 1: coupled PLL network
                         // mode = 2: transmit data to PC

// recording BUFFER_SIZE
uint8_t dataBuffer[BUFFER_SIZE];
volatile uint16_t bufferIdx;
volatile uint16_t delayedIdx;
volatile int counter;

int kk   = 0;
int idx;
// edgetype contains the information about the type of edge: 1->raising, 0->falling
int edgetype = 0;
int edgePLL;

// output bits and port
// bits:  7   6   5   4   3   2   1   0
// pins:  30  31  32  33  34  35  36  37
// plls:  -   -   2   2   1   1   0   0
volatile uint8_t outBits;

// input bits and port
// bits:  7   6   5   4   3   2   1   0
// pins:  A07 A06 A05 A04 A03 A02 A01 A00
// plls:  -   -   -   -   -   2   1   0
volatile uint8_t inBits;

// transmission delay in sampling steps
volatile int transmissionDelayMax = 99;
volatile int transmissionDelay;

// coupling topologies
// "0": 2 bi-directional coupled PLLs
// "1": 2 bi-directional coupled PLLs
// "2": chain of 3 bi-directional coupled PLLs
// "3": ring of 3 bi-directional coupled PLLs
uint8_t couplingTopology;
volatile boolean coupling = true;

// buttons
ClickButton buttonA(PIN_BUTTON_A,LOW,CLICKBTN_PULLUP);
ClickButton buttonB(PIN_BUTTON_B,LOW,CLICKBTN_PULLUP);
ClickButton buttonC(PIN_BUTTON_C,LOW,CLICKBTN_PULLUP);

// 7 segment LED display
Adafruit_7segment sevseg = Adafruit_7segment();

/**********************************************
                FUNCTIONS
**********************************************/

uint8_t virtualCoupling(uint8_t data){
    uint8_t out = 0x00;                                                         // initialize out with all zeros
    switch(couplingTopology){
        case 0:
            // 2 bi-directionally coupled plls
            bitWrite(out, 0, bitRead(data,1));                                  // PLL 0: 1
            bitWrite(out, 1, bitRead(data,1));                                  // PLL 0: 1
            bitWrite(out, 2, bitRead(data,0));                                  // PLL 1: 0
            bitWrite(out, 3, bitRead(data,0));                                  // PLL 1: 0
            bitWrite(out, 4, 0);                                                // PLL 2: -
            bitWrite(out, 5, 0);                                                // PLL 2: -
            break;
        case 1:
            // 2 bi-directionally coupled plls
            bitWrite(out, 0, 0);                                                // PLL 0: -
            bitWrite(out, 1, 0);                                                // PLL 0: -
            bitWrite(out, 2, bitRead(data,2));                                  // PLL 1: 2
            bitWrite(out, 3, bitRead(data,2));                                  // PLL 1: 2
            bitWrite(out, 4, bitRead(data,1));                                  // PLL 2: 1
            bitWrite(out, 5, bitRead(data,1));                                  // PLL 2: 1
            break;
        case 2:
            // chain of 3 bi-directionally coupled plls
            bitWrite(out, 0, bitRead(data,1));                                  // PLL 0: 1
            bitWrite(out, 1, bitRead(data,1));                                  // PLL 0: 1
            bitWrite(out, 2, bitRead(data,0));                                  // PLL 1: 0
            bitWrite(out, 3, bitRead(data,2));                                  // PLL 1: 2
            bitWrite(out, 4, bitRead(data,1));                                  // PLL 2: 1
            bitWrite(out, 5, bitRead(data,1));                                  // PLL 2: 1
            break;
        case 3:
            // ring of 3 bi-directionally coupled plls
            bitWrite(out, 0, bitRead(data,1));                                  // PLL 0: 1
            bitWrite(out, 1, bitRead(data,2));                                  // PLL 0: 2
            bitWrite(out, 2, bitRead(data,0));                                  // PLL 1: 0
            bitWrite(out, 3, bitRead(data,2));                                  // PLL 1: 2
            bitWrite(out, 4, bitRead(data,0));                                  // PLL 2: 0
            bitWrite(out, 5, bitRead(data,1));                                  // PLL 2: 1
            break;
    }
    return out;
}

// update 7-segment display
void updateDisplay(){
    sevseg.print(transmissionDelay,DEC);
    sevseg.writeDigitNum(0,couplingTopology,false);
    sevseg.writeDigitRaw(1,B01000000);
    if(coupling){
        sevseg.blinkRate(0);
    }
    else{
        sevseg.blinkRate(1);
    }
    sevseg.writeDisplay();
}

// check transmission delay
void checkTransmissionDelay(){
    if (transmissionDelay < 0) transmissionDelay = 0;
    if (transmissionDelay > transmissionDelayMax) transmissionDelay = transmissionDelayMax;
}

void introduceDeltaPertOnlyEnd(){                                               // shifts are given in terms of index of dataBuffer
    Serial.println("Perturbation function started. Change only last state of dataBuffer for PLL 1 (middle)!"); // SOLUTION_3
    if( bitRead(dataBuffer[BUFFER_SIZE-1],1)==0 ){
      bitWrite(dataBuffer[BUFFER_SIZE-1], 1, 1);
    }
    else{
      bitWrite(dataBuffer[BUFFER_SIZE-1], 1, 0);
    }
}

// serial transmit data
void serialWriteDataBuffer(){
    digitalWrite(BOARD_LED, LOW);                                               // turn off board led
    Serial.println("data"); //Serial.flush();                                   // line 'data' to indicate the start of a transmission
    Serial.println(transmissionDelay); //Serial.flush();                        // transmit delay value
    //Serial.write(dataBuffer,BUFFER_SIZE);//Serial.flush();                      // transmit data buffer
    for(int i=0; i<BUFFER_SIZE; i++){
        Serial.print(bitRead(dataBuffer[i],0)); Serial.print("\t"); Serial.print(bitRead(dataBuffer[i],1)); Serial.print("\t"); Serial.print(bitRead(dataBuffer[i],2));
        Serial.print("\n");
    }
    Serial.println(" ");//Serial.flush();                                       // newline
    digitalWrite(BOARD_LED, HIGH);                                              // turn on board led
}

/**********************************************
                    INIT
**********************************************/

void setup() {
    // initialize serial port
    Serial.begin(BAUDRATE);
    Serial.println("Baudrate set!");

    // initialize the board
    pinMode(BOARD_LED, OUTPUT); digitalWrite(BOARD_LED, HIGH);
    // initialize the trigger
    LATFCLR = B11; TRISF = B00;
    // set input ports
    AD1PCFG = 0xFFFF;                                                           // use analogue pins as digital pins
    LATBCLR = 0xFFFF; TRISB = 0xFFFF;                                           // initialize input pins [port: B pins: A0 - A07 ]
    // set output ports
    LATECLR = 0xFFFF; TRISE = 0x0000;                                           // initialize output pins [port: E pins: 37 - 30 ]
    // button definitions
    buttonA.debounceTime      = 20;                                             // debounce time in ms
    buttonB.debounceTime      = 20;
    buttonC.debounceTime      = 20;
    buttonA.multiclickTime    = 250;                                            // time limit for multi clicks
    buttonB.multiclickTime    = 250;
    buttonC.multiclickTime    = 250;
    buttonA.longClickTime     = 1000;                                           // time until "hold down clicks" register
    buttonB.longClickTime     = 1000;
    buttonC.longClickTime     = 1000;
    // clear buffer (set zero)
    memset(dataBuffer, 0, BUFFER_SIZE);
    // "0": 2 bi-directional coupled PLLs
    // "1": 2 bi-directional coupled PLLs
    // "2": chain of 3 bi-directional coupled PLLs
    // "3": ring of 3 bi-directional coupled PLLs
    couplingTopology          = 3;
    // set intitial transmission delay
    transmissionDelay         = 45;
    // pause
    delay(1000);
    // 7 segment display
    sevseg.begin(0x70);
    sevseg.setBrightness(0.2);
          updateDisplay();

    Serial.println("uC is ready! Start measurement!");
    // start timer interrupt to read/write --> will trigger the loop
    attachCoreTimerService(timerISR);
}

/**********************************************
                MAIN LOOP
**********************************************/

void loop() {
    // read button states
    buttonA.Update();
    // buttonB.Update();
    // buttonC.Update();

    // button A
    // double click:  coupling ON/OFF
    // long click:    change coupling topology
    if (buttonA.clicks == 2 ) {                                                 //start measurement
      setup();                                                                  // initialize system, clear buffer
      coupling = !coupling;                                                     // turn off coupling
      delay(1000);                                                              // wait for 1 second for the setup to finish
      period = 10;                                                              // set waiting time in milliseconds before coupling turns on
      bufferIdx = 0; triggerRun = 1;                                            // reset buffer index and trigger the run
      time_now = millis();
      while(millis() < time_now + period){                                      //wait approx. [period] ms -- instead of using delay, since
      }                                                                         // while delay, the program halts and no buffer is written etc.
      coupling = !coupling;                                                     // turn on coupling
      // Serial.println("Coupling on!");
    }
    //if (buttonA.clicks == -1) couplingTopology = (couplingTopology+1) % 4;

    // button B
    // double click:  decrement transmission delay
    // click hold:    decrement till release
    // if (buttonB.clicks == 2 ){
    //     transmissionDelay--;
    //     checkTransmissionDelay();
    // }
    // if (buttonB.clicks == -1){
    //     while(buttonB.depressed == true){
    //         transmissionDelay--;
    //         checkTransmissionDelay();
    //         updateDisplay();
    //         delay(200);
    //         buttonB.Update();
    //     }
    // }

    // button C
    // double click:  increment transmission delay
    // click hold:    increment till release
    // if (buttonC.clicks == 2 ){
    //     transmissionDelay++;
    //     checkTransmissionDelay();
    // }
    // if (buttonC.clicks == -1){
    //     while(buttonC.depressed == true){
    //         transmissionDelay++;
    //         checkTransmissionDelay();
    //         updateDisplay();
    //         delay(200);
    //         buttonC.Update();
    //     }
    // }

    // 7 segment display
    if (buttonA.clicks){                 //|| buttonB.clicks || buttonC.clicks){
        updateDisplay();
    }
}

/**********************************************
              TIMER INTERRUPT
**********************************************/

uint32_t timerISR(uint32_t currentTime){
    // interrupt trigger [PORT 46]
    LATFSET = B11;

    // free running PLLs
    if (!coupling) {
      LATESET               = 0xFF;                                             // set all output pins to HIGH
      inBits                = PORTB bitand 0xFF;                                // read data from PLLs [NOTE: '&' used to be 'bitand']
      dataBuffer[bufferIdx] = inBits;                                           // write input bits to buffer
      bufferIdx             = (bufferIdx + 1) % BUFFER_SIZE;                    // increment buffer index
    }
    // delay coupled PLLs
    else {
      //Serial.println("OK");
      delayedIdx            = (bufferIdx + BUFFER_SIZE - transmissionDelay*DELAY_SCALE)%BUFFER_SIZE;
      outBits               = virtualCoupling(dataBuffer[delayedIdx]);          // read delayed states
      LATESET               = outBits bitand 0xFF;                              // write data [NOTE: '&' used to be 'bitand']
      LATECLR               = ~outBits bitand 0xFF;                             // write data [NOTE: '&' used to be 'bitand']

      inBits                = PORTB bitand 0xFF;                                // read data from PLLs [NOTE: '&' used to be 'bitand']
      dataBuffer[bufferIdx] = inBits;                                           // write input bits to buffer
      bufferIdx             = (bufferIdx + 1) % BUFFER_SIZE;                    // increment buffer index
    }

    LATFCLR = B11;                                                              // trigger OFF

    // data buffer is full
    if (bufferIdx == (BUFFER_SIZE-1) && triggerRun == 1) {
        Serial.println("Buffer full! bufferIdx:");
        Serial.println(bufferIdx,DEC);
        serialWriteDataBuffer();
        delay(10000);
        coupling = !coupling; triggerRun = 0;
        //detachCoreTimerService(timerISR);
    }

    // CORE_TICK_RATE of chipKit max32: 40000 core_ticks per ms, hence a core-tick takes Tcore = 2.5E-8 s
    // hence, if SAMPLERATE = 300 Samples/ms [i.e. 300kHz], then time is incemented in steps of
    // Tsample = 1/300E3 = 3.33E-6 s, which corresponds in turn to Tsample/Tcore = 133.3 core ticks,
    // i.e., CORE_TICK_RATE has to be divided by SAMPLERATE = 300 1/ms, which is 300 kHz
    // this also implies, that the dataBuffer entries in this case are spaced in time by Tsample = 3.33E-6 s
    // to realize a delay of 1ms, one needs to go back bufferIdx = 300 entries in the dataBuffer, which means
    // that is one wants a transmissionDelayMax = 99% of one period of the free-running frequency of ~1000Hz
    // the DELAY_SCALE = 3
    // HENCE: there are a bit more than 3 periods of the free running frequency that fit into the buffer of length 1024
    return(currentTime+CORE_TICK_RATE/SAMPLERATE);
}
