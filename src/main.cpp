#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>

#define DEVICE_OCTAVE 3
#define RECIEVER 
#define DISABLE_THREADS
#define DISABLE_ISR
#define TEST_SCANKEYS
// #define TEST_DISPLAY_UPDATE
// #define TEST_DECODE
// #define TEST_CAN_TX
// #define TEST_SAMPLE_ISR
// #define TEST_CAN_RX_ISR
// #define TEST_CAN_TX_ISR




struct {
std::bitset<32> inputs;
SemaphoreHandle_t mutex;
std::bitset<2> state = 0;
int knob3Rotation = 0;
} sysState;

//Constants
  const uint32_t interval = 100; //Display update interval

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

QueueHandle_t msgInQ, msgOutQ;
SemaphoreHandle_t CAN_TX_Semaphore;

uint8_t RX_Message[8] = {0};

// variable to store step size of key that is pressed
volatile uint32_t currentStepSize;

// Array to store the message to be sent over CAN

// Array to store the step size of each note

constexpr std::array<uint32_t, 12> calcStepSize(){
  std::array<uint32_t, 12> stepSizes = {};
  double twelfthRootOfTwo = 1.0594630943592953;


  for(int i = 0; i < 12; i++){
    stepSizes[i] = static_cast<uint32_t>(std::pow(2, 32) * (3520 >> (7-DEVICE_OCTAVE)) * std::pow(twelfthRootOfTwo, i - 9) / 22000);
  }
  return stepSizes;
}

class Knob {
  public:
  Knob(int id): prevState(0b00), rotation(0), rotationChange(0), lowerVol(0), upperVol(8){}

  void updateRotation(const std::bitset<32>& inputs) {
        curState[0] = inputs[12];
        curState[1] = inputs[13];

        if((prevState == 0b11 && curState == 0b10) ||
          (prevState == 0b00 && curState == 0b01)){
          rotationChange = 1;
        }
        else if((prevState == 0b01 && curState == 0b00) ||
                (prevState == 0b10 && curState == 0b11)){
          rotationChange = -1;
        }
        else if(!(prevState == ~curState)){
          rotationChange = 0;
        }

        rotation += rotationChange;
        rotation = std::min(std::max(rotation, lowerVol), upperVol);
        prevState = curState;

    }

  int get_rotation(){
    return rotation;
  }

  void set_limit(int lower, int upper){
    upperVol = upper;
    lowerVol = lower;
  }

  private:
  std::bitset<2> curState, prevState;
  int rotationChange, rotation;
  int lowerVol, upperVol;
};



const std::string notes[] = {
  "C",
  "C#",
  "D",
  "D#",
  "E",
  "F",
  "F#",
  "G",
  "G#",
  "A",
  "A#",
  "B",
  ""
};
std::string note = "";

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}


std::bitset<4> readCols(){
  std::bitset<4> result;

  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);

  return result;  
}

void setRow(uint8_t rowIdx){  

  digitalWrite(REN_PIN, LOW);
  //i.e. if rowIdx is 2 then digitalWrite(RA2) will be the only row to be high
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, (rowIdx >> 1) & 0x01);
  digitalWrite(RA2_PIN, (rowIdx >> 2) & 0x01);

  digitalWrite(REN_PIN, HIGH);
  
}

void selectOctave(int octave){
  // TODO implement to be able to selct the octave at runtime (???)
}



void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}
void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

void sampleISR() {
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;

  int32_t Vout = (phaseAcc >> 24) - 128;
  Vout = Vout >> (8 - sysState.knob3Rotation);
  analogWrite(OUTR_PIN, Vout + 128);
}

void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS; // xFreq = initiation interval of task, set to 50 ms (converted to RTOS ticks)
  TickType_t xLastWakeTime = xTaskGetTickCount(); // xLast = stores time (tick count) of last initiation, initialised with API call to get current time
  
  std::array<uint32_t, 12> stepSizes = calcStepSize(); // could be changed to constexpr

  Knob knob3(3);

  uint8_t TX_Message[8] = {0};

  #ifdef TEST_SCANKEYS

    uint32_t localCurrentStepSize = 0;

    for(uint8_t row = 0; row < 4; row++){
      setRow(row);
      delayMicroseconds(3);
      std::bitset<4> cols = readCols();
      sysState.inputs &= ~(0xF << row * 4);
      sysState.inputs |= std::bitset<32>(readCols().to_ulong() << row * 4);
    }

    // Generate a key press message for each of the 12 keys
    for(int i = 0; i < 12; i++){
      localCurrentStepSize = stepSizes[i];
      note = notes[i];
      TX_Message[0] = 'P';
      TX_Message[1] = DEVICE_OCTAVE;
      TX_Message[2] = i;
      xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
    }

    knob3.updateRotation(sysState.inputs);

    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
    __atomic_store_n(&sysState.knob3Rotation, knob3.get_rotation(), __ATOMIC_RELAXED);

  #else
  

  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency ); // blocks execution until xFreq ticks have passed since last exec of loop. Places thread in a waiting state
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    uint32_t localCurrentStepSize = 0;
    // bool keyPressed = false;

    for(uint8_t row = 0; row < 4; row++){
      setRow(row);
      delayMicroseconds(3);
      std::bitset<4> cols = readCols();
      sysState.inputs &= ~(0xF << row * 4);
      sysState.inputs |= std::bitset<32>(readCols().to_ulong() << row * 4);
    }

    for(int i = 0; i < 12; i++){
      if(!(sysState.inputs[i])){
        localCurrentStepSize = stepSizes[i];
        note = notes[i];
        TX_Message[0] = 'P';
        TX_Message[1] = DEVICE_OCTAVE;
        TX_Message[2] = i;
        xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
      }
      else if(note == notes[i]){
        TX_Message[0] = 'R';
        TX_Message[1] = DEVICE_OCTAVE;
        TX_Message[2] = i;
        note = notes[12];
        xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
      }
    }
    knob3.updateRotation(sysState.inputs);
    // CAN_TX(0x123, TX_Message);

    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
    __atomic_store_n(&sysState.knob3Rotation, knob3.get_rotation(), __ATOMIC_RELAXED);

    // this is apparently how __atomic_load_n(&currentStepSize, &localCurrentStepSize, __ATOMIC_RELAXED); is implemented
    // localCurrentStepSize = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);

    xSemaphoreGive(sysState.mutex);
  }
  #endif
}



void displayUpdateTask(void * pvParameters){
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS; 
  TickType_t xLastWakeTime = xTaskGetTickCount();


  #ifdef TEST_DISPLAY_UPDATE
    uint32_t ID = 0x123;

    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font

    u8g2.setCursor(2,20);
    u8g2.print(sysState.inputs.to_ulong() & 0xFFF, HEX);

    u8g2.setCursor(2,30);
    u8g2.print(note.c_str());

    u8g2.setCursor(40,20);
    u8g2.print(sysState.knob3Rotation);

    u8g2.setCursor(66,30);
    u8g2.print(char(RX_Message[0]));
    u8g2.print(RX_Message[1]);
    u8g2.print(RX_Message[2]);
  
    u8g2.sendBuffer(); // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);
    
  #else

  while(1){
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);

    uint32_t ID = 0x123;
    


    while (CAN_CheckRXLevel())
	    CAN_RX(ID, RX_Message);

    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font

    u8g2.setCursor(2,20);
    u8g2.print(sysState.inputs.to_ulong() & 0xFFF, HEX);

    u8g2.setCursor(2,30);
    u8g2.print(note.c_str());

    u8g2.setCursor(40,20);
    u8g2.print(sysState.knob3Rotation);

    u8g2.setCursor(66,30);
    u8g2.print(char(RX_Message[0]));
    u8g2.print(RX_Message[1]);
    u8g2.print(RX_Message[2]);
  
    u8g2.sendBuffer(); // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);
    xSemaphoreGive(sysState.mutex);
  }
  #endif
}



void decodeTask(void * pvParameters){
  std::array<uint32_t, 12> stepSizes = calcStepSize();
  uint32_t localCurrentStepSize = 0;

  #ifdef TEST_DECODE
      xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);

      if(RX_Message[0] == 'P'){
        localCurrentStepSize = stepSizes[RX_Message[2]];
      }
      __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
    
  #else

  while(1){
    xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);

    if(RX_Message[0] == 'P'){
      localCurrentStepSize = stepSizes[RX_Message[2]];
    }
    else if(RX_Message[0] == 'R'){
      localCurrentStepSize = 0;
    }
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
  }

  #endif
}

void CAN_TX_Task (void * pvParameters) {
	uint8_t msgOut[8];
  #ifdef RECIEVER
    while (1) {
      xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
      xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
      CAN_TX(0x123, msgOut);
    }
  #endif
}


void setup() {
  // put your setup code here, to run once:

  //Set pin directions

  
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(9600);
  Serial.println("1Hello World");


  
    //Initialise timer
    TIM_TypeDef *Instance = TIM1;
    HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  
    // Timer is configured by setting period, attaching ISR, and stating the timer
    #ifndef DISABLE_ISR
      sampleTimer->setOverflow(22000, HERTZ_FORMAT);
      sampleTimer->attachInterrupt(sampleISR);
      sampleTimer->resume();
    #endif
    sysState.mutex = xSemaphoreCreateMutex();
  
    CAN_Init(true); // Places CAN hardware in loopback mode, will recieve and ack its own messages
    // ^^ attach 2nd keyboard and change above to false to disable loopback mode
    setCANFilter(0x123,0x7ff); // 2nd param is the mask, 0x7ff means all bits are compared
    CAN_RegisterRX_ISR(CAN_RX_ISR);
    CAN_RegisterTX_ISR(CAN_TX_ISR);
    CAN_Start();

    #ifdef DISABLE_THREADS
        msgInQ = xQueueCreate(384,8); // 1st param = no. items it can store, 2nd param = size of each item in bytes
        msgOutQ = xQueueCreate(384,8);
    #else
        msgInQ = xQueueCreate(36,8); // 1st param = no. items it can store, 2nd param = size of each item in bytes
        msgOutQ = xQueueCreate(36,8);
    #endif
  
    CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);


  // Initialise FreeRTOS
  #ifndef DISABLE_THREADS
    TaskHandle_t scanKeysHandle = NULL;
    xTaskCreate(
      scanKeysTask,   // func that implements the task
      "scanKeysTask", // task name
      64,             // stack size in words, not bytes
      NULL,           // parameters to pass to the task
      2,              // task priority
      &scanKeysHandle // pointer to store the task handle
    );

  // #else

    TaskHandle_t displayUpdateHandle = NULL;
    xTaskCreate(displayUpdateTask, "displayUpdateTask", 256, NULL, 1, &displayUpdateHandle);

    TaskHandle_t decodeHandle = NULL;
    xTaskCreate(decodeTask, "decodeTask", 32, NULL, 2, &decodeHandle);

    TaskHandle_t CAN_TX_Handle = NULL;
    xTaskCreate(CAN_TX_Task, "CAN_TX_Task", 32, NULL, 2, &CAN_TX_Handle);
    vTaskStartScheduler();
  #endif
  

  #ifdef TEST_SCANKEYS
      Serial.println("Testing scanKeysTask...");
	    uint32_t startTime = micros();
	    for (int iter = 0; iter < 32; iter++) {
	    	scanKeysTask(nullptr);
	    }
	    Serial.println((micros()-startTime)/32);
  #endif

  delayMicroseconds(2000);

  Serial.println("outside scan");

  #ifdef TEST_CAN_TX
    // Serial.println("Testing CAN_TX_Task...");
    uint32_t startTime3 = micros();
    for (int iter = 0; iter < 32; iter++) {
      CAN_TX_Task(nullptr);
    }
    Serial.println((micros()-startTime3)/32);
  #endif

  delayMicroseconds(200);

  Serial.println("outside can tx");

  #ifdef TEST_SAMPLE_ISR
    // Serial.println("Testing sampleISR...");
    uint32_t startTime4 = micros();
    for (int iter = 0; iter < 32; iter++) {
      sampleISR();
    }
    Serial.println((micros()-startTime4)/32);
  #endif


  delayMicroseconds(200);

  Serial.println("outside sample isr");


  #ifdef TEST_CAN_RX_ISR
    // Serial.println("Testing CAN_RX_ISR...");
    uint32_t startTime5 = micros();
    for (int iter = 0; iter < 32; iter++) {
      CAN_RX_ISR();
    }
    Serial.println((micros()-startTime5)/32);
  #endif

  delayMicroseconds(200);
  Serial.println("outside can rx");


  #ifdef TEST_CAN_TX_ISR
    // Serial.println("Testing CAN_TX_ISR...");
    uint32_t startTime6 = micros();
    for (int iter = 0; iter < 32; iter++) {
      CAN_TX_ISR();
    }
    Serial.println((micros()-startTime6)/32);
  #endif

  delayMicroseconds(200);


  #ifdef TEST_DECODE
    // Serial.println("Testing decodeTask...");
    uint32_t startTime2 = micros();
    for (int iter = 0; iter < 32; iter++) {
      decodeTask(nullptr);
    }
    Serial.println((micros()-startTime2)/32);
  #endif

  delayMicroseconds(200);


  #ifdef TEST_DISPLAY_UPDATE
    // Serial.println("Testing displayUpdateTask...");
    uint32_t startTime1 = micros();
	  for (int iter = 0; iter < 32; iter++) {
	  	displayUpdateTask(nullptr);
	  }
	  Serial.println((micros()-startTime1)/32);
  #endif
  

  delayMicroseconds(200);

  while(1);
  
}

void loop() {
  
}