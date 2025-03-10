#include <TM1637Display.h>
#include <Wire.h>
#include <string>
#include <Adafruit_VL53L0X.h>
#include <M2M_LM75A.h>
#include <math.h>

//Pins

  //Distance sensor and temperature pins
    //I2C supports at least 2 devices working on the same signal
  #define SENS_SDA_PIN 21
  #define SENS_SCL_PIN 22

  //Speaker pin
  #define AUDIOAMP_PIN 25

  //Display pins
    //Timer display
    #define TIMERDISP_CLK_PIN 19
    #define TIMERDISP_DIO_PIN 18

    //Temp sensor display
    #define TEMPSENSDISP_CLK_PIN 5  //26     
    #define TEMPSENSDISP_DIO_PIN 17 //25     

  //Button pins
    //Audio tone select
    #define AUDIOTONESELECT_PIN 32

    //Timer mode select
    #define TIMERMODESELECT_PIN 16

    //Variable brightness
    #define TIMERDISPLAYBRIGHTNESS_PIN 4
//


//Global variables
  //Temp display brightness value;
  int dispBrightness = 1;

  //Timer values
    //Time logic values
    unsigned long endTime;
    unsigned long timeLeft;
    bool countingDown = false;
    bool haveShownTime = false;
    bool timerDispIsOn = true;
    bool thingHasBeenRemoved = false;
    bool saveTime = false;
    bool timerStartSeq = true;
    
    // 1 is 10 mins, 0 is 5 mins
    bool timerModes = 1;
    int StartSeqFlash = 0;

    volatile uint32_t remainingTime = 600;

    hw_timer_t * timer = NULL;
    volatile SemaphoreHandle_t timerSemaphore;
    portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

    //Distance sensor
    Adafruit_VL53L0X lox = Adafruit_VL53L0X();
    int lastValidDistance = 0; 

    //Temperature Sensor
    M2M_LM75A lm75a;
    float lastValidTemperature = 0.0;


  //Speaker logic
  const int audioOutPin = 25;  // ESP32 DAC pin (GPIO 25 or 26)
  const int waveFreq = 550;    // Target frequency (550 Hz)
  const float volumeScale = 0.1; // Adjust volume (1.0 = max)

  int sampleRates[] = {11050, 12500, 9040};  // Sample rates for 440 Hz, 420 Hz, and 464 Hz
  int currentSampleRateIndex = 0;  // Track current sample rate
  unsigned long lastToggleTime = 0;
  const int toggleInterval = 2000; // 2 seconds
    
  bool startTheSpeaker = false;

  //Demo toggle
    bool haveStartedDemo = false;
    bool demo2 = false;
    bool demo3 = false;
    int demoTemp = 50;
    bool forDemoTime = true;

  TM1637Display timerDisplay(TIMERDISP_CLK_PIN, TIMERDISP_DIO_PIN);
  TM1637Display tempDisplay(TEMPSENSDISP_CLK_PIN, TEMPSENSDISP_DIO_PIN);


// Timer ISR: called every second.
    void ARDUINO_ISR_ATTR onTimer() {
    // Give a semaphore that we can check in the loop
    xSemaphoreGiveFromISR(timerSemaphore, NULL);
    }

  void soundAlarm(void * parameter) {
  while(1) {
    if(startTheSpeaker) {
      // Serial.println("The speaker is going off.");
      int sampleRate = sampleRates[currentSampleRateIndex];
      int samplesPerWave = round((float)sampleRate / waveFreq);
      int delayTime = round(1000000.0 / sampleRate);
      
      // Generate one cycle of the sine wave continuously
      for (int i = 0; i < samplesPerWave; i++) {
        // Check if the speaker should still be playing
        if(!startTheSpeaker) break;
        float angle = (2.0 * M_PI * i) / samplesPerWave;
        int value = (sin(angle) * (127 * volumeScale)) + 128;
        dacWrite(audioOutPin, value);
        delayMicroseconds(delayTime);
      }
    } else {
      // If not playing, yield a little to avoid hogging the CPU
      delay(10);
    }
  }

}


void setup() {
  Serial.begin(115200);          // Start Serial communication

  //Timer setup
  timer = timerBegin(10000000);
  timerSemaphore = xSemaphoreCreateBinary();
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 10000000, true, 0);

  //Display setup
  timerDisplay.setBrightness(dispBrightness);     
  tempDisplay.setBrightness(dispBrightness); 


  //Sensors setup
    Wire.begin(SENS_SDA_PIN, SENS_SCL_PIN);
    if (!lox.begin()) {
        Serial.println("Failed to detect VL53L0X sensor! Check wiring.");
        while (1);  // Stop execution if the sensor is not found
    }

  //Button setup
  pinMode(TIMERDISPLAYBRIGHTNESS_PIN, INPUT_PULLUP);
  pinMode(TIMERMODESELECT_PIN, INPUT_PULLUP);
  pinMode(AUDIOTONESELECT_PIN, INPUT_PULLUP);

  //Background speaker logic
  xTaskCreatePinnedToCore(soundAlarm, "Audio Task", 2*2048, NULL, 1, NULL, 1);

}

void readTemperature() {
  float temperature = lm75a.getTemperature();
  if (isnan(temperature) || temperature < -50 || temperature > 150) {  // Check for invalid readings
        //Serial.println("Invalid temperature reading! Using last known temperature.");
        temperature = lastValidTemperature;
    } else {
        lastValidTemperature = temperature;  // Update last valid temperature
    }

  // Serial.print("Temperature: ");         //Output the temperature to the Serial Monitor terminal
  // Serial.print(temperature);
  // Serial.println(" °C\n");


    temperature = temperature * 100;                              //Move the decimal place of the temperature value to work with it easier
    long tempInThousands = static_cast<long>(temperature);        //Copy the floating value as a long value to allow for large values later when converted to hex
    
    long displayArray[4] = {0};                                   //Create an array to store the value of each number in the temperature, initializing with all 0's
    displayArray[0] = tempInThousands / 1000;                     //Find the tens value of the original temp (thousands for this value) and store it in displayArray
    displayArray[1] = (tempInThousands / 100) % 10;               //Find the ones value of the original temp (hundrends for this value) and store it in displayArray
    displayArray[2] = (tempInThousands / 10) % 10;                //Find the tenths value of the original temp (tens for this value) and store it in displayArray
                                                                  
                                                                  //The hundreths value of the original temp (ones for this value) is intentionally kept at 0 to allow...
                                                                  //...the last display to always show 'c' for celcius

    long displayTemp = displayArray[0]*16*16*16                   //Create a value to store the hex equivalent of the temp, with the final display gaurenteed... 
    + displayArray[1]*16*16 + displayArray[2]*16 + 12;            //...to be 'c' in hexidecimal because we and always add 12 (c in hex) to the always-zero hundreths 

    sendTempToDisplay(displayTemp);

}

int readDistance(){

  delay(1000);
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  int distance = measure.RangeMilliMeter;

  if (measure.RangeStatus != 0 || distance == 0 || distance > 2000) {
      //Serial.println("Invalid distance reading! Using last known distance.");
      distance = lastValidDistance;
  } else {
      lastValidDistance = distance;
  }
  
  return distance;
}

void toggleDispBrightness(){
  
  if(dispBrightness > 2){
    dispBrightness = 0;
  }

  dispBrightness++;

  timerDisplay.setBrightness(dispBrightness);
  tempDisplay.setBrightness(dispBrightness);


  Serial.print("The display brightness is now set to: ");
  Serial.print(dispBrightness);
  Serial.print("\n");
}



void showTimerMode(unsigned long minutes){
  Serial.print("The current timer mode is: ");
  Serial.print(minutes);
  Serial.print(" minutes");
  Serial.print("\n");
  haveShownTime = true;
}

void startCountDown(volatile uint32_t minutes){
  remainingTime = minutes;

  Serial.print("Countdown started: ");
  Serial.print(minutes);
  Serial.print(" minutes");
  Serial.print("\n");
  
}

void toggleTimerDisplay(){
  
  if(timerDispIsOn){
    Serial.print("Display is off\n");
    timerDispIsOn = false;
    timerDisplay.setBrightness(0);
    // sendTimeToDisplay(remainingTime);

  }
  else{
    timerDispIsOn = true;
    if(dispBrightness > 2 ){
      timerDisplay.setBrightness(dispBrightness);
    }
    else{
      timerDisplay.setBrightness(2);
    }
    // sendTimeToDisplay(remainingTime);
    Serial.print("Display is on\n");
  }

}

void sendTempToDisplay(int displayTemp){

  tempDisplay.showNumberHexEx(displayTemp, 0b01000000, false, 4,0); //Send the temperature to the display as a hexidecimal value, turn on the colon, no leading-zeros,...
                                                                  //...a length of 4 digits, with most significant bit at the leftmost display

  // Serial.print("This is the signal that will be sent to the temperature display: ");
  // Serial.print(displayTemp);
  // Serial.print("\n");

}

void sendTimeToDisplay(volatile uint32_t timeLeft){

  int seconds = timeLeft % 60;
  int minutes = timeLeft / 60;

  // Serial.print("This is the signal that will be sent to the timer display: ");
  // Serial.print(minutes);
  // Serial.print(":");
  // Serial.print(seconds);
  // Serial.print("\n");

  int nums = 0000;
  nums += 100*minutes;
  nums += seconds;


  timerDisplay.showNumberDecEx(nums,0b01000000, true, 4, 0);
}

void toggleTimerMode(){
  
  // if (haveStartedDemo || demo2){
  //   remainingTime = 1*60;
  // }
  StartSeqFlash = 0;
  if(timerModes == 1){
    remainingTime = 5*60;
    timerModes = 0;
  }
  else{
    remainingTime = 10*60;
    timerModes = 1;
  }

  timerStartSeq = true;

  haveShownTime = false;
}

void toggleSpeakerMode(){
  
  currentSampleRateIndex++;
  
  if(currentSampleRateIndex > 2){
    currentSampleRateIndex = 0;
  }

  

  Serial.print("The speaker tone is now set to: ");
  Serial.print(currentSampleRateIndex);
  Serial.print("\n");


}

bool prevBrightnessButtonState = HIGH;
bool prevTimerModeButtonState = HIGH;
bool prevToneSelectButtonState = HIGH;


void loop() {
    
  //Distance sensor logic
  int distance = readDistance();
  Serial.println(distance);
  if(distance > 30){
    thingHasBeenRemoved = true;
  }
  else{
    thingHasBeenRemoved = false;
  }


  //Timer logic
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
    // Read the interrupt count and time

    uint32_t timeLeft;
    portENTER_CRITICAL(&timerMux);
    timeLeft = remainingTime;
    portEXIT_CRITICAL(&timerMux);
    
    // Calculate minutes and seconds for display.
    uint32_t minutes = timeLeft / 60;
    uint32_t seconds = timeLeft % 60;
    Serial.printf("Time remaining: %02d:%02d\n", minutes, seconds);
    


    if(!timerStartSeq){

      sendTimeToDisplay(remainingTime);
      // if(demo3){
      // toggleDispBrightness();
      // sendTempToDisplay(demoTemp);
      // toggleSpeakerMode();
      // demoTemp++;
      // }


      if(!thingHasBeenRemoved){
        if(remainingTime > 0){
          remainingTime--;
          startTheSpeaker = false;
          timerDisplay.setBrightness(dispBrightness);
        }
        else{
          Serial.printf("Timer ended signal sent\n");
          toggleTimerDisplay();
          startTheSpeaker = true;
        }
      }
      else{
        Serial.printf("TIMERPAUSED\n");
        toggleTimerDisplay();
        startTheSpeaker = true;
      }
    }
    else{
      startTheSpeaker = false;
      sendTimeToDisplay(remainingTime);
      StartSeqFlash++;
      if(StartSeqFlash == 6){
        timerStartSeq = false;
        StartSeqFlash = 0;
      }
      toggleTimerDisplay();
    }

  }


  //Check if any of the buttons are pressed
  bool brightnessButtonState = digitalRead(TIMERDISPLAYBRIGHTNESS_PIN);
  bool timerModeButtonState = digitalRead(TIMERMODESELECT_PIN);
  bool audioToneButtonState = digitalRead(AUDIOTONESELECT_PIN);

  if(brightnessButtonState == LOW && prevBrightnessButtonState == HIGH){
    toggleDispBrightness();
    Serial.print("TIMERDISPLAYBRIGHTNESS_PIN enables\n");
  }
  if(timerModeButtonState == LOW && prevTimerModeButtonState == HIGH){
    toggleTimerMode();
    Serial.print("TIMERMODESELECT_PIN enables\n");
  }
  if(audioToneButtonState == LOW && prevToneSelectButtonState == HIGH){
    toggleSpeakerMode();
    Serial.print("AUDIOTONESELECT_PIN enables\n");
  }

  prevBrightnessButtonState = brightnessButtonState;
  prevTimerModeButtonState = timerModeButtonState;
  prevToneSelectButtonState = audioToneButtonState;

  //Read temp and send it to the temp display
  readTemperature();

  // delay(50);

  // //Demos
  // if(haveStartedDemo){
  //   toggleTimerMode();
  //   //startCountDown(60);

  //   haveStartedDemo = false;
  // }

  if(demo2){
    if(forDemoTime){
      // toggleTimerMode();
      forDemoTime =  false;
    }
    if(remainingTime <= 590){
      thingHasBeenRemoved = true;
    }
  }
  

  
}


