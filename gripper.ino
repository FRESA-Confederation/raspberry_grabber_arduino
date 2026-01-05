  #include <Servo.h>
  #include <pitches.h>

  int backoffTicks = 0;
  const int backoffTicksNeeded = 3;
  bool ignoreContactOnce = false;

  unsigned long lastToggleTime = 0;
  const unsigned long toggleInterval = 1000; // ms

  int melody[] = {
    NOTE_E5, NOTE_E5, NOTE_E5,
    NOTE_E5, NOTE_E5, NOTE_E5,
    NOTE_E5, NOTE_G5, NOTE_C5, NOTE_D5,
    NOTE_E5,
    NOTE_F5, NOTE_F5, NOTE_F5, NOTE_F5,
    NOTE_F5, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_E5,
    NOTE_E5, NOTE_D5, NOTE_D5, NOTE_E5,
    NOTE_D5, NOTE_G5
  };

  int durations[] = {
    8, 8, 4,
    8, 8, 4,
    8, 8, 8, 8,
    2,
    8, 8, 8, 8,
    8, 8, 8, 16, 16,
    8, 8, 8, 8,
    4, 4
  };



/* SERVOs DEFINITION *///////////////////////////////////////////////////////////
  
  Servo gripper;
  Servo classifier; 
  Servo trapdoor; 

/* SERVO SECTION END *///////////////////////////////////////////////////////////


/* ASCII KEYBOARD CHARACTERS *///////////////////////////////////////////////////
  const char keyDetect = 'A';   // Key to start color detection
  const char keyRelease = 'R';  // Key to trigger release of trapdoor
  const char keyIdle = 'Z';     // Key to set the system back to IDLE state
  const char keyGrasp = 'G';    // Key to manually set grasping 
  
/*ASCII KEYBOARD END *///////////////////////////////////////////////////////////

/* PIN DEFINITION SECTION /// Define Pins for Actuators and Sensors *////////////

  const int fsrPin = A0;        // Force Sensor 
  const int ldrPin = A1;        // Light/Color Sensor 

  const int gripperPin = 9;     // Servo from the gripper
  const int classifierPin = 10;   // Servo for the classifier
  const int trapdoorPin = 11;       // Servo for the trapdoor opening 

  const int buttonPin = 7;      // Trigger button pin 
  const int releasePin = 8;     // Reomving this --------------------------------
  const int ledPin = 12;        // Leds used for the color detection 

  const int bigLedPin = 2; 
  const int smallLedPin = 3; 

  const int ripeLedPin = 5; 
  const int unripeLedPin = 4; 

  const int buzzerPin = 6; 

  // redled 5, whiteled 4, yellow 3, blue 2 
/* PIN DEFINITION SECTION ENDS */////////////////////////////////////////////////

/* GRIPPER SECTION VARIABLES *///////////////////////////////////////////////////
  // Angles for the Gripper Servo 
  const int openAngle   = 140;  // servo totalmente abierto
  const int closedAngle = 60;   // servo totalmente cerrado
  /* Original [57, 135] */

  /* FSR VARIABLES *//**//**///**/

  /* Amount of force detected before stopping grasping action */
  const float normThres = 0.10; // Tweak probably to 0.3 
  const int   rawMax = 1023;             // Max possible sensor value
  const int   rawMin = 0;                // Min possible sensor value 

  /* Filter Variables */
  float fsrFiltered = 0.0;
  const float alpha = 0.3; // Also tweak probably to 0.2 

  /* Contact Variables */ 
  int rawPrev = 0; 
  bool contactDetected = false; 
  const int backoffDeg = 12;

    // Tunables  
    const int contactRawThres = 17;   // 10
    const int contactDeltaThres = 9;  // 7 // 8



  // Initial state during initiation 
  float currentAngle = openAngle;

  /* Dynamic parameters for the grasping action */ 
  const float maxRateDegPerLoop = 2.0;   // Rate of change for the angle during grasping 


  /* Measurements */
  float maxDiameter = 62;     // mm 
  float minDiameter = 15;     // mm 

/* GRIPPER SECTION END*/////////////////////////////////////////////////////////

/* CLASSIFIER SECTION VARIABLES*////////////////////////////////////////////////
  const int noneAngle = 90; 
  const int bigAngle = 140; 
  const int smallAngle = 50; 

  int classAngle = noneAngle; 

/* CLASSIFIER SECTION END */////////////////////////////////////////////////////

/* TRAPDOOR SECTION VARIABLES */////////////////////////////////////////////////
  const int lockAngle = 100; 
  const int unlockAngle = 10; 

  int trapdoorAngle = lockAngle; 

/* TRAPDOOR SECTION END *///////////////////////////////////////////////////////

/* LDR COLOR SECTION VARIABLES *////////////////////////////////////////////////
  const int ldr_samples = 50; 
  const int ldr_delay_ms = 5; 
  const int ldr_settle_ms = 50; 

  int highThreshold = 75;  //70 //100
  int lowThreshold = 65; //60   //60 
/* LDR SECTION ENDS *///////////////////////////////////////////////////////////

/* Random iteration variables */
  bool taco = true;   
  int tacoTimer = 0;
/* Section Ends */

/* ---- debounce variables for trigger button ---- *//////////////////////////REMOVE LATER 
  int lastButtonStable = HIGH;
  int lastButtonRaw    = HIGH;
  unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 20; // ms

  //////////////////////////////////////////////////////////////////////////////

  /* Enum typedef states for the FSM */
  typedef enum 
  {
    /* Main States for Gripper */
    IDLE, 
    GRASPING,
    GRABBING,
    GRABBED,
    DETECTION, 
    RELEASE,

  } SystemState;

  /* Color detection states */
  typedef enum 
  {
    UNCERTAIN,
    RIPE, 
    UNRIPE, 
  } Ripeness; 

  /* Size detection states */
  typedef enum
  {
    NONE,
    BIG,
    SMALL,

  } Size;

  Size raspberrySize = NONE;
  Ripeness ripeness = UNCERTAIN; 
  SystemState state = IDLE; 

  // Auxiliary Function for Button Debouncing 
  bool readButtonPressed() {
    int raw = digitalRead(buttonPin);
    unsigned long now = millis();

    if (raw != lastButtonRaw) {
      lastDebounceTime = now;
      lastButtonRaw = raw;
    }

    // si se mantiene estable más de debounceDelay, lo tomamos como válido
    if ((now - lastDebounceTime) > debounceDelay) {
      if (raw != lastButtonStable) {
        lastButtonStable = raw;
        // flanco de HIGH -> LOW (con INPUT_PULLUP eso significa "click")
        if (lastButtonStable == LOW) {
          return true;
        }
      }
    }

    return false;
  }


  
  float mapRaspberry(float angleDeg) 
  {
    return map(angleDeg, closedAngle, openAngle, minDiameter, maxDiameter);
  }
  

  bool classifyRaspberry(float diameter)
  {
    return diameter > 20.0; 

  }

  /*Auxiliaty functions for color detection */
  int readAverageAnalog(int pin, int samples, int delayMs) 
  {
    long sum = 0;
    for (int i = 0; i < samples; i++) {
      sum += analogRead(pin);
      delay(delayMs);
    }
    return (int)(sum / samples);
  }
  
  // LED OFF -> ambient
  int measureAmbientLDR() 
  {
    digitalWrite(ledPin, LOW);
    delay(ldr_settle_ms);
    return readAverageAnalog(ldrPin, ldr_samples, ldr_delay_ms);
  }

  int measureNetLDR() 
  {
    int ambient = measureAmbientLDR();

    digitalWrite(ledPin, HIGH);
    delay(ldr_settle_ms);
    int onVal = readAverageAnalog(ldrPin, ldr_samples, ldr_delay_ms);

    digitalWrite(ledPin, LOW);

    int net = onVal - ambient;
    if (net < 0) net = 0;
    return net;
  }


  // Hysteresis classification
  Ripeness classifyRipenessNet(int net) {
    if (net > highThreshold) return UNRIPE;
    if (net < lowThreshold)  return RIPE;
    // between thresholds: keep UNCERTAIN or treat as "uncertain"
    return UNCERTAIN;
  }


  char cmd = 0; 

  void setup() {
    Serial.begin(9600);

    /* Servos Setup */
    gripper.attach(gripperPin); 
    classifier.attach(classifierPin); 
    trapdoor.attach(trapdoorPin); 

    gripper.write((int)currentAngle);
    delay(1000); 

    classifier.write((int)classAngle); 
    delay(1000); 
    
    trapdoor.write((int)trapdoorAngle); 
    //delay(2000); 

    /* PinMode Setup */
    pinMode(fsrPin, INPUT); 
    pinMode(ldrPin, INPUT); 
    pinMode(buttonPin, INPUT_PULLUP);
    //pinMode(releasePin, INPUT_PULLUP);
    pinMode(ledPin, OUTPUT); 

    /* Visual Interfacing Leds */
    pinMode(ripeLedPin, OUTPUT); 
    pinMode(unripeLedPin, OUTPUT); 
    pinMode(bigLedPin, OUTPUT); 
    pinMode(smallLedPin, OUTPUT); 

    pinMode(buzzerPin, OUTPUT); 
    

  }

  void loop() {

    // Read the button 
    bool buttonPressed = readButtonPressed();

    // Read Keyboard 
    if (Serial.available() > 0) cmd = Serial.read();

    // Read the FSR Sensor 
    int raw = analogRead(fsrPin);
    fsrFiltered = fsrFiltered + alpha * (raw - fsrFiltered);

    // Get Force Derivative 
    int dRaw = raw - rawPrev; 
    rawPrev = raw; 
    
    

    // Normalize lecture to [0, 1]
    //    0  -> no force
    //    1  -> max force
    float norm = (float)(fsrFiltered - rawMin) / (float)(rawMax - rawMin);
    norm = constrain(norm, 0.0, 1.0);
    
    // Calc Degrees per Loop Rate Based on 
    //    When norm = 0  then rate = maxRateDegPerLoop
    //    When norm = 1  then rate = 0
    float rate = maxRateDegPerLoop * (1.0 - norm);

    switch (state)
    {
      case IDLE:
        // Fuga 
        currentAngle = openAngle;

        contactDetected = false; 
        rawPrev = raw; 

        
        if (taco)
        {
          classifier.write(noneAngle);
          delay(500);

          trapdoor.write(lockAngle);  
          delay(500); 
          taco = false; 
          digitalWrite(ripeLedPin, HIGH); 
          digitalWrite(unripeLedPin, HIGH); 
          digitalWrite(bigLedPin, HIGH); 
          digitalWrite(smallLedPin, HIGH); 
          digitalWrite(buzzerPin, LOW);
        }
        
        
        
        // Set all sensing states back to IDLE 
        ripeness = UNCERTAIN; 
        raspberrySize = NONE; 

        if (millis() - lastToggleTime >= toggleInterval) {
        lastToggleTime = millis();

        digitalWrite(unripeLedPin, !digitalRead(unripeLedPin));
        digitalWrite(ripeLedPin,   !digitalRead(ripeLedPin));
        digitalWrite(bigLedPin,    !digitalRead(bigLedPin));
        digitalWrite(smallLedPin,  !digitalRead(smallLedPin));
        //digitalWrite(buzzerPin,    !digitalRead(buzzerPin));
        }



        /*
        digitalWrite(ripeLedPin, HIGH); 
        digitalWrite(unripeLedPin, HIGH); 
        digitalWrite(bigLedPin, HIGH); 
        digitalWrite(smallLedPin, HIGH); 
        digitalWrite(buzzerPin, LOW);
        */
        




        // Await button to commence action 
        if (cmd == keyDetect)
        {
          digitalWrite(ripeLedPin, LOW); 
          digitalWrite(unripeLedPin, LOW); 
          digitalWrite(bigLedPin, LOW); 
          digitalWrite(smallLedPin, LOW); 
          delay(100); 
          taco = true; 
          state = DETECTION; 
          cmd = 0; 
          
        }
        else if (cmd == keyGrasp) 
        {
          digitalWrite(ripeLedPin, LOW); 
          digitalWrite(unripeLedPin, LOW); 
          digitalWrite(bigLedPin, LOW); 
          digitalWrite(smallLedPin, LOW);
          contactDetected = false; 
          rawPrev = raw;
          state = GRASPING; 
          cmd = 0; 
        }
        else if (cmd == keyRelease)
        {
          state = RELEASE; 
          cmd = 0; 
        }

        break; 
      case DETECTION: {
          // Take ONE measurement and classify
          int net = measureNetLDR();
          ripeness = classifyRipenessNet(net);

          
          Serial.print("LDR net = ");
          Serial.println(net);
          

          if (ripeness == RIPE) {

                      
            
            Serial.println("Ripeness: RIPE");
            contactDetected = false; 
            rawPrev = raw;
            ignoreContactOnce = true;

            digitalWrite(ripeLedPin, HIGH);  

            Serial.print("Enter GRASPING angle=");
            Serial.println(currentAngle);

            state = GRASPING; 

          } else if (ripeness == UNRIPE) {
            digitalWrite(unripeLedPin, HIGH); 
            Serial.println("Ripeness: UNRIPE");
            state = IDLE; 
          } else {
            Serial.println("Ripeness: UNCERTAIN/UNCERTAIN (between thresholds)");
            state = IDLE; 
          }
        
        break; 
          }
      case GRASPING: 

        /* This section should go in an additional state, somwhat as "DETECTION" functioning as the LDR logic */ 
        /*
        if (taco)
        {
          digitalWrite(ledPin, HIGH); 

          delay(1000); 

          digitalWrite(ledPin, LOW); 

          delay(1000); 
          taco = false; 
        }
        */
        /* Section ENDS */

        if (ignoreContactOnce) {
          rawPrev = raw;
          ignoreContactOnce = false;
          break;  // no cierres ni detectes contacto este loop
        }

        if (!contactDetected)
        {
          currentAngle -= rate; 

          if ((raw > contactRawThres) && (dRaw > contactDeltaThres))
          {
            contactDetected = true; 
            backoffTicks = 0;
            state = GRABBING; 
            break; 

          }

          if (currentAngle <= closedAngle) 
          {
            state = GRABBING;
            break;
          }
          

        }
        else
        {
          // seguridad: si por algo contactDetected quedó true, ve a backoff
          state = GRABBING;
          break;  
        }


        if (cmd == keyIdle) 
        {
          state = IDLE;
        }

        /*
        if (currentAngle <= closedAngle)
        {
          
          /*for(int i = 0; i <= 15; i++)
          {
            currentAngle++; 
          }
          currentAngle = closedAngle; 
          

          state = GRABBED; 
        }
        else if (norm > norm_thres)
        {
          state = GRABBING; 
        }


        if (cmd == keyIdle) {
          state = IDLE;
        }

        currentAngle -= rate;
        */
        break;

      case GRABBING: 
        
        currentAngle = min(currentAngle + backoffDeg, openAngle);
        delay(200);
        state = GRABBED; 
        
        /*
        if (backoffTicks == 0) {
          currentAngle = min(currentAngle + backoffDeg, openAngle); // jalón
        }
        backoffTicks++;
        if (backoffTicks >= backoffTicksNeeded) {
          state = GRABBED;
        }
        */
          

        

        break; 

      case GRABBED:  
  
        //if (contactDetected && norm < 0.02) contactDetected = false;

        if((raspberrySize == NONE) && (raw < contactRawThres) && !contactDetected)
        {
          Serial.print("Nothing Grabbed, just closed"); 
          classifier.write(smallAngle); 

        }
        else //if (raspberrySize == NONE)
        {
          float raspberryDiam = mapRaspberry(currentAngle - backoffDeg/4);  

          Serial.print("Raspberry Diameter in mm: "); 
          Serial.println(raspberryDiam); 

          raspberrySize = classifyRaspberry(raspberryDiam) ? BIG : SMALL; 


            

          classifier.write(raspberrySize == BIG ? bigAngle : smallAngle); 
          
          delay(100); 

          digitalWrite(bigLedPin, raspberrySize == BIG ? HIGH : LOW ); 
          digitalWrite(smallLedPin, raspberrySize == SMALL ? HIGH : LOW);
          

          delay(100); 

          Serial.print("Raspberry is: "); 
          Serial.println(raspberrySize == BIG ? "BIG" : "SMALL");
          Serial.print("Press Button to Release");

          

        }

        delay(4000); 
        gripper.write(openAngle);
        currentAngle = openAngle;
        delay(1500); 
        taco = true; 
        state = IDLE; 


        if(cmd == keyIdle)
        {
          gripper.write(openAngle);
          currentAngle = openAngle;
          delay(1000);  //1500
          taco = true; 
          state = IDLE; 
          
        }

        break; 
      
      case RELEASE: 
        trapdoor.write(unlockAngle); 

        int size = sizeof(durations) / sizeof(int);

        

        if(!taco)
        {
          taco = true;

          digitalWrite(unripeLedPin, LOW);
          digitalWrite(ripeLedPin, LOW);
          digitalWrite(bigLedPin, LOW);
          digitalWrite(smallLedPin, LOW);

          for (int note = 0; note < size; note++) {
          
          //to calculate the note duration, take one second divided by the note type.
          //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
          int duration = 1000 / durations[note];
          tone(buzzerPin, melody[note], duration);

          //to distinguish the notes, set a minimum time between them.
          //the note's duration + 30% seems to work well:
          int pauseBetweenNotes = duration * 1.70;

          switch(melody[note])
          {
            case NOTE_D5: 
              digitalWrite(ripeLedPin, HIGH); 
              break;
            case NOTE_E5: 
              digitalWrite(unripeLedPin, HIGH); 
          
              break; 
            case NOTE_F5:
              digitalWrite(bigLedPin, HIGH); 
              break; 
            case NOTE_G5: 
              digitalWrite(smallLedPin, HIGH);
              break; 


          }


          delay(pauseBetweenNotes);
          
          
          
          //stop the tone playing:
          noTone(buzzerPin);

          digitalWrite(unripeLedPin, LOW);
          digitalWrite(ripeLedPin, LOW);
          digitalWrite(bigLedPin, LOW);
          digitalWrite(smallLedPin, LOW);
          delay(20);
          }
        }

        if (cmd == keyIdle) 
        {
          //trapdoor.write(lockAngle); 
          
          //delay(500); 
          taco = true;
          cmd = 0;
          state = IDLE; 
          
          
        }
        break; 

    }



    // Limitar al ángulo mínimo (cerrado total)
    if (currentAngle > openAngle) currentAngle = openAngle;
    if (currentAngle < closedAngle) currentAngle = closedAngle;

    // Mover el servo
    gripper.write((int)currentAngle);

    // Debug por Serial
    
    Serial.print("raw = ");
    Serial.println(raw);
    /*
    Serial.print("\t norm = ");
    Serial.print(norm, 3);
    Serial.print("\t rate = ");
    Serial.print(rate, 3);
    Serial.print("\t angle = ");
    Serial.println(currentAngle, 1);
    */
    

    Serial.println(state == IDLE ? "IDLE" : state == GRASPING ? "GRASPING" : state == GRABBING ? "GRABBING" : state == GRABBED ? "GRABBED" : "RELEASE");


    /*
    if (tacoTimer < 10)
    {
      tacoTimer++; 
    }
    else 
    {
      tacoTimer = 0; 
      taco = !taco; 
      digitalWrite(ledPin, taco ? HIGH : LOW); 
    }
    */
    


    
    //cmd = 0;

    delay(20);  // controla el "dt" de tu sistema (PRIMERO FUE 20 LO QUE SON 50HZ)
  }
