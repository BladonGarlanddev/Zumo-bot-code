uint16_t timeInThisState()
{
  return (uint16_t)(millis() - stateStartTime);
}

void turn(int deg) {
  if (deg > 0) {
    motors.setSpeeds(400, -400);
  } else {
    motors.setSpeeds(-400, 400);
  }
  delay(1.25 * abs(deg));
}

void startFunctionA() {
  for (int i = 0; i < 5; i++) {
    delay(1000);
    buzzer.playNote(NOTE_G(3), 50, 12);
  }
  turn(-60);
  forward(2);
  motors.setSpeeds(0,0);
  scanDir = DirectionRight;
  return;
}

void startFunctionB() {
  for (int i = 0; i < 5; i++) {
    delay(1000);
    buzzer.playNote(NOTE_G(3), 50, 12);
  }
  turn(60);
  scanDir = DirectionLeft;
  forward(2);
}

void startFunctionC() {
  for (int i = 0; i < 5; i++) {
    delay(1000);
    buzzer.playNote(NOTE_G(3), 50, 12);
  }
  changeState(StateDriving);
  setUp = 'c';

}

char buttonMonitor() {
  if (buttonA.getSingleDebouncedPress()) {
    return 'A';
  }

  if (buttonB.getSingleDebouncedPress()) {
    return 'B';
  }

  if (buttonC.getSingleDebouncedPress()) {
    return 'C';
  }

  return 0;
}

void select() {
  while (1) {
    switch (buttonMonitor()) {
      case 'A':
        startFunctionA();
        return;

      case 'B':
        startFunctionB();
        return;

      case 'C':
        startFunctionC();
        return;
    }
  }
}

void forward(float d) {
  acc.readAcceleration(loop_start_time);
  motors.setSpeeds(400, 400);
  for(int i = 0; i < d*10; i++) {
    delay((d/10)*75);
    if(check_for_contact() == true) {
      changeState(StateCollision);
    }
  }
  
}

bool check_for_contact()
{
  static long threshold_squared = (long) XY_ACCELERATION_THRESHOLD * (long) XY_ACCELERATION_THRESHOLD;
  return ((acc.ss_xy_avg() >  threshold_squared) && (millis() - stateStartTime > NEW_STATE_DELAY));
 
}

void changeState(uint8_t newState)
{
  state = (State)newState;
  justChangedState = true;
  stateStartTime = millis();
  display.clear();
  displayCleared = true;
}

void setup()
{
  
  lineSensors.initThreeSensors();
  prox.initThreeSensors();

  Wire.begin();

  acc.init();
  acc.enableDefault();

#ifdef LOG_SERIAL
  acc.getLogHeader();
#endif

  randomSeed((unsigned int) millis());

  select();
  
  if(setUp != 'c') {
    changeState(StateScanning);
  } else {
    changeState(StateCollision);
  }
}

void loop()
{
  loop_start_time = millis();
  acc.readAcceleration(loop_start_time);
  
  if (state == StateBacking)
  {
    if (justChangedState) {
      justChangedState = false;
      display.print(F("back"));
    }

    if(double_line == 'r') {
      motors.setSpeeds(0,400);
      delay(350);
      motors.setSpeeds(400,400);
      delay(350);
      double_line='0';
    } else if(double_line == 'l') {
      motors.setSpeeds(400,0);
      delay(350);
      motors.setSpeeds(400,400);
      delay(350);
      double_line='0';
    } else {
       motors.setSpeeds(-reverseSpeed, -reverseSpeed);
    }
    // After backing up for a specific amount of time, start
    // scanning.
      if (timeInThisState() >= reverseTime)
      {
       changeState(StateScanning);
      }
  }

  else if (state == StateScanning)
  {
    // In this state the robot rotates in place and tries to find
    // its opponent.
        if (justChangedState)
    {
      justChangedState = false;
      display.print(F("scan"));
      prox.read();
      if((prox.countsFrontWithLeftLeds() + prox.countsLeftWithLeftLeds())>(prox.countsFrontWithRightLeds() + prox.countsRightWithRightLeds())) {
        scanDir = DirectionLeft;
      } else {
        scanDir = DirectionRight;
      }

    }

    if (scanDir == DirectionRight)
    {
      motors.setSpeeds(400, -400);
    }
    else
    {
      motors.setSpeeds(-400, 400);
    }

    lineSensors.read(sensor_values);
    if (sensor_values[0] < lineSensorThreshold)
    {
      double_line = 'l';
      changeState(StateBacking);
    }
    else if (sensor_values[2] < lineSensorThreshold)
    {
      double_line = 'r';
      changeState(StateBacking);
    }
    uint16_t time = timeInThisState();
      // Read the proximity sensors.  If we detect anything with
      // the front sensor, then start driving forwards.
      prox.read();
      if (prox.countsFrontWithLeftLeds() >= 2
        || prox.countsFrontWithRightLeds() >= 2)
      {
        changeState(StateDriving);
      } 
  }
  else if (state == StateDriving) //new state here
  {
    // In this state we drive forward while also looking for the
    // opponent using the proximity sensors and checking for the
    // white border.
       
    if (justChangedState)
    {
      justChangedState = false;
    }

   
    // Check for borders.
    lineSensors.read(sensor_values);
    if (sensor_values[0] < lineSensorThreshold)
    {
      if(timeInThisState() < 100) {
        double_line = 'l';
      } else {
         scanDir = DirectionRight;
      }
        changeState(StateBacking);
        return;
      
    }
    else if (sensor_values[2] < lineSensorThreshold)
    {
       if(timeInThisState() < 100) {
        double_line = 'r';
      } else {
        changeState(StateBacking);
        scanDir = DirectionRight;
        return;
      }
    }

    // Read the proximity sensors to see if know where the
    // opponent is.
    prox.read();
    uint8_t sum = prox.countsFrontWithRightLeds() + prox.countsFrontWithLeftLeds();
    int8_t diff = prox.countsFrontWithRightLeds() - prox.countsFrontWithLeftLeds();

    if (sum >= 6)
    {
      motors.setSpeeds(rammingSpeed, rammingSpeed);
    }
    else if (sum == 0)
    {
      motors.setSpeeds(400,400);
      if (prox.countsLeftWithLeftLeds() >= 2)
      {
        
        scanDir = DirectionLeft;
        changeState(StateScanning);
      }

      if (prox.countsRightWithRightLeds() >= 2)
      {
        // Detected something to the right.
        scanDir = DirectionRight;
        changeState(StateScanning);
      }
    }
    else
    {
      // We see something with the front sensor but it is not a
      // strong reading.
      int veerSpeed = 400 - (abs(diff) * 100);
      veerSpeed = constrain(veerSpeed, 150, 400);
      if (diff >= 2)
      {
        // The right-side reading is stronger, so veer to the right.
        motors.setSpeeds(400, veerSpeed);
        scanDir = DirectionRight;
      }
      else if (diff <= -2)
      {
        // The left-side reading is stronger, so veer to the left.
        motors.setSpeeds(veerSpeed, 400);
        scanDir = DirectionLeft;
      }
      else
      {
        // Both readings are equal, so just drive forward.
        motors.setSpeeds(400, 400);
      }
    }
     if(check_for_contact() == true) {
        changeState(StateCollision);
      }
     }
  else if(state == StateCollision) 
  {

    if(justChangedState) {
      justChangedState = false;
      display.print(F("collide"));
      buzzer.playFromProgramSpace(sound_effect);
    }

    lineSensors.read(sensor_values);
    if (sensor_values[0] < lineSensorThreshold)
    {
      changeState(StateBacking);
      return;
    }
    if (sensor_values[2] < lineSensorThreshold)
    {
      changeState(StateBacking);
      return;
    } else 
    {
      motors.setSpeeds(400,400);
    }
  }
}
