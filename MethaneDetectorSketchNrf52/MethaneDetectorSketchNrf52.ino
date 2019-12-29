#include <bluefruit.h>

#define STATUS_LED (19)
/* Home Environment Sensing Service Definitions
   Home Environment Service:  0x28FF
   Methane Sensing Char: 0x2A3A
*/

// Power Reduction: https://github.com/adafruit/Adafruit_nRF52_Arduino/issues/165
// Serial seems to increase consumption by 500uA https://github.com/adafruit/Adafruit_nRF52_Arduino/issues/51#issuecomment-368289198
#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(x)  Serial.print(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif


const char* DEVICENAME = "Methane HP%X%X";
const char* DEVICE_MODEL = "HomePi Motion";
const char* DEVICE_MANUFACTURER = "Rounin Labs";

/**
   Home Environmental Sensing Service.
   This service exposes measurement data from a home sensor intended for home automation applications.
   A wide range of environmental parameters is supported.
 **/
const int UUID16_SVC_HOME_ENV_SENSE = 0x28FF;
const int UUID16_CHR_METHANE_SENSE_MEASUREMENT = 0x4A3A;

const int VAL_METHANE_DETECTED = 0x01; // Methane detected 

volatile int methaneDetectedValue = 0x00;

BLEService        hess = BLEService(UUID16_SVC_HOME_ENV_SENSE);
BLECharacteristic msc = BLECharacteristic(UUID16_CHR_METHANE_SENSE_MEASUREMENT);

BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance


const int BATTERY_INFO_PIN = A7;
// Level at which to signal low battery
const int LOW_BATT = 25;
int lastBattLevel = 100;
// Interval at which the battery is reported in MS
const int BATT_REPORTING_INTERVAL = 60000;

const int iMethaneInterruptPin = A2;

/**
   Notification task informaiton
*/
static TaskHandle_t _notifyMethaneValueHandle;
static TaskHandle_t _taskToNotify;

uint32_t _notifyMethaneValueStackSize = 512;
void TaskNotifyMethane(void * pvParameters);


void setup()
{
  
  #ifdef DEBUG
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
  #endif
  
  DEBUG_PRINTLN("Setting up Methane Sensor");
  DEBUG_PRINTLN("-----------------------\n");

  pinMode(STATUS_LED, OUTPUT);

  // Initialise the Bluefruit module
  DEBUG_PRINTLN("Initialise the Bluefruit nRF52 module");
  Bluefruit.begin();

  // We'll control the LED so we can save some power.
  Bluefruit.autoConnLed(false);

  // Set the advertised device name (keep it short!)
  DEBUG_PRINT("Setting Device Name to ");
  uint8_t address [6];
  Bluefruit.Gap.getAddr(address);
  char nameBuff[50] = "";
  sprintf(nameBuff, DEVICENAME, address[1], address[0]);
  DEBUG_PRINTLN(nameBuff);
  Bluefruit.setName(nameBuff);

  // Set the connect/disconnect callback handlers
  Bluefruit.setConnectCallback(connect_callback);
  Bluefruit.setDisconnectCallback(disconnect_callback);

  // Configure and Start the Device Information Service
  DEBUG_PRINTLN("Configuring the Device Information Service");
  bledis.setManufacturer(DEVICE_MANUFACTURER);
  bledis.setModel(DEVICE_MODEL);
  bledis.begin();

  // Start the BLE Battery Service and set it to 100%
  DEBUG_PRINTLN("Configuring the Battery Service");
  blebas.begin();
  blebas.write(100);

  DEBUG_PRINTLN("Configuring the Methane Sensor Service");
  setupLS();

  // Setup the advertising packet(s)
  DEBUG_PRINTLN("Setting up the advertising payload(s)");
  startAdv();
  DEBUG_PRINTLN("\nAdvertising");

  //Setup Motion Sensor.
  setupMethaneSensor();
  setupMethaneDectectionInterrupts();

  // Setup FreeRTOS notification tasks
  DEBUG_PRINTLN("Setting up FreeRTOS notification task(s)");
  setupNotificationTasks();

  DEBUG_PRINTLN("\nSetup Complete!");
}

void connect_callback(uint16_t conn_handle)
{
  char central_name[32] = { 0 };
  Bluefruit.Gap.getPeerName(conn_handle, central_name, sizeof(central_name));

  DEBUG_PRINT("Connected to ");
  DEBUG_PRINTLN(central_name);
  // Disable the BT connection LED to save battery.
  digitalWrite(STATUS_LED, LOW);
}

/**
   Helper function to notify the battery level.
*/
void notifyBatteryLevel(int level)
{
  blebas.notify(level);
}

/**
   Callback invoked when a connection is dropped
   @param conn_handle connection where this event happens
   @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
   https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/cores/nRF5/nordic/softdevice/s140_nrf52_6.1.1_API/include/ble_hci.h
*/
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  DEBUG_PRINTLN("Disconnected");

  // Consider ligthing LED when it is disconnected.
}

void cccd_callback(BLECharacteristic& chr, uint16_t cccd_value)
{
  // Display the raw request packet
  DEBUG_PRINT("CCCD Updated: ");
  //Serial.printBuffer(request->data, request->len);
  DEBUG_PRINT(cccd_value);
  DEBUG_PRINTLN("");

  // Check the characteristic this CCCD update is associated with in case
  // this handler is used for multiple CCCD records.
  if (chr.uuid == msc.uuid)
  {
    if (chr.notifyEnabled())
    {
      DEBUG_PRINTLN("Methane Sensing Measurement 'Notify' enabled");
      // Notify current CO detection.
      notifyMethaneDetectionValue(methaneDetectedValue);
    }
    else
    {
      DEBUG_PRINTLN("Methane Sensing Measurement 'Notify' disabled");
    }
  }
      
}

void setupLS(void)
{
  // Configure the Motion Sensing service
  // Supported Characteristics:
  // Name                         UUID    Requirement Properties
  // ---------------------------- ------  ----------- ----------
  // Methane Sense Measurement    0x2A3A  Mandatory   Notify
  hess.begin();

  // Note: You must call .begin() on the BLEService before calling .begin() on
  // any characteristic(s) within that service definition.. Calling .begin() on
  // a BLECharacteristic will cause it to be added to the last BLEService that
  // was 'begin()'ed!

  msc.setProperties(CHR_PROPS_NOTIFY);
  msc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  msc.setFixedLen(2);
  msc.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  msc.setUserDescriptor("Methane Sense Measurement. A value of greater than 0 means detected.");
  msc.begin();
  uint8_t lsdata[1] = { 0 }; // Set the characteristic to use 8-bit values, with the sensor connected and detected
  msc.notify(lsdata, 1);                   // Use .notify instead of .write!

}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include Motion Sensor Service UUID
  Bluefruit.Advertising.addService(hess);

  // Include Name
  Bluefruit.Advertising.addName();

  /* Start Advertising
     - Enable auto advertising if disconnected
     - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
     - Timeout for fast mode is 30 seconds
     - Start(timeout) with timeout = 0 will advertise forever (until connected)

     For recommended advertising interval
     https://developer.apple.com/library/content/qa/qa1931/_index.html
  */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

/**
   Set up to read from the Carbon Monoxide Sensor ouput pin.
*/
void setupMethaneSensor()
{
    pinMode(iMethaneInterruptPin, INPUT);
}

void setupMethaneDectectionInterrupts()
{
    attachInterrupt(digitalPinToInterrupt(iMethaneInterruptPin), methaneChanged, CHANGE);
}

/**
   Interrupt function called when
   there is a change on the pins detecting a CO.
*/
void methaneChanged()
{

  BaseType_t xHigherPriorityTaskWoken;

  /* Clear the interrupt. */
  //prvClearInterruptSource();
  /*
  NRF_GPIOTE->EVENTS_IN[0] = 0;
  NRF_GPIOTE->EVENTS_IN[1] = 0;
  NRF_GPIOTE->EVENTS_IN[2] = 0;
  NRF_GPIOTE->EVENTS_IN[3] = 0;
  NRF_GPIOTE->EVENTS_IN[4] = 0;
  NRF_GPIOTE->EVENTS_IN[5] = 0;
  NRF_GPIOTE->EVENTS_IN[6] = 0;
  NRF_GPIOTE->EVENTS_IN[7] = 0;
  */

  // TODO: Either clear the interrupt or ensure only notification happens on 
  // change so we don't bombard the remote device with updates of the 
  // same value. 
  /* xHigherPriorityTaskWoken must be initialised to pdFALSE.
    If calling vTaskNotifyGiveFromISR() unblocks the handling
    task, and the priority of the handling task is higher than
    the priority of the currently running task, then
    xHigherPriorityTaskWoken will be automatically set to pdTRUE. */
  xHigherPriorityTaskWoken = pdFALSE;

  /* Unblock the handling task so the task can perform any processing
    necessitated by the interrupt.  xHandlingTask is the task's handle, which was
    obtained when the task was created.  vTaskNotifyGiveFromISR() also increments
    the receiving task's notification value. */
  vTaskNotifyGiveFromISR( _notifyMethaneValueHandle, &xHigherPriorityTaskWoken);

  /* Force a context switch if xHigherPriorityTaskWoken is now set to pdTRUE.
    The macro used to do this is dependent on the port and may be called
    portEND_SWITCHING_ISR. */
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void setupNotificationTasks()
{
  xTaskCreate(
    TaskNotifyMethane
    , (const portCHAR *)"NotifyMethane" // A name just for humans
    , _notifyMethaneValueStackSize // Stack Size
    , NULL // Parameters, should be address to variable on heap not stack.
    , TASK_PRIO_NORMAL //TASK_PRIO_LOW // priority
    , &_notifyMethaneValueHandle
  );
}

void TaskNotifyMethane(void * pvParameters)
{
  (void)pvParameters;
  BaseType_t xEvent;
  uint32_t ulNotifiedValue;
  for (;;)
  {

    // Get the this task's handle.
    //_taskToNotify = xTaskGetCurrentTaskHandle();
    
    /* Block indefinitely (without a timeout, so no need to check the function's
        return value) to wait for a notification.  Here the RTOS task notification
        is being used as a binary semaphore, so the notification value is cleared
        to zero on exit.  NOTE!  Real applications should not block indefinitely,
        but instead time out occasionally in order to handle error conditions
        that may prevent the interrupt from sending any more notifications. */
    ulNotifiedValue = ulTaskNotifyTake( pdTRUE,          /* Clear the notification value before
                                           exiting. */
                                        portMAX_DELAY ); /* Block indefinitely. */

    DEBUG_PRINTLN("Methane Task Notified");
        
    /* ulNotifiedValue holds a count of the number of outstanding
      interrupts.  Process each in turn. */
    if ( ulNotifiedValue > 0 )
    {
      if (Bluefruit.connected())
      {
        DEBUG_PRINTLN("Notifying Methane Data");
        // Use this chance to update the battery value as well.
        int batteryLevel = readBatteryLevel();
        notifyBatteryLevel(batteryLevel);

        if (batteryLevel < LOW_BATT)
        {
          digitalToggle(LED_RED);
        }

        int methaneReading = getMethaneReading();
        
        DEBUG_PRINT("Methane: "); DEBUG_PRINTLN(methaneReading);

        methaneDetectedValue = methaneReading;
        notifyMethaneDetectionValue(methaneDetectedValue);
      }
      else
      {
        //digitalToggle(STATUS_LED);
      }

    }

    // The task suspends itself after notifying.
    //vTaskSuspend( NULL );
  }

}

int getMethaneReading()
{
  int methaneReading = 0;

  methaneReading = digitalRead(iMethaneInterruptPin) != HIGH ? 0x01 : 0x00;
  
  return methaneReading;
}

void notifyMethaneDetectionValue(int value)
{
  uint8_t methaneData[1] = {value};
  if (msc.notify(methaneData, sizeof(methaneData)))
  {
    DEBUG_PRINT("Methane Value Measurement updated to: ");
    DEBUG_PRINTLN(methaneData[0]);
  }
  else
  {
    DEBUG_PRINTLN("ERROR: Notify not set in the CCCD or not connected!");
  }

}

void loop()
{
  if (Bluefruit.connected() )
  {
    int batteryLevel = readBatteryLevel();

    // Notify the battery level only if it has changed.
    if (batteryLevel != lastBattLevel)
    {
      notifyBatteryLevel(batteryLevel);
      lastBattLevel = batteryLevel;
    }

  }

  // Only send update every BATT_REPORTING_INTERVAL milliseconds
  delay(BATT_REPORTING_INTERVAL);

}

/**
   Reads the battery level from the feather pin.
   Note: Updated from: https://learn.adafruit.com/adafruit-feather-32u4-basic-proto/power-management
*/

/**
   Excerpt From https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/libraries/Bluefruit52Lib/examples/Hardware/adc_vbat/adc_vbat.ino
*/
#define VBAT_PIN          (A7)
#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider

int readVBAT(void)
{
  int raw;

  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  // Let the ADC settle
  delay(1);

  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(VBAT_PIN);

  // Set the ADC back to the default settings
  analogReference(AR_DEFAULT);
  analogReadResolution(10);

  return raw;
}

uint8_t mvToPercent(float mvolts) {
  uint8_t battery_level;

  if (mvolts >= 3000)
  {
    battery_level = 100;
  }
  else if (mvolts > 2900)
  {
    battery_level = 100 - ((3000 - mvolts) * 58) / 100;
  }
  else if (mvolts > 2740)
  {
    battery_level = 42 - ((2900 - mvolts) * 24) / 160;
  }
  else if (mvolts > 2440)
  {
    battery_level = 18 - ((2740 - mvolts) * 12) / 300;
  }
  else if (mvolts > 2100)
  {
    battery_level = 6 - ((2440 - mvolts) * 6) / 340;
  }
  else
  {
    battery_level = 0;
  }

  return battery_level;
}

int readBatteryLevel()
{
  /*
    float measuredvbat = analogRead(BATTERY_INFO_PIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    DEBUG_PRINT("VBat: " ); DEBUG_PRINTLN(measuredvbat);
    DEBUG_PRINT("CBat: " ); DEBUG_PRINTLN(map(measuredvbat, 3.0, 4.2, 0, 100));
  */
  int vbat_raw = readVBAT();

  // Convert from raw mv to percentage (based on LIPO chemistry)
  uint8_t vbat_per = mvToPercent(vbat_raw * VBAT_MV_PER_LSB);
  DEBUG_PRINT("VBat: " ); DEBUG_PRINT(vbat_per); DEBUG_PRINTLN("%");
  return vbat_per;
}
