// ================================ LIBRARIES ================================
// RTC --------------------------
#include "Arduino.h"     // Arduino
#include "uRTCLib.h"     // RTC
#include "uEEPROMLib.h"  // EEPROM
// LCD --------------------------
#include "Wire.h"               // I2C
#include "LiquidCrystal_I2C.h"  // LCD Utils
// uSD --------------------------
#include "SPI.h"
#include "SD.h"
// DHT --------------------------
#include "DHT.h"  // DHT
// MQ135 --------------------------
#include "MQ135.h"

// ================================ DEFINES ================================
#define isNaN(X) (!((X) == (X)))

// ================================ CONFIGS ================================
const byte devs = 2;
String cellDevelopers[devs] = { "09623217042", "09612931412" };
String cellBeneficiary = "09092328129";  // 09092328129
const float rangeTemp[2] = { 34.0, 36.0 };
const float rangeHum[2] = { 55.0, 65.0 };
const float thresholdPPM = 37;
const byte rangePWM[2] = { 100, 250 };

// ================================ PIN CONFIGURATION ================================
// FANS --------------------------
const byte countFAN = 5;
const byte pinsFAN[countFAN] = { 9, 10, 11, 12, 13 };  // intake, toChamber, exhaust, blower
// DHT --------------------------
const byte dhtCount = 4;
const byte dhtPins[dhtCount] = { 4, 6, 7, 5 };
// MQ --------------------------
const int mqPin = A0;
// BUTTON --------------------------
const byte btnCount = 2;
byte btnPins[btnCount] = { 33, 32 };
// VOLTAGE SENSOR --------------------------
const byte pinVoltSense = A1;

// ================================ UTILS ================================
// STRING MANIPULATOR ----------------------------------------------------------------------------
struct {
  String StringMultiplier(String value, int count = 1) {
    String result = "";
    for (byte i = 0; i < count; i++) result += value;
    return result;
  }
  String centerText(String value, int limit = 20) {
    if (value.length() >= limit) return value;
    byte spaces = 20 - value.length();
    byte spaceLeft = spaces / 2;
    byte spaceRight = (spaces / 2) + (spaces % 2);
    return StringMultiplier(" ", spaceLeft) + value + StringMultiplier(" ", spaceRight);
  }
  String leftText(String value, int limit = 20) {
    if (value.length() >= limit) return value;
    byte spaces = 20 - value.length();
    return value + StringMultiplier(" ", spaces);
  }
  String rightText(String value, int limit = 20) {
    if (value.length() >= limit) return value;
    byte spaces = 20 - value.length();
    return StringMultiplier(" ", spaces) + value;
  }
  String toString(long value) {
    return value < 10 ? "0" + String(value) : String(value);
  }
  String toStringFloat(float value) {
    return value < 10 ? "0" + String(value) : String(value);
  }
  String arrayToString(int length, String* value) {
    String result = "";
    for (int i = 0; i < length; i++) {
      result += value[i];
      if (i + 1 == length) break;
      result += "\n";
    }
    return result;
  }
} StringManipulator;


// CALENDAR ----------------------------------------------------------------------------
struct {
  byte days[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
  bool isLeapYear(byte year) {
    return year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
  }
  byte daysInMonth(byte year, byte month) {
    byte tempMonth = month--;
    return days[tempMonth] + ((tempMonth != 1 || !isLeapYear(year)) ? 0 : 1);
  }
  byte getThreshold(byte i, byte year, byte month) {
    switch (i) {
      case 1:
        return 12;
      case 2:
        return daysInMonth(year, month);
      case 3:
        return 24;
      case 4:
        return 60;
      case 5:
        return 60;
      default:
        return 0;
    }
  }
  String getIdentifier(byte i) {
    switch (i) {
      case 0:
        return "yrs";
      case 1:
        return "mnt";
      case 2:
        return "day";
      case 3:
        return "hrs";
      case 4:
        return "min";
      case 5:
        return "sec";
    }
  }
  String toString(long value) {
    return value < 10 ? "0" + String(value) : String(value);
  }
} Calendar;

// DATE AND TIME ----------------------------------------------------------------------------
struct DateTime {
  byte year, month, day, hour, minute, second;
  byte value[6];
  void get() {
    value[0] = year;
    value[1] = month;
    value[2] = day;
    value[3] = hour;
    value[4] = minute;
    value[5] = second;
  }
  void setArray(byte* arr) {
    for (byte i = 0; i < 6; i++) set(i, arr);
  }
  void setRTC(uRTCLib rtc) {
    value[0] = rtc.year();
    value[1] = rtc.month();
    value[2] = rtc.day();
    value[3] = rtc.hour();
    value[4] = rtc.minute();
    value[5] = rtc.second();
  }
  void set(byte i, byte value) {
    switch (i) {
      case 0:
        year = value;
        break;
      case 1:
        month = value;
        break;
      case 2:
        day = value;
        break;
      case 3:
        hour = value;
        break;
      case 4:
        minute = value;
        break;
      case 5:
        second = value;
        break;
      default:
        break;
    }
  }
  void fromString(String _value) {
    byte iStart = 0, iEnd = 0;
    byte tempValue[6];
    for (byte i = 0; i < 6; i++) {
      iEnd = _value.indexOf(' ', iStart);
      if (iEnd == -1) {
        tempValue[i] = _value.substring(iStart, _value.length()).toInt();
        break;
      }
      tempValue[i] = _value.substring(iStart, iEnd).toInt();
      iStart = iEnd + 1;
    }
    setArray(tempValue);
  }
  String toWord(byte limit = 2) {
    get();
    limit %= 6;
    String result = "";
    byte count = 0;
    for (byte i = 0; i < 6; i++) {
      if (value[i] == 0) continue;
      result += Calendar.toString(value[i]) + Calendar.getIdentifier(i);
      count++;
      if (count >= limit) break;
      else result += " ";
    }
    return result;
  }
  String _toString(byte iMax = 6, String separator = " ", byte offset = 0) {
    get();
    String result = "";
    for (byte i = 0; i < iMax; i++) result += Calendar.toString(value[i + offset]) + (i != (iMax - 1) ? separator : "");
    return result;
  }
  String toString() {
    return _toString();
  }
  String toStringTime() {
    return _toString(3, ":", 3);
  }
  String toStringDate() {
    return _toString(3, "/");
  }

  // Conditionals
  bool equalTo(DateTime other) {
    get();
    bool result = true;
    for (byte i = 0; i < 6; i++) result = result && (value[i] == other.value[i]);
    return result;
  }
  bool greaterThanTo(DateTime other, bool includeEqual = false) {
    get();
    bool result = true;
    for (byte i = 0; i < 6; i++) result = result && (includeEqual ? (value[i] >= other.value[i]) : (value[i] > other.value[i]));
    return result;
  }
  bool lesserThanTo(DateTime other, bool includeEqual = false) {
    get();
    bool result = true;
    for (byte i = 0; i < 6; i++) result = result && (includeEqual ? (value[i] <= other.value[i]) : (value[i] < other.value[i]));
    return result;
  }
  bool isMin() {
    get();
    bool result = true;
    for (byte i = 0; i < 6; i++) result = result && (value[i] == 0);
    return result;
  }
  bool isMax() {
    get();
    bool result = true;
    for (byte i = 0; i < 6; i++) result = result && (value[i] == 255);
    return result;
  }
};

// DATE AND TIME OPERATIONS ----------------------------------------------------------------------------
struct {
  DateTime difference(DateTime start, DateTime end) {
    int value[6];
    const byte j = 6, lastIdx = j - 1;
    for (byte i = 0; i < j; i++) {
      byte idx = lastIdx - i;
      start.get();
      end.get();
      value[idx] = end.value[idx] - start.value[idx];
      if (value[idx] >= 0) continue;
      value[idx] = value[idx] + Calendar.getThreshold(idx, end.value[0], start.value[1]);
      if (i != lastIdx) end.set(idx - 1, end.value[idx - 1] - 1);
    }
    DateTime result = { value[0], value[1], value[2], value[3], value[4], value[5] };
    return result;
  }
  DateTime sum(DateTime base, DateTime add) {
    int value[6];
    const byte j = 6, lastIdx = j - 1;
    for (byte i = 0; i < j; i++) {
      byte idx = lastIdx - i;
      base.get();
      add.get();
      value[idx] = add.value[idx] + base.value[idx];
      if (idx == 1) {
        int daysInCurrentMonth = Calendar.daysInMonth(base.year, base.month);
        while (value[2] > daysInCurrentMonth) {
          value[2] -= daysInCurrentMonth;
          base.month += 1;
          if (base.month > 12) {
            base.month = 1;
            base.year += 1;
          }
          daysInCurrentMonth = Calendar.daysInMonth(base.year, base.month);
        }
      } else {
        if (value[idx] < 60) continue;
        value[idx] = value[idx] - Calendar.getThreshold(idx, add.value[0], base.value[1]);
        if (i != lastIdx) base.set(idx - 1, base.value[idx - 1] + 1);
      }
    }
    DateTime result = { value[0], value[1], value[2], value[3], value[4], value[5] };
    return result;
  }
} DateTimeOperations;

// RUNTIME ----------------------------------------------------------------------------
struct {
  uRTCLib rtc = uRTCLib(0x68);
  DateTime start, current, currentRTC, life;
  void init() {
    URTCLIB_WIRE.begin();
  }
  void update() {
    rtc.refresh();
    currentRTC.setRTC(rtc);
    if (current.equalTo(currentRTC)) return;
    current = currentRTC;
    life = DateTimeOperations.difference(current, start);
  }
  void set(byte* value) {
    start.setArray(value);
  }
  void setDT(DateTime value) {
    start = value;
  }
} Runtime;

// TIMERS ----------------------------------------------------------------------------
struct TimerMS {
  unsigned long increment;
  unsigned long prevTime = 0;
  TimerMS(unsigned long i = 0, bool _update = false)
    : increment(i) {
    if (_update) update();
  }
  void update() {
    prevTime = millis();
  }
  bool isDone(bool autoUpdate = true) {
    bool result = prevTime + increment <= millis();
    if (autoUpdate && result) update();
    return result;
  }
  unsigned long getEndTime() {
    return prevTime + increment;
  }
  unsigned long getPassedTime() {
    return millis() - prevTime;
  }
  unsigned long getRemainingTime() {
    return (prevTime + increment) - millis();
  }
};

struct TimerDT {
  DateTime prevTime, increment;
  TimerDT(byte* _increment, bool _update = false) {
    increment.setArray(_increment);
    if (_update) update();
  }
  TimerDT(DateTime _increment, bool _update = false)
    : increment(_increment) {
    if (_update) update();
  }
  void update() {
    prevTime = Runtime.current;
  }
  bool isDone(bool autoUpdate = true) {
    DateTime time = DateTimeOperations.difference(Runtime.current, prevTime);
    bool result = time.greaterThanTo(increment, true);
    if (autoUpdate && result) update();
    return result;
  }
  DateTime getEndTime() {
    DateTime time = DateTimeOperations.sum(prevTime, increment);
    return time;
  }
  DateTime getPassedTime() {
    DateTime time = DateTimeOperations.difference(Runtime.current, prevTime);
    return time;
  }
  DateTime getRemainingTime() {
    DateTime time = DateTimeOperations.sum(prevTime, increment);
    time = DateTimeOperations.difference(time, Runtime.current);
    return time;
  }
};

// SYSTEM ----------------------------------------------------------------------------
const int baudRate = 9600;
const byte defaultTime[6] = { 0, 0, 0, 0, 0, 0 };
const String blankLCD = "                    ";

const int tickRate = 10;
TimerMS tick(tickRate);
TimerMS timerPrinter(200);
TimerMS DisplayTimer(1000);

const byte intervalPreheat[6] = { 0, 0, 0, 0, 2, 0 };
const byte intervalSIMLog[6] = { 0, 0, 0, 1, 0, 0 };
const byte intervalSDLog[6] = { 0, 0, 0, 1, 0, 0 };
TimerDT timerPreheat(intervalPreheat);
TimerDT timerSIMLog(intervalSIMLog);
TimerDT timerSDLog(intervalSDLog);

bool changingToStorage = false, changingToIdle = false;
int storageProcess = 0;

byte state = 0;
const byte countTimeCurrent = 4;
DateTime timeCurrent[countTimeCurrent];  // INIT/PREHEAT, STORAGE, SIM LOG, SD LOG

// SERIAL ----------------------------------------------------------------------------
void initSerial() {
  Serial.begin(baudRate);
  Serial.println("\n\n");
  Serial.println("Initialization Started =========================================");
}

// ================================ COMPONENTS ================================
// VOLTAGE SENSOR -------------------------------------------------------------------------------
struct VoltageSensorDIY {
  byte pin;
  float r1, r2;
  VoltageSensorDIY(byte _pin, float _r1, float _r2)
    : pin(_pin) {
    pinMode(_pin, INPUT);
    r1 = _r1;
    r2 = _r2;
  }
  int readRaw() {
    return analogRead(pin);
  }
  float readVolts() {
    return 5.0 * (analogRead(pin) / 1023.0);
  }
  float read() {
    return readVolts() * ((r1 + r2) / r2);
  }
  float readPercent() {
    return (read() / 14.5) * 100.0;
  }
};

struct {
  VoltageSensorDIY sense = VoltageSensorDIY(pinVoltSense, 3000.0, 1000.0);
  const float limits[2] = { 2.0, 9.0 };
  TimerMS timer = TimerMS(5000);
  bool state = false;

  static const int elements = 10;
  float valuesPrevious[elements], valuesCurrent[elements];
  int index = 0;

  float voltage, percent;
  void init() {
    voltage = sense.read();
    getPercent();
  }
  void setValue(float value) {
    valuesPrevious[index] = valuesCurrent[index];
    valuesCurrent[index] = value;
    index++;
    if (index == elements) {
      index = 0;
      state = false;
    }
  }
  void update() {
    float currentVoltage = sense.read();
    if (currentVoltage != 0 && voltage == 0) {
      voltage = currentVoltage;
      getPercent();
    }
    if (currentVoltage == 0 && voltage != 0) {
      voltage = 0;
      percent = 0;
    }

    if (timer.isDone()) state = true;
    if (!state || currentVoltage == 0) return;
    setValue(currentVoltage);

    if (state) return;
    getVoltage();
    getPercent();
  }
  void getPercent() {
    percent = ((voltage - limits[0]) / (limits[1] - limits[0])) * 100.0;
    percent = min(100.0, max(percent, 0.0));
  }
  void getVoltage() {
    float sum = 0;
    for (int i = 0; i < elements; i++) sum += valuesCurrent[i];
    voltage = sum / float(elements);
  }
} BatteryVoltage;

// LCD -------------------------------------------------------------------------------
struct {
  int code = 0;
  const String blankLCD = "                    ";
  LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4);
  void write(String text, int row = 0, bool center = true) {
    lcd.setCursor(0, row);
    text = (center) ? StringManipulator.centerText(text) : StringManipulator.leftText(text);
    lcd.print(text);
  }
  void print(String* texts, bool center = true) {
    for (byte i = 0; i < 4; i++) write(texts[i].equals("") ? blankLCD : texts[i], i, center);
  }
  void print(String text, int row = 0, bool center = true) {
    for (byte i = 0; i < 4; i++) write(row == i ? text : blankLCD, i, center);
  }
  void init() {
    lcd.init();
    lcd.backlight();
    lcd.clear();
    delay(1000);
    String messages[4] = { "",
                           "   Initialization   ",
                           "=       05%         ",
                           "        LCD         " };
    Serial.println("05% ------------------------------ LCD");
    print(messages);
  }
  void setCondition(int dedicatedCode, bool condition) {
    if (condition && dedicatedCode > code) code = dedicatedCode;
  }
} LCD;

// SIM ----------------------------------------------------------------------------
struct DataSIM {
  String cell = "";
  String message = "";
  int priority = 0;
  DataSIM(String _cell = "", String _message = "", int _priority = 1) {
    set(_cell, _message, _priority);
  }
  void set(DataSIM value) {
    cell = value.cell;
    message = value.message;
    priority = value.priority;
  }
  void set(String _cell, String _message, int _priority = 1) {
    cell = _cell;
    message = _message;
    priority = _priority;
  }
  void clear() {
    cell = "";
    message = "";
    priority = 0;
  }
  bool isEmpty() {
    return cell == "" || priority == 0;
  }
};

struct QueueDataSIM {
  const static byte length = 10;
  DataSIM queue[length];
  byte selected = 0, lastPriority = 0;
  void clear() {
    for (byte i = 0; i < length; i++) queue[i].clear();
  }
  bool add(DataSIM value) {
    bool result = false;
    if (value.isEmpty()) return result;
    for (byte i = 0; i < length; i++) {
      if (!queue[i].isEmpty()) continue;
      queue[i].set(value);
      result = true;
      break;
    }
    return result;
  }
  bool remove(byte i) {
    if (i >= length || queue[i].isEmpty()) return false;
    queue[i].clear();
    return true;
  }
  DataSIM get() {
    // Updating current selected value to the next taken element
    byte newSelected = selected;
    if (queue[selected].isEmpty()) {
      newSelected = length;
      for (byte i = 0; i < length; i++) {
        byte current = (lastPriority + i) % length;
        if (queue[current].isEmpty()) continue;
        newSelected = current;
        break;
      }
      // Array is empty
      if (newSelected == length) {
        DataSIM tempValue;
        return tempValue;
      }
    }
    selected = newSelected;
    byte priority = newSelected;
    for (byte i = 0; i < length; i++) {
      byte current = (lastPriority + i) % length;
      priority = (queue[current].priority > queue[priority].priority ? current : priority);
    }
    if (queue[priority].priority > 1) lastPriority = priority;
    DataSIM tempValue = queue[priority];
    queue[priority].clear();
    return tempValue;
  }
};

struct {
  QueueDataSIM queue;
  TimerMS timer = TimerMS(5000);
  void init() {
    Serial1.begin(9600);
    LCD.write("        SIM         ", 3);
    resetSIM();
    LCD.write("======  30%         ", 2);
    Serial.println("30% ------------------------------ SIM");
  }
  void clear() {
    for (byte i = 0; i < 3; i++) Serial1.write(27);
    Serial1.print("\r\n");
  }
  void resetSIM() {
    Serial.println("\n\n");
    Serial.println("SIM MODULE INITIALIZATION START ------------------");
    String gsminitat;
    TimerMS simTimeout = TimerMS(2000, true);
    while (true) {
      Serial1.println("AT");
      gsminitat = Serial1.readString();
      delay(500);
      Serial.println(gsminitat);
      String serialPrint;
      Serial.println();
      if (gsminitat.indexOf("OK") > -1) {
        serialPrint = "SIM HAS SUCCESSFULLY INITIALIZED";
        LCD.write(blankLCD);
        break;
      } else if (simTimeout.isDone(false)) {
        serialPrint = "SIM HAS FAILED TO INITIALIZED";
        LCD.write("ERROR PLEASE RESTART");
      }
      Serial.println(serialPrint);
    }
    Serial1.println("AT+CNMI=1,2,0,0,0");
    String gsmRep = Serial1.readString();
    Serial.println(gsmRep);
    Serial1.println("AT+CMGF = 1");
    gsmRep = Serial1.readString();
    Serial.println(gsmRep);
    clear();
    Serial.println("SIM MODULE INITIALIZATION END ------------------");
    Serial.println("\n\n");
  }
  void send(String cell, String message, int priority = 1) {
    queue.add(DataSIM(cell, message, priority));
  }
  void send(String cell, int length, String* messages, int priority = 1) {
    queue.add(DataSIM(cell, StringManipulator.arrayToString(length, messages), priority));
  }
  void notify(String cell, String message) {
    Serial.println("Notifying Start------------------------------------------");
    Serial.println("Cell:\t" + cell + "\nMessage----------\n" + message + "\n-----------------");

    for (byte i = 0; i < 3; i++) Serial1.write(27);
    Serial1.print("\r\n");
    Serial1.println("AT+CMGS=\"" + cell + "\"");
    Serial1.readString();
    Serial1.print(message);
    Serial1.write(0x1A);
    Serial.println("Notifying End--------------------------------------------");
  }
  void update() {
    if (!timer.isDone()) return;
    DataSIM chosen = queue.get();
    if (chosen.isEmpty()) return;
    notify(chosen.cell, chosen.message);
  }
} SIM;

// FANS ------------------------------------------------------------------------------
struct Fan {
  byte pin = 255;
  byte pwm = 0;
  TimerMS timer = TimerMS(3000);
  void init(byte _pin) {
    pin = _pin;
    pinMode(_pin, OUTPUT);
    pwm = rangePWM[1];
    analogWrite(_pin, rangePWM[1]);
    timer.update();
  }
  void power(byte _pwm = 0) {
    if (pwm == _pwm || !timer.isDone(false)) return;
    pwm = _pwm;
    analogWrite(pin, _pwm);
  }
  void minimize() {
    power(rangePWM[0]);
  }
  void maximize() {
    power(rangePWM[0]);
  }
};
Fan fans[5];

void initFAN() {
  LCD.write("        FAN         ", 3);
  for (byte i = 0; i < countFAN; i++) fans[i].init(pinsFAN[i]);
  delay(5000);
  LCD.write("========55%         ", 2);
  Serial.println("55% ------------------------------ FAN");
}

// DHT -------------------------------------------------------------------------------
struct dhtData {
  DHT component;
  float temperature = NULL, humidity = NULL;
  dhtData(byte pin = 255)
    : component(pin, DHT22) {
    if (pin == 255) return;
    component.begin();
  }
  bool read() {
    temperature = component.readTemperature();
    humidity = component.readHumidity();
  }
};

struct {
  float temperature, humidity, ambientTemperature, ambientHumidity;
  dhtData stack[dhtCount];
  TimerMS timer = TimerMS(750);
  void init() {
    LCD.write("        DHT         ", 3);
    for (byte i = 0; i < dhtCount; i++) stack[i] = dhtData(dhtPins[i]);
    LCD.write(blankLCD);
    LCD.write("========70%===      ", 2);
    Serial.println("70% ------------------------------ DHT");
  }
  void update() {
    if (!timer.isDone()) return;
    temperature = 0, humidity = 0;
    byte countTemp = 0, countHum = 0;
    for (byte i = 0; i < dhtCount - 1; i++) {
      stack[i].read();
      if (!isNaN(stack[i].temperature)) {
        temperature += stack[i].temperature;
        countTemp++;
      }
      if (!isNaN(stack[i].humidity)) {
        humidity += stack[i].humidity;
        countHum++;
      }
    }
    if (countTemp > 0) temperature /= countTemp;
    if (countHum > 0) humidity /= countHum;

    stack[dhtCount - 1].read();
    ambientTemperature = stack[dhtCount - 1].temperature;
    ambientHumidity = stack[dhtCount - 1].humidity;
  }
} dhtStack;

// MQ135 --------------------------
struct {
  MQ135 mq = MQ135(mqPin);
  TimerMS timer = TimerMS(750);
  float value = 0;
  bool exceed = false, lastExceed = false;
  String toString() {
    return StringManipulator.toStringFloat(value);
  }
  void update() {
    if (!timer.isDone()) return;
    float rzero = mq.getRZero();
    float correctedRZero = mq.getCorrectedRZero(dhtStack.temperature, dhtStack.humidity);
    float resistance = mq.getResistance();
    float ppm = mq.getPPM();
    float correctedPPM = mq.getCorrectedPPM(dhtStack.temperature, dhtStack.humidity) * 10.0;
    value = correctedPPM;

    exceed = value >= thresholdPPM;
    if (lastExceed != exceed) return;
    lastExceed = exceed;
    if (!exceed) return;
    notify();
  }
  void notify() {
    Serial.println("CO2 LEVEL WARNING!");
    String message = "ALERT!! Please check the onion storage!";
    SIM.send(cellBeneficiary, message);
    for (byte i = 0; i < devs; i++) SIM.send(cellDevelopers[i], message);
  }
} MQ;


// BUTTON ----------------------------------------------------------------------------
struct Button {
  byte pin;
  bool defaultValue = false;
  TimerMS timer;
  bool value;
  const int triggerThreshold = 500;
  byte state = 0;  // 1: trigger, 2: hold

  Button(byte _pin, int thresholdhold = 0, bool _defaultValue = false) {
    pin = _pin;
    pinMode(_pin, INPUT_PULLUP);
    defaultValue = _defaultValue;

    timer = TimerMS(thresholdhold);
    timer.update();
  }
  void update() {
    bool newValue = digitalRead(pin);
    if (value != newValue) {
      if (newValue == defaultValue) {
        if (timer.getPassedTime() <= triggerThreshold) state = 1;
        else if (timer.isDone(false)) state = 2;
      } else {
        timer.update();
      }
    }
    value = newValue;
  }
  bool onTrigger() {
    if (state != 1) return false;
    state = 0;
    return true;
  }
  bool onHold() {
    if (state != 2) return false;
    state = 0;
    return true;
  }
  bool holding() {
    return timer.getPassedTime() > triggerThreshold && value != defaultValue;
  }
  bool isDone() {
    return timer.isDone(false);
  }
  bool getValue() {
    return value != defaultValue;
  }
} btnGreen(btnPins[0], 3000, true), btnRed(btnPins[1], 5000, true);

// SD ----------------------------------------------------------------------------
struct {
  bool enabled = false;
  void init() {
    LCD.write("         SD         ", 3);
    enabled = !!SD.begin();
    Serial.println("SD Initialization: " + String(enabled ? ("Successful") : "Failed"));
    do {
      LCD.write("ERROR PLEASE RESTART");
      enabled = !!SD.begin();
    } while (!enabled);
    LCD.write(blankLCD);
    LCD.write("========100%========", 2);
    Serial.println("100% -----------------------------  SD");
    createFile("configs");
  }
  String read(String name, String target = "") {
    if (!enabled) return;
    String filename = target + "/" + name + ".txt";
    if (!SD.exists(filename)) return "";
    String data = "";
    File file = SD.open(filename, FILE_READ);
    if (file && file.available()) data = file.readString();
    file.close();
    Serial.println("Reading file: " + String(file.name()));
    Serial.println(data);
    Serial.println("Closing file: " + String(file.name()));
    return data;
  }
  void write(String name, String data, String target = "") {
    if (!enabled) return;
    String filename = target + "/" + name + ".txt";
    File file = SD.open(filename, FILE_WRITE);
    Serial.println("Writing file: " + String(file.name()));
    file.println(data);
    Serial.println(data);
    file.close();
    Serial.println("Closing file: " + String(file.name()));
  }
  void write(String name, int count, String* data, String target = "", bool addSpace = false) {
    if (!enabled) return;
    String filename = target + "/" + name + ".txt";
    File file = SD.open(filename, FILE_WRITE);
    Serial.println("Writing file: " + String(file.name()));
    for (int i = 0; i < count; i++) {
      file.println(data[i]);
      Serial.println(data[i]);
      if (i == count - 1 && addSpace) {
        file.println("\n\n");
        Serial.print("\n\n");
      }
    }
    file.close();
    Serial.println("Closing file: " + String(file.name()));
  }
  void overwrite(String name, String data, String target = "") {
    if (!enabled) return;
    String filename = target + "/" + name + ".txt";
    SD.remove(filename);
    File file = SD.open(filename, FILE_WRITE);
    Serial.println("Writing file: " + String(file.name()));
    file.println(data);
    Serial.println(data);
    file.close();
    Serial.println("Closing file: " + String(file.name()));
  }
  void overwrite(String name, int count, String* data, String target = "") {
    if (!enabled) return;
    String filename = target + "/" + name + ".txt";
    SD.remove(filename);
    File file = SD.open(filename, FILE_WRITE);
    Serial.println("Overwriting file: " + String(file.name()));
    for (int i = 0; i < count; i++) {
      file.println(data[i]);
      Serial.println(data[i]);
    }
    file.close();
    Serial.println("Closing file: " + String(file.name()));
  }
  void clean(String dirPath = "") {
    if (!enabled) return;
    Serial.println("Deleting files in " + dirPath);
    File dir = SD.open(dirPath);
    while (true) {
      File entry = dir.openNextFile();
      if (!entry) break;
      if (entry.isDirectory()) {
        clean(entry.name());
        continue;
      }
      Serial.println(entry.name());
      entry.close();
      SD.remove(entry.name());
    }
    dir.close();
    Serial.println("Deleted files in " + dirPath);
  }
  int countDir(String dirPath = "") {
    if (!enabled) return 0;
    File dir = SD.open(dirPath);
    int count = 0;
    while (true) {
      File entry = dir.openNextFile();
      if (!entry) break;
      if (!entry.isDirectory() || String(entry.name()).indexOf("SYSTEM") != -1) continue;
      count++;
    }
    dir.close();
    Serial.println(dirPath == "" ? "root" : dirPath + " has " + String(count) + " directories.");
    return count;
  }
  int countFile(String dirPath = "") {
    if (!enabled) return 0;
    File dir = SD.open(dirPath);
    int count = 0;
    while (true) {
      File entry = dir.openNextFile();
      if (!entry) break;
      if (entry.isDirectory()) continue;
      count++;
    }
    dir.close();
    Serial.println(dirPath == "" ? "root" : dirPath + " has " + String(count) + " files.");
    return count;
  }
  int countElements(String dirPath = "") {
    if (!enabled) return 0;
    File dir = SD.open(dirPath);
    int count = 0;
    while (true) {
      File entry = dir.openNextFile();
      count++;
    }
    dir.close();
    Serial.println(dirPath == "" ? "root" : dirPath + " has " + String(count) + " elements.");
    return count;
  }
  void createFile(String name, String target = "") {
    if (!enabled) return;
    String filename = target + "/" + name + ".txt";
    File file = SD.open(filename, FILE_WRITE);
    file.close();
    Serial.println(filename + " has been created.");
  }
  void removeFile(String name, String target = "") {
    if (!enabled) return;
    String filename = target + "/" + name + ".txt";
    SD.remove(filename);
    Serial.println(filename + " has been removed.");
  }
  void resetFile(String name, String target = "") {
    if (!enabled) return;
    removeFile("configs");
    createFile("configs");
  }
  void createDir(String name, String target = "") {
    if (!enabled) return;
    String filename = target + "/" + name;
    SD.mkdir(filename);
    Serial.println(filename + " has been created.");
  }
  int countRow(String data) {
    if (!enabled || data.length() == 0) return 0;
    int count = 1;
    for (int i = 0; i < data.length(); i++) {
      if (data[i] != '\n') continue;
      count++;
    }
    return count;
  }
  bool isFileEmpty() {
    if (!enabled) return true;
    File file = SD.open("configs.txt", FILE_READ);
    bool result = file.available() != 0;
    file.close();
    return result;
  }
} uSD;


// ================================ PROCESS ================================
struct {
  int storageProcess = 0;
  String getDir() {
    return "storage" + String(storageProcess);
  }
  void updateConfig() {
    String data[countTimeCurrent + 1] = { String(state), timeCurrent[0].toString(), timeCurrent[1].toString(), timeCurrent[2].toString(), timeCurrent[3].toString() };
    uSD.overwrite("configs", countTimeCurrent + 1, data);
  }
  void getDataString(String* result) {
    result[0] = Runtime.current.toStringDate() + " " + Runtime.current.toStringTime() + ":     " + Runtime.life.toWord();
    result[1] = "T: " + StringManipulator.toStringFloat(dhtStack.temperature) + "C";
    result[2] = "H: " + StringManipulator.toStringFloat(dhtStack.humidity) + "%";
    result[3] = "RTCT: " + StringManipulator.toStringFloat(Runtime.rtc.temp() / 100.00) + "C";
    result[4] = "CO2: " + MQ.toString() + "ppm";
    result[5] = "AT: " + StringManipulator.toStringFloat(dhtStack.ambientTemperature) + "C";
    result[6] = "AH: " + StringManipulator.toStringFloat(dhtStack.ambientHumidity) + "%";
  }
  void update() {
    if (state != 2) return;
    byte count = 7;
    String data[count];
    getDataString(data);

    bool simUpdate = updateSIM(data, count);
    bool sdUpdate = updateSD(data, count);

    if (!simUpdate && !sdUpdate) return;
    updateConfig();
  }
  bool updateSIM(String* data, byte count) {
    if (!timerSIMLog.isDone()) return false;
    timeCurrent[2].setArray(Runtime.current.value);
    for (byte i = 0; i < devs; i++) SIM.send(cellDevelopers[i], count, data);
    return true;
  }
  bool updateSD(String* data, byte count) {
    if (!timerSDLog.isDone()) return false;
    timeCurrent[3].setArray(Runtime.current.value);
    String title = Runtime.current.toStringDate();
    uSD.write(title, count, data, getDir(), true);
    return true;
  }
  void initialize() {
    String messages[4] = { "", "   Preparation   ", "====================", "" };
    LCD.print(messages);
    prepare();
  }
  void prepare() {
    LCD.code = 0;
    state = 0;

    timeCurrent[0].setRTC(Runtime.rtc);
    for (byte i = 1; i < countTimeCurrent; i++) {
      timeCurrent[i].setArray(defaultTime);
    }

    uSD.resetFile("configs");
  }
} DataSystem;

struct {
  void check() {
    DataSystem.storageProcess = uSD.countDir();
    Serial.println("Current Process: " + String(DataSystem.storageProcess));

    bool runningProcess = uSD.isFileEmpty();

    // if (runningProcess) recover();
    // else DataSystem.initialize();
    String newTitle = DataSystem.getDir();
    String message = (runningProcess ? "Restoring Crash: " : "Preparing: ");
    String messages[4] = { "", message, newTitle, "" };
    LCD.print(messages);
    SIM.notify(cellDevelopers[0], message + newTitle);
    delay(3000);

    if (runningProcess) recover();
    else DataSystem.initialize();
    Serial.println("Initialization Done =============================================");
  }
  void recover() {
    Serial.println("Restoring Crash ===========================================");

    String data = uSD.read("configs");
    Runtime.rtc.refresh();
    Runtime.currentRTC.setRTC(Runtime.rtc);
    String resolveDT = Runtime.currentRTC.toStringDate() + "\t" + Runtime.currentRTC.toStringTime();
    uSD.write("resolves", resolveDT, DataSystem.getDir());
    // String data = "1\n24 05 09 02 14 09\n00 00 00 00 00 00";
    int rows = uSD.countRow(data);
    String rowString[rows];
    int idxStart = 0, idxEnd = 0, i = 0;
    while (idxEnd != -1) {
      idxEnd = data.indexOf('\n', idxStart);
      if (idxEnd == -1) {
        rowString[i] = data.substring(idxStart, data.length());
        Serial.println(rowString[i]);
      } else {
        rowString[i] = data.substring(idxStart, idxEnd);
        Serial.println(rowString[i]);
        idxStart = idxEnd + 1;
      }

      switch (i) {
        case 0:
          state = rowString[i].toInt();
          break;
        default:
          timeCurrent[i - 1].fromString(rowString[i]);
          break;
      }
      i++;
    }

    Runtime.start.setArray(timeCurrent[state >= 2].value);
    timerSIMLog.prevTime = timeCurrent[2];
    timerSDLog.prevTime = timeCurrent[3];
    Serial.println("Crash Restored =============================================");
  }
} RecoverySystem;

// UPDATES ----------------------------------------------------------------------------
void buttonUpdate() {
  btnGreen.update();
  btnRed.update();
  bool greenTrigger = btnGreen.onTrigger();
  bool greenHold = btnGreen.onHold();
  bool redTrigger = btnRed.onTrigger();
  bool redHold = btnRed.onHold();
  LCD.code = LCD.code == 399 ? 399 : 0;

  if (LCD.code == 0 && greenTrigger) LCD.setCondition(399, greenTrigger);
  else if (LCD.code == 399 && (redTrigger || greenTrigger)) LCD.code = 0;
  // Both Button is triggered
  LCD.setCondition(404, btnRed.getValue() && btnGreen.getValue() || btnRed.holding() && btnGreen.holding());
  if (state == 0) {
    // Green Button is triggered
    LCD.setCondition(400, btnGreen.holding() && !btnGreen.isDone());
    LCD.setCondition(401, btnGreen.holding() && btnGreen.isDone());
  } else {
    // Red Button is triggered
    LCD.setCondition(402, btnRed.holding() && !btnRed.isDone());
    LCD.setCondition(403, btnRed.holding() && btnRed.isDone());
  }

  if (LCD.code == 403 && !changingToIdle) changingToIdle = true;
  if (changingToIdle && LCD.code != 403) terminateStorage();

  if (LCD.code == 401 && !changingToStorage) changingToStorage = true;
  if (changingToStorage && LCD.code != 401) prepareStorage();

  if (state == 1 && timerPreheat.isDone(false)) executeStorage();
}

void sensorUpdate() {
  if (state != 2) return;
  dhtStack.update();
  MQ.update();
}

TimerMS timerFAN(3000);
void actuatorUpdate() {
  if (!timerFAN.isDone(false)) return;
  if (state != 2)
    for (byte i = 0; i < countFAN - 2; i++) fans[i].minimize();

  if (dhtStack.temperature < rangeTemp[0]) {
    fans[0].minimize();
    fans[2].minimize();
  } else if (dhtStack.temperature < rangeTemp[1]) {
    float value = dhtStack.temperature - rangeTemp[0];
    float max = rangeTemp[1] - rangeTemp[0];
    float percent = value / max;
    int maxPwm = rangePWM[1] - rangePWM[0];
    int pwm = rangePWM[0] + percent * maxPwm;
    for (byte i = 0; i < countFAN - 2; i++) fans[i].power(pwm);
  } else {
    fans[0].maximize();
    fans[2].maximize();
  }
}

void displayUpdate() {
  String messages[4] = { "", "", "", "" };
  switch (LCD.code) {
    case 399:
      messages[0] = "  Battery  Voltage  ";
      messages[1] = StringManipulator.toStringFloat(BatteryVoltage.voltage) + "V";
      messages[2] = StringManipulator.toString(BatteryVoltage.percent) + "%";
      messages[3] = "Draining";
      break;
    case 404:
      messages[0] = "  Do not hold both  ";
      messages[1] = " hold  both buttons ";
      messages[2] = "  at the same time  ";
      break;
    case 403:
      messages[1] = "Release RED button  ";
      messages[2] = "to start process!   ";
      break;
    case 402:
      messages[0] = "Hold RED button     ";
      messages[1] = "for " + String(max(btnRed.timer.getRemainingTime() + 1000, 0) / 1000) + " seconds to    ";
      messages[2] = "terminate process!  ";
      break;
    case 401:
      messages[0] = "Release GREEN button";
      messages[1] = "to start process!   ";
      break;
    case 400:
      messages[0] = "Hold GREEN button   ";
      messages[1] = "for " + String(max(btnGreen.timer.getRemainingTime() + 1000, 0) / 1000) + " seconds to    ";
      messages[2] = "start preheating.   ";
      break;
    case 0:
      messages[0] = Runtime.current.toStringDate() + "    " + Runtime.current.toStringTime();
      if (state == 0) {
        messages[1] = "  Hold Green Button ";
        messages[2] = "  to start storage  ";
      } else if (state == 1) {
        DateTime preheatDT = timerPreheat.getEndTime();

        messages[1] = "  Storage  process  ";
        messages[2] = "   will start in    ";
        messages[3] = preheatDT.toStringDate() + "    " + preheatDT.toStringTime();
      } else {
        messages[1] = StringManipulator.leftText("Runtime: " + Runtime.life.toWord());
        messages[2] = "T: " + StringManipulator.toStringFloat(dhtStack.temperature) + "C  H: " + StringManipulator.toStringFloat(dhtStack.humidity) + "%";
        messages[3] = "AT:" + StringManipulator.toStringFloat(dhtStack.ambientTemperature) + "C  AH:" + StringManipulator.toStringFloat(dhtStack.ambientHumidity) + "%";
      }
      break;
    default:
      break;
  }
  LCD.print(messages);
}

// INSTANTANEOUS ----------------------------------------------------------------------------
void terminateStorage() {
  SIM.queue.clear();

  String messageToSend = "Storage Terminated: " + DataSystem.getDir();
  for (byte i = 0; i < devs; i++) SIM.send(cellDevelopers[i], messageToSend);

  changingToIdle = false;
  Serial.println("Storage Termination");
  LCD.print("Storage  Termination", 1);

  DataSystem.prepare();
}

void prepareStorage() {
  changingToStorage = false;
  if (!timerPreheat.isDone(false)) executePreheat();
  else executeStorage();
  LCD.code = 0;
}

void executePreheat() {
  Serial.println("Preheating");
  LCD.print("Preheating", 1);
  state = 1;
  DataSystem.updateConfig();
}

void executeStorage() {
  String messageToSend = "Storage Started: " + DataSystem.getDir();
  for (byte i = 0; i < devs; i++) SIM.send(cellDevelopers[i], messageToSend);

  DataSystem.storageProcess++;
  uSD.createDir(DataSystem.getDir());

  Runtime.start.setArray(Runtime.current.value);
  timeCurrent[1].setArray(Runtime.start.value);

  Serial.println("Preparing Storage");
  LCD.print("Preparing Storage");
  state = 2;
  for (byte i = 0; i < countFAN; i++) fans[i].maximize();
  DataSystem.updateConfig();
}

void forceSender() {
  if (Serial.available() <= 0) return;  // checks for any data  coming through serial port of arduino.
  switch (Serial.read()) {
    case 'n':
      MQ.notify();
      break;
    case 's':
      byte count = 7;
      String data[count];
      DataSystem.getDataString(data);
      for (byte i = 0; i < devs; i++) SIM.send(cellDevelopers[i], count, data);
      break;
    case 'f':
      uSD.clean();
      break;
  }
}


// ================================ MAIN ================================
void setup() {
  initSerial();
  Runtime.init();
  BatteryVoltage.init();
  LCD.init();
  SIM.init();
  initFAN();
  dhtStack.init();
  uSD.init();
  Serial.println("Initialization Ended ===========================================");
  RecoverySystem.check();
  Serial.println("=========================================");
}

void loop() {
  if (!tick.isDone()) return;
  forceSender();
  Runtime.update();
  BatteryVoltage.update();
  buttonUpdate();
  sensorUpdate();
  actuatorUpdate();
  DataSystem.update();
  SIM.update();
  displayUpdate();
}
