/*
  Управление двигателями для системы электронного дифференциала. Разработал AnLi.
  Версия  0.8.0. Версия оптимизирована под микропроцессор Atmega 328p. Ядро - GyverCore (https://alexgyver.ru/gyvercore/)
*/

//-----------------Библиотеки---------//
#include <avr/eeprom.h> // Для работы с EEPROM памятью
#include "MPU6050.h" //Библиотека для датчика MPU6050
#include <Servo.h> //Библиотека управления сервоприводами 
#include <Fuzzy.h> // Библиотека нечёткого регулятора (eFLL)


//---------------Постоянные----------//
#define WorkMode 0 // Режим работы, 0 - основной режим работы, 1 - сбор данных, 2 - отправка данных, 3 - дифференциал Анкермана-Джекмана, 4 - тест исполнительных элементов
#define ServoMotor  5 //Пин сервопривода рулевой рейки
Servo SteerServo; //Сервопривод передних колёс
MPU6050 mpu; // Гироскоп и акселерометр

Fuzzy *fuzzy = new Fuzzy(); // Созданим объект fuzzy
int speedmax = 100; // Максимальное значение отклонения скорости (для нечёткого регулятора)

#define MinServoAngle  20 // Минимальный угол поворота серводвигателя
#define MaxServoAngle  160 // Максимальный угол поворота серводвигателя

#define EncoderLeft 3 //Первый оптический датчик скорости
#define Encoder1Right 2 //Второй оптический датчик скорости

#define LeftMotorForward  6 //Левый двигатель, движение вперёд
#define LeftMotorBack  9 // Левый двигатель, движение назад
#define RightMotorForward  10 //Правый двигатель, движение вперёд
#define RightMotorBack  11 //Правый двигатель, движение вперёд

#define GreenLED  A0 // Зелёный светодиод индикации
#define RedLED  A1 // Красный светодиод индикации
#define BlinkPeriod 200 //Период для мерцания светодиодов, мс

#define CoilPin  A3 //Пин управления соленоидом

#define cleanFlag false //Флаг сброса EEPROM-памяти при старте
#define fuzzyFlag false //Флаг для включения инициализации работы нечёткого регулятора

//---Постоянные автомобиля---
#define L 0.19 //Расстояние между передним и задним колесом (м)
#define dr 0.13 // Расстояние между задними колесами (м)
#define lr 0.11 // Расстояние между задним колесом и центром тяжести (м)
#define r 0.032 // Радиус колеса (м)
#define K 0.014 // Расстояние между правым и левым kingpin (м)

//------------------Переменные--------------//
uint32_t myTimer1; // Переменная хранения времени 1
uint32_t myTimer2; // Переменная хранения времени 2

//Для энкодеров
volatile uint32_t LeftMotorTurnCounts; //Количество тиков с левого энкодера
volatile uint32_t RightMotorTurnCounts; //Количество тиков с правого энкодера
bool StartRecord = false; //Перемнная разрешения записи

//Для электронного дифференциала
float Omega1; //Скорость вращения 1, реальная скорость вращения (не ШИМ)
float Omega2; //Скорость вращения 2, реальная скорость вращения (не ШИМ)
float LinearIdleSpeed = 0.25; //Скорость машинки, линейная (м/c) по дефолту указана максимальная

char data; //Переменная для хранения получаемого символа

int MotorSpeed; // Общая скорость вращения двигателей
int LeftMotorSpeed; // Скорость вращения левого двигателя (ШИМ-значение)
int RightMotorSpeed; // Скорость вращения правого двигателя (ШИМ-значение)
int ServoAngle; // Угол поворота серводвигателя

//Значения с датчика ускорения и гироскопа
int16_t ax, ay, az; // Ускорение от от -32768 до 32768 (сырые данные)
int16_t gx, gy, gz; // Угловая скорость от -32768 до 32768 (сырые данные)
int16_t temp; // Температура

//--------------------------------------------------------------//
//-----------------------Инициализация--------------------------//
//--------------------------------------------------------------//
void setup()
{
  Serial.begin(19200);
  Wire.begin();
  //Serial.setTimeout(1);

  //Двигатели
  pinMode(ServoMotor, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBack, OUTPUT);
  pinMode(RightMotorForward, OUTPUT);
  pinMode(RightMotorBack, OUTPUT);
  //Переферия
  pinMode(GreenLED, OUTPUT);
  pinMode(RedLED, OUTPUT);
  pinMode(CoilPin, OUTPUT);

  // Расгоняем ШИМ на пинах D9 и D10 до 31.4 кГц, разрядность 8 бит
  //TCCR1A = 0b00000001;  // 8bit
  //TCCR1B = 0b00000001;  // x1 phase correct

  //Выключаем всё
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(LeftMotorBack, LOW);
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(RightMotorBack, LOW);
  digitalWrite(CoilPin, LOW);
  digitalWrite(RedLED, LOW);
  digitalWrite(GreenLED, LOW);

  //Подключаем прерывания
  attachInterrupt(digitalPinToInterrupt(EncoderLeft), EncoderLeftTick, RISING); //Подключаем прерывание для левого энкодера
  attachInterrupt(digitalPinToInterrupt(Encoder1Right), EncoderRightTick, RISING); //Подключаем прерывание для правого энкодера

  SteerServo.attach(ServoMotor); //Подключаем объект сервопривода
  SteerServo.write(int((MinServoAngle + MaxServoAngle) / 2)); // Задаем начальное положение приводу (по середине)

  mpu.initialize();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); //  – диапазон -2.. 2 g
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250); // – диапазон -250.. 250 град/с
  // состояние соединения

  //Если требуется очистка EEPROM-памяти
  if (cleanFlag) {
    MemoryClean ();
  }

  //Если нужна инициализация нечёткого регулятора
  if (fuzzyFlag) {
    FuzzyLogicSetup ();
  }
}

//--------------------------------------------------------------//
//--------------------------Основной цикл-----------------------//
//--------------------------------------------------------------//
void loop()
{
  //Включение текущего режима
  switch (WorkMode) {
    case 0:
      BluetoothControl(); //Управление машиной при помощи нейросети через Bluetooth
      break;
    case 1:
      CollectionData(); //Сбор сырых данных для обучения нейросети
      break;
    case 2:
      SendData(); //Отправка собранных данных
      break;
    case 3:
      EasyElectronicDiff(); //Режим электронного дифференциала Анкермана-Джекмана
      break;
    case 4:
      SystemTest(); // Тест работоспособности системы
      break;
  }
}

//--------------------------------------------------------------//
//-----------------1. Основные режимы работы-------------------//
//--------------------------------------------------------------//

//--------1.1 Обработка Bluetooth-сигналов-------
void BluetoothControl() {

  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');

    int Start = CharToInt(command, 0);
    int SendingData = CharToInt(command, 1);
    int CarDirrection = CharToInt(command, 2);
    int Acceleration = CharToInt(command, 3);

    //Читаем байты скорости для левого двигателя
    int HundredSpeed_1 = CharToInt(command, 4); //Количество сотен
    int TenSpeed_1 = CharToInt(command, 5); //Количество десятков
    int UnitSpeed_1 = CharToInt(command, 6); //Количество единиц
    LeftMotorSpeed = HundredSpeed_1 * 100 + TenSpeed_1 * 10 + UnitSpeed_1;

    //Читаем байты скорости для правого двигателя
    int HundredSpeed_2 = CharToInt(command, 7); //Количество сотен
    int TenSpeed_2 = CharToInt(command, 8); //Количество десятков
    int UnitSpeed_2 = CharToInt(command, 9); //Количество единиц
    RightMotorSpeed = HundredSpeed_2 * 100 + TenSpeed_2 * 10 + UnitSpeed_2;

    //Читаем байты угла для сервопривода
    int HundredServo = CharToInt(command, 10); //Количество сотен
    int TenServo = CharToInt(command, 11); //Количество десятков
    int UnitServo = CharToInt(command, 12); //Количество единиц

    ServoAngle = HundredServo * 100 + TenServo * 10 + UnitServo; // Угол для сервопривода

    if (Start == 1) {
      digitalWrite(GreenLED, HIGH);
    }

    if (SendingData == 1) {
      SendData();
    }

    if (CarDirrection == 1) {
      BlinkLight(RedLED);
    }

    // Включим двигатели, если газ и направление вперёд
    if (Acceleration == 1 && CarDirrection == 0) {
      analogWrite(LeftMotorForward, LeftMotorSpeed);
      //analogWrite(RightMotorForward, RightMotorSpeed);
      analogWrite(RightMotorBack, RightMotorSpeed);
    }

    // Включим двигатели, если газ и направление назад
    if (Acceleration == 1 && CarDirrection == 1) {
      analogWrite(LeftMotorBack, LeftMotorSpeed);
      //analogWrite(RightMotorBack, RightMotorSpeed);
    }

    //Если "газ" отпущен
    else if (Acceleration == 0) {
      digitalWrite(LeftMotorForward, LOW);
      digitalWrite(RightMotorForward, LOW);
      digitalWrite(LeftMotorBack, LOW);
      digitalWrite(RightMotorBack, LOW);
    }

    //Спозиционируем сероводвигатель
    SteerServo.write(ServoAngle);
  }
}

//---------1.2 Сбор данных с датчиков--------
void CollectionData() {
  eeprom_update_byte(0, 1);
}

//------------1.3 Отправка данных--------------
void SendData() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  temp = mpu.getTemperature();
  Serial.print(LeftMotorSpeed); Serial.print("\t");
  Serial.print(RightMotorSpeed); Serial.print("\t");
  Serial.print(ServoAngle); Serial.print("\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.print(gz); Serial.print("\t");
  Serial.print(temp); Serial.print("\t");
  Serial.println();
}

//------1.4. Функция простого электронного дифференциала----
void EasyElectronicDiff() {
  byte CarWay[] = {}; //Массив пути для демонстрации работы
  ElectronicDiff(1); //Расчитаем скорости
}

//------1.4. Тест приводов и вспомогательных систем----
void SystemTest () {
  //Подкрутим двигатели
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  delay(1000);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorForward, LOW);
  delay(2);
  digitalWrite(LeftMotorBack, HIGH);
  digitalWrite(RightMotorBack, HIGH);
  delay(1000);
  digitalWrite(LeftMotorBack, LOW);
  digitalWrite(RightMotorBack, LOW);
  delay(2);

  //Подвигаем сервоприводом
  SteerServo.write(120);
  delay(1000);
  SteerServo.write(60);
  delay(1000);
  SteerServo.write(90);
  delay(1000);

  // Проверим индикаю и соленоид
  digitalWrite(CoilPin, HIGH);
  digitalWrite(RedLED, HIGH);
  digitalWrite(GreenLED, HIGH);
  delay(1000);
  digitalWrite(CoilPin, LOW);
  digitalWrite(RedLED, LOW);
  digitalWrite(GreenLED, LOW);
}

//------1.5. Регулятор на нечёткой логике----
void FuzzyLogic (int input) {
  // Подадим на вход
  fuzzy->setInput(1, input);
  // Начинанием фазификацию
  fuzzy->fuzzify();
  // Начинаем дефазификацию
  float output = fuzzy->defuzzify(1);
  return output;
}

//------1.6. Инициализая регулятора на нечёткой логике----
void FuzzyLogicSetup () {
  //--Настройка входа--
  FuzzyInput *inputSpeed = new FuzzyInput(1); // Инициализируем объект FuzzyInput

  FuzzySet *low = new FuzzySet(-0.3 * speedmax, -0.1 * speedmax, 0.25 * speedmax, 0.5 * speedmax); // Инициализируем объект функции активации FuzzySet
  inputSpeed->addFuzzySet(low);// Включаем FuzzySet в FuzzyInput
  FuzzySet *middle = new FuzzySet(0.3 * speedmax, 0.5 * speedmax, 0.5 * speedmax, 0.7 * speedmax);
  inputSpeed->addFuzzySet(middle);
  FuzzySet *hight = new FuzzySet(0.5 * speedmax, 0.7 * speedmax, 0.7 * speedmax, 0.85 * speedmax);
  inputSpeed->addFuzzySet(hight);
  FuzzySet *done = new FuzzySet(0.85 * speedmax, 0.9 * speedmax, 1 * speedmax, 1.3 * speedmax);
  inputSpeed->addFuzzySet(done);

  fuzzy->addFuzzyInput(inputSpeed); // Включаем FuzzyInput в Fuzzy

  //--Настройка выхода--
  // Инициализируем объект FuzzyOutput
  FuzzyOutput *outputSpeed = new FuzzyOutput(1);

  FuzzySet *slow = new FuzzySet(0, 0.01 * speedmax, 0.01 * speedmax, 0.011 * speedmax); // Инициализируем объект функции активации FuzzySet
  outputSpeed->addFuzzySet(slow); // Включаем FuzzyOutput в Fuzzy
  FuzzySet *average = new FuzzySet(0.011 * speedmax, 0.25 * speedmax, 0.25 * speedmax, 0.5 * speedmax);
  outputSpeed->addFuzzySet(average);
  FuzzySet *fast = new FuzzySet(0.2 * speedmax, 0.5 * speedmax, 0.5 * speedmax, 0.8 * speedmax);
  outputSpeed->addFuzzySet(fast); // Включаем FuzzyOutput в Fuzzy
  FuzzySet *extremeFast  = new FuzzySet(0.75 * speedmax, 1 * speedmax, 1 * speedmax, 1 * speedmax);
  outputSpeed->addFuzzySet(extremeFast);

  fuzzy->addFuzzyOutput(outputSpeed);


  //-- Построение правила if (Speed is Low) then (Out is Extreme_Fast)--
  FuzzyRuleAntecedent *ifInputSpeedLow = new FuzzyRuleAntecedent(); // Создаём объект FuzzyRuleAntecedent
  ifInputSpeedLow->joinSingle(low); // Создаём FuzzyRuleAntecedent с одним FuzzySet
  FuzzyRuleConsequent *thenOutputSpeedExFast = new FuzzyRuleConsequent(); // Инициализируем объект FuzzyRuleConsequent
  thenOutputSpeedExFast->addOutput(extremeFast); // Включаем FuzzySet в FuzzyRuleConsequent
  FuzzyRule *fuzzyRule01 = new FuzzyRule(1, ifInputSpeedLow, thenOutputSpeedExFast); // Инициализируем объекты FuzzyRule
  fuzzy->addFuzzyRule(fuzzyRule01); // Включаем FuzzyRule в Fuzzy

  //-- Построение правила if (Speed is Middle) then (Out is Fast)--
  FuzzyRuleAntecedent *ifInputSpeedMiddle = new FuzzyRuleAntecedent(); // Создаём объект FuzzyRuleAntecedent
  ifInputSpeedMiddle->joinSingle(middle); // Создаём FuzzyRuleAntecedent с одним FuzzySet
  FuzzyRuleConsequent *thenOutputSpeedFast = new FuzzyRuleConsequent(); // Инициализируем объект FuzzyRuleConsequent
  thenOutputSpeedFast->addOutput(fast); // Включаем FuzzySet в FuzzyRuleConsequent
  FuzzyRule *fuzzyRule02 = new FuzzyRule(1, ifInputSpeedMiddle, thenOutputSpeedFast); // Инициализируем объекты FuzzyRule
  fuzzy->addFuzzyRule(fuzzyRule02); // Включаем FuzzyRule в Fuzzy

  //-- Построение правила if (Speed is High) then (Out is Average)--
  FuzzyRuleAntecedent *ifInputSpeedHigh = new FuzzyRuleAntecedent(); // Создаём объект FuzzyRuleAntecedent
  ifInputSpeedHigh->joinSingle(hight); // Создаём FuzzyRuleAntecedent с одним FuzzySet
  FuzzyRuleConsequent *thenOutputSpeedAverage = new FuzzyRuleConsequent(); // Инициализируем объект FuzzyRuleConsequent
  thenOutputSpeedAverage->addOutput(average); // Включаем FuzzySet в FuzzyRuleConsequent
  FuzzyRule *fuzzyRule03 = new FuzzyRule(1, ifInputSpeedHigh, thenOutputSpeedAverage); // Инициализируем объекты FuzzyRule
  fuzzy->addFuzzyRule(fuzzyRule03); // Включаем FuzzyRule в Fuzzy

  //-- Построение правила if (Speed is Done) then (Out is Slow)--
  FuzzyRuleAntecedent *ifInputSpeedDone = new FuzzyRuleAntecedent(); // Создаём объект FuzzyRuleAntecedent
  ifInputSpeedDone->joinSingle(done); // Создаём FuzzyRuleAntecedent с одним FuzzySet
  FuzzyRuleConsequent *thenOutputSpeedSlow = new FuzzyRuleConsequent(); // Инициализируем объект FuzzyRuleConsequent
  thenOutputSpeedSlow->addOutput(slow); // Включаем FuzzySet в FuzzyRuleConsequent
  FuzzyRule *fuzzyRule04 = new FuzzyRule(1, ifInputSpeedDone, thenOutputSpeedSlow); // Инициализируем объекты FuzzyRule
  fuzzy->addFuzzyRule(fuzzyRule04); // Включаем FuzzyRule в Fuzzy
}

//--------------------------------------------------------------//
//----------------2 Функции отработки прерываний--------------
//--------------------------------------------------------------//

// 2.1. Обработка прерываний с левого оптического датчика
void EncoderLeftTick() {
  LeftMotorTurnCounts++;
}

// 2.2. Обработка прерываний с правого оптического датчика
void EncoderRightTick() {
  RightMotorTurnCounts++;
}

//--------------------------------------------------------------//
//-----------------3. Иные вспомогательные функциии--------------
//--------------------------------------------------------------//

//------------3.1. Очистка EEPROM-памяти-----------------
void MemoryClean () {
  for (int i = 0; i <= 1023; i++ ) {
    eeprom_update_byte(i, 0);
  }
}

//------3.2. Функция мерцания светодиодом-----
void BlinkLight (int LED) {
  digitalWrite(LED, LOW);
  if (millis() - myTimer1 >= BlinkPeriod) {
    myTimer1 += BlinkPeriod;
    digitalWrite(LED, !digitalRead(LED));
  }
}

//------3.3. Функция отключения текущего режима и выход в ожидание-----
void StandBy() {
  digitalWrite(ServoMotor, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(LeftMotorBack, LOW);
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(RightMotorBack, LOW);
  digitalWrite(CoilPin, LOW);
}

//------3.4. Функция простого электронного дифференциала----
void ElectronicDiff(int Angle) {
  float AngleRad = Angle / 57.32; //Переведём градусы в радианы
  float tanAngle = tan(fabs(AngleRad));
  float delta1 = atan((L / (L / tanAngle - K / 2)));
  float delta2 = atan((L / (L / tanAngle + K / 2)));
  float R1 = L / sin(delta1);
  float R2 = L / sin(delta2);
  float R3 = L / tanAngle - dr / 2;
  float Rcg = sqrtf(R3 + dr * dr / 4 + lr * lr);
  Omega1 = (LinearIdleSpeed * R1) / (Rcg * r);
  Omega2 = (LinearIdleSpeed * R2) / (Rcg * r);
}

//------3.5. Функция для раскодирования пакета данных----
int CharToInt (String com, int indx) {
  char Char = com.charAt(indx);
  switch (Char) {
    case '0':
      return 0;
      break;
    case '1':
      return 1;
      break;
    case '2':
      return 2;
      break;
    case '3':
      return 3;
      break;
    case '4':
      return 4;
    case '5':
      return 5;
    case '6':
      return 6;
    case '7':
      return 7;
    case '8':
      return 8;
    case '9':
      return 9;
  }
}
