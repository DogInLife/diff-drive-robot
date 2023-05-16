#ifndef TWO_WHEELED_ROBOT_H
#define TWO_WHEELED_ROBOT_H

#include <math.h>
#include "TimerMs.h"
#include "constants.h"
#include "pid.h"
#include "position.h"
#include "velocity.h"
#include "motorBlock.h"
#include "motionController.h"
//#include "motionControllerByPID.h"
#include "RFIDReader.h"
//#include <MFRC522.h>
#include "hallSensor.h"
#include "magneticLineSensor.h"

class TwoWheeledRobot
{
    private:    
        float wheelRadius;          // Средний радиус колёс [m]
        float baseLength;           // Ширина базы между колёсами [m]

        Velocity* vel;              // Скорость робота [m/s]
        Position* pos;              // Местоположение робота [m]

        MotorBlock* motorBlockL;    // Управление мотором левого колеса
        MotorBlock* motorBlockR;    // Управление мотором правого колеса

        MotionController* motionControler;   // Контроллер движения робота на основе кривизны траектории
        //MotionControllerByPID* motionControler;   // Контроллер движения робота на основе PID регулятора

        bool globalStop = true;     // Преждевременная остановка цикла. True - остановка необходима, иначе False
        byte inByte = 0;            // Входящий с консоли символ

        TimerMs* discretTimer;      // Время дискретизации
        TimerMs* motorTimer;      // Время дискретизации
        TimerMs* msgTimer;          // Период отправки сообщений через Serial

        /* Движение в заданную заранее на контроллере координату */ 
        void moveToTargetPosition(int del, bool deb);

    public:
        TwoWheeledRobot();
        ~TwoWheeledRobot();

        //================= CONTROL =================//
        /* Управление с консоли 
        del - частота дискретизации [ms] 
        msg_del - частота отправки сообщений через Serial [ms] 
        deb - true, если нужен дебаг
        */ 
        void serialControl(int del, int del_motor, int del_msg, bool deb);  
        /* Проверка вводимых символов в консоль в глобальной системе контроля */ 
        void globalSerialControl();           
        /* Проверка входящего символа в глобальной системе контроля 
        inB - входящий символ
        */ 
        void globalSerialControl(byte inB);
        /* Запуск ручного отправления */ 
        void manualControl(int dt);
        
        // ========= FUNCTIONS ===========
        /* Обнуление позиции и ошибок PID регулятора */ 
        void resertPosition();
        /* Обнуление ошибок PID регулятора регулятора */ 
        void resertMotorsPID();        
        /* Запуск движения в заданную координату.
        _x, _y - координты точки [m] 
        beta - конечный угол после достижения точки [rad] 
        del - частота [ms]
        deb - true, если нужен дебаг */ 
        void setPositionAndStartMove(float _x, float _y, int del, bool deb);
        /* Запуск движения в заданную координату. Конечный угол не учитывается.
        _x, _y - координты точки [m] 
        del - частота [ms]
        deb - true, если нужен дебаг */ 
        void setPositionAndStartMove(float _x, float _y, float beta, int del, bool deb);
        /* Запуск движения по квадратной траектории по часовой стрелке [CW]
        points - массив точек
        del - частота [ms]
        deb - true, если нужен дебаг */ 
        void goCWtest(float L, int del, bool deb);
        /* Запуск движения по квадратной траектории против часовой стрелки [CCW]
        L – сторона квадрата [m]
        del - частота [ms]
        deb - true, если нужен дебаг */ 
        void goCCWtest(float L, int del, bool deb);

        //void goMagneticLine(int del, bool deb);
        /* Запуск движения в заданную координату сетки, образуемой магнитными линиями
        _x, _y - координты перекрёстка
        del - частота [ms]
        deb - true, если нужен дебаг */ 
        //void goMagneticLine2Point(float _x, float _y, int del, bool deb);

        /* Запустить моторы 
        velL - скорость левого колеса [rad/s]
        velR - скорость правого колеса [rad/s]
        */ 
        void driveMoving(float velL, float velR);
        /* Остановиться */ 
        void stopMoving();

        /* Управление всеми таймерами */ 
        void timersStart();
        void timersTick();

        //================= GET =================//
        /* Возвращает байт информации из буфера последовательного порта */
        byte getSerialData();                    
};

#endif // TWO_WHEELED_ROBOT_H