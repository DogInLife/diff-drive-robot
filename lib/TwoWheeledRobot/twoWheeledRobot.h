#ifndef TWO_WHEELED_ROBOT_H
#define TWO_WHEELED_ROBOT_H

#include <math.h>

#include "constants.h"

#include "velocity.h"
#include "position.h"
#include "motorBlock.h"
#include "pid.h"
#include "motionController.h"

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
        PID* pidL;                  // PID для управления скоростью левого колеса
        PID* pidR;                  // PID для управления скоростью правого колеса

        MotionController* motionControler;   // Контроллер движения робота

        bool globalStop = true;     // Преждевременная остановка цикла. True - остановка необходима, иначе False
        int newMinRange = 0;        // Для функции map в setVelocity
        byte inByte = 0;            // Входящий с консоли символ

        /* Движение в заданную на контроллере координату */ 
        void goToPosition(int del, bool deb);

    public:
        TwoWheeledRobot();
        ~TwoWheeledRobot();

        //================= CONTROL =================//
        /* Управление с консоли */ 
        void serialControl(int del, bool deb);  
        /* Проверка вводимых символов в консоль в глобальной системе контроля */ 
        void globalSerialControl();           
        /* Проверка символа inB в глобальной системе контроля */ 
        void globalSerialControl(byte inB);
        /* Запуск ручного отправления */ 
        void manualControl(int dt);
        

        // ========= FUNCTIONS ===========
        /* Обнуление позиции и ошибок регулятора */ 
        void resertPosition();
        /* Запуск движения в заданную координату.
        _x, _y - координты точки [m] 
        beta - конечный угол после достижения точки [rad] 
        del - частота [ms]
        deb - true, если нужен дебаг */ 
        void startGoToPosition(float _x, float _y, int del, bool deb);
        /* Запуск движения в заданную координату. Конечный угол не учитывается.
        _x, _y - координты точки [m] 
        del - частота [ms]
        deb - true, если нужен дебаг */ 
        void startGoToPosition(float _x, float _y, float beta, int del, bool deb);
        /* Запуск движения по маршруту заданному точками.
        points - массив точек
        del - частота [ms]
        deb - true, если нужен дебаг */ 
        void goTrack(Position points[], int del, bool deb);
        /* Запуск движения по квадратной траектории по часовой стрелке [CW]
        L – сторона квадрата [m]
        del - частота [ms]
        deb - true, если нужен дебаг */ 
        void goCWtest(float L, int del, bool deb);
        /* Запуск движения по квадратной траектории против часовой стрелке [CCW]
        L – сторона квадрата [m]
        del - частота [ms]
        deb - true, если нужен дебаг */ 
        void goCCWtest(float L, int del, bool deb);
        /* Запуск движения по кругу 
        ... */ 
        void goCircle(float radius, int ptsNum, bool deb, int circles);
        /* Запуск движения по магнитной линии
        del - частота [ms]
        deb - true, если нужен дебаг */ 

        //void goMagneticLine(int del, bool deb);
        /* Запуск движения в заданную координату сетки, образуемой магнитными линиями
        _x, _y - координты перекрёстка
        del - частота [ms]
        deb - true, если нужен дебаг */ 
        //void goMagneticLine2Point(float _x, float _y, int del, bool deb);

        /* Двигаться прямо */ 
        void goForward(int velL, int velR);
        /* Двигаться влево */ 
        void turnLeft(int velL, int velR);
        /* Двигаться вправо */ 
        void turnRight(int velL, int velR);
        /* Остановиться */ 
        void stopMoving();

        //================= GET =================//
        /* Возвращает байт информации из буфера последовательного порта */
        byte getSerialData();                    
};

#endif // TWO_WHEELED_ROBOT_H
