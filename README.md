# Differential Drive Robot

*****
## Описание
Проект предназначен для управления мобильным роботом с дифференциальной кинематикой.

*****
## Оглавление
1. [Конфигурация робота](#Конфигурация-робота)
2. [Программное обеспечение](#Программное-обеспечение)
3. [Структура репозитория](#Структура-репозитория)
4. [Описание классов](#Описание-классов)
    - [TwoWheeledRobot](#TwoWheeledRobot_desc)
    - [MotorBlock](#MotorBlock_desc)
    - [Velosity](#Velosity_desc)
    - [Position](#Position_desc)
    - [MotionController](#MotionController_desc)
    - [DiffDriveController](#DiffDriveController_desc)
    - [MotionControllerByPID](#MotionControllerByPID_desc)
    - [MotionControllerByPID](#MotionControllerByPID_desc)
    - [MagneticLineSensor](#MagneticLineSensor_desc)
5. [Using](#Using)
6. [License](#License)

*****
## Конфигурация робота

Робот включает в себя следующие модули и датчики:

- Single board computer **Raspberry pi 4 8gb**
- Hardware platform **Arduino Mega 2560**
- Motor driver **TB6612FNG**
- Hall-Effect Magnetic Encoder **AS5600 x2**
- I2C Multiplexer **CJMCU-9548 (for encoders)**

[:arrow_up_small:Оглавление](#Оглавление)
*****
## Программное обеспечение
This code works on Arduino Mega 2560 using **PlatformioIO**

You can download [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation.html#piocore-install-shell-commands) separately.

Or use the [Visual Studio Code IDE Extension](https://platformio.org/install/ide?install=vscode).

(Please note that you do not need to install PlatformIO Core if you are going to use PlatformIO IDE. PlatformIO Core is built into PlatformIO IDE and you will be able to use it within PlatformIO IDE Terminal.)

[:arrow_up_small:Оглавление](#Оглавление)
*****
## Структура репозитория

### Стартовый файл
> <em>src</em>
>> <em>main.cpp</em>

### Файл с параметрами
> <em>src</em>
>> <em>InitData</em>
>>> <p><em>constants.h</em> - в файле задаются постоянные значения параметров робота, пинов подключения датчиков, коэффициентов и т.д.</p>

### Классы
> <em>lib</em>
>> <em>[TwoWheeledRobot](#TwoWheeledRobot_desc)</em> - главный класс, в котором описываются методы управления мобильным роботом.</br>
>> <em>[MotorBlock](#MotorBlock_desc)</em> - класс для управления мотором по скорости с обратной связью по энкодеру.</br>
>> <em>[Velosity](#Velosity_desc)</em> - класс для хранения скорости движения робота и угловых скоростей колёс дифференциального привода.</br>
>> <em>[Position](#Position_desc)</em> - класс для хранения положения и направления робота в пространстве. Включает в себя колёсную одометрию.</br>
>> <em>[MotionController](#MotionController_desc)</em> - контроллер управления движением. Для расчёта желаемых скоростей использует <em>DiffDriveController</em>.</br>
>> <em>[DiffDriveController](#DiffDriveController_desc)</em> - контроллер управления дифференциальным приводом. Основан на расчёте кривизны траектории.</br>
>> <em>[MotionControllerByPID](#MotionControllerByPID_desc)</em> - контроллер управления движением. Для расчёта желаемых скоростей использует <em>DiffDriveControllerByPID</em>.</br>
>> <em>[DiffDriveControllerByPID](#DiffDriveControllerByPID_desc)</em> - контроллер управления дифференциальным приводом. Основан на PID регуляторе на вход которого подаётся ошибка направления движения.</br>
>> ~~<em>HallSensorReader</em> - класс для описания датчиков Холла используемых в <em>MagneticLineSensor</em>.~~ _Необходимо обновление_</br>
>> ~~<em>[MagneticLineSensor](#MagneticLineSensor_desc)</em> - класс для работы с самодельным датчиком магнитной линии.~~ _Необходимо обновление_</br>
>> ~~<em>RFIDReader</em> - класс для работы с RFID метками. Использует стороннюю библиотеку MFRC522.~~ _Необходимо обновление_

### Библиотеки
> <em>lib</em>
>> <em>Encoder</em> - библиотека для считывания позиции энкодера. Использует стороннюю библиотеку AS5600.</br>
>> <em>PID</em> - библиотека для работы с PID регулятором.</br>

### Сторонние библиотеки 
> <em>lib</em>
>> <em>AS5600</em> - библиотека для взаимодействия с датчиком угла поворота AS5600. [GPL-3.0 License](https://choosealicense.com/licenses/gpl-3.0/)</br>
>> <em>MFRC522</em> - библиотека для использования RFID модуля Arduino MFRC522.</br>
>> <em>TimerMs</em> - многофункциональный программный таймер на системном таймере millis() для Arduino.</br>

[:arrow_up_small:Оглавление](#Оглавление)
*****
## Описание классов 

> :information_source: Все параметры для работы классов описаны в файле с параметрами [constants.h](lib/InitData/constants.h).

<a name="TwoWheeledRobot_desc">***TwoWheeledRobot***</a> - главный класс, в котором описываются методы управления мобильным роботом.

_Основные методы:_
- <em>serialControl</em> - основной метод для управления мобильным роботом из консоли, позволяет из консоли выбрать алгоритм управления мобильным роботом. Запускается из src/main.cpp. Выхода из метода не предполагается.
- <em>globalSerialControl</em> - функция для проверки вводимых символов в консоль в глобальной системе контроля. Необходимо использовать во всех методах управления для доступа оператора к эксренным командам. Символы, на которые реагирует метод:<br />
`z` - останавливает движение робота, эксренный выход из запущенного метода
- <em>manualControl</em> - метод управления движением робота с консоли. Для управления используется клавиатура компьютера:<br />
`w` - forward  
`x` - back  
`a` - left turn  
`d` - right turn  
`Space` - stop moving  
`e` - increase the speed of rotation of the wheels  
`q` - reduce the speed of rotation of the wheels  
`Ctrl+c` - shutdown client and server
- <em>setPositionAndStartMove</em> - функция, через которую задаются целевая координата и недостающие параметры для контроллера управления движением. Запускает метод <em>moveToTargetPosition</em>.
- <em>moveToTargetPosition</em> - метод управления движением мобильного робота с помощью контроллера управления движением. Работает до тех пор, пока мобильный робот не доедет до целевой позиции, после чего робот останавливается. В рамках метода реализовано плавное ускорение для уменьшения проявляения стохастических погрешностей при локализации с помощью колёсной одометрии.
- <em>resertPosition</em> - функция обнуления позиции и ошибок PID регулятора.
- <em>goCWtest</em> - запуск движения по квадратной траектории по часовой стрелке.
- <em>goCWtest</em> - запуск движения по квадратной траектории против часовой стрелки.
- ~~<em>goMagneticLinet</em> - запуск движения по магнитной линии.~~ _Не работает в текущей версии_
- ~~<em>goMagneticLine2Point</em> - запуск движения в заданную координату сетки, образуемой магнитными линиями.~~ _Не работает в текущей версии_
- <em>driveMoving</em> - функция задаёт необходимые скорости вращения колёс для моторов.
- <em>stopMoving</em> - функция останавливает вращение моторов.
- <em>timersStart</em> - функция задаёт параметры таймеров.
- <em>timersTick</em> - функция вызывает тик таймеров и обработку прерываний.

> :exclamation: Для обработки экстренных событий оператором во всех методах управления необходимо в рамках цикла вызывать globalSerialControl.<br />

> :exclamation: Все методы управления реализованы на таймерах, поэтому для корректной работы необходимо перед запуском метода запускать <em>timersStart</em> и в рамках цикла вызывать <em>timersTick</em>.<br />
> В текущей версии программы используется 3 таймера:
> - <em>discretTimer</em> - отвечает за время дискретизации для методов управления.
> - <em>motorTimer</em> - отвечает за время дискретизации для управления мощностью моторов. Чем меньше значение, тем чаще корректируется мощность двигателя для поддержания заданной скорости вращения, однако при этом уменьшается качество оценки скорости вращения с помощью энкодера.
> - <em>msgTimer</em> - отвечает за период отправки сообщений в консоль. Чем реже отправка, тем больше вычислительных ресурсов останется для алгоритмов управления.

<a name="MotorBlock_desc">***MotorBlock***</a> - класс для управления мотором. Позволяет управлять мощностью двигателя через задаваемую желамую скорость вращения колеса. Реализовано через PID регулятор с обратной связью через энкодер. Так как класс взаимодействует с энкодером, то он также включает в себя оценку скорости вращения колеса и оценку пройденного расстояния для колёсной одометрии.

_Основные методы:_
- <em>stopMoving</em> - останавливает вращение.
- <em>updateVelocity</em> - функция, задающая необходимую скорость колеса, которую необходимо поддерживать. Вызывается, когда необходимо задать новую скорость.
- <em>updatePWM</em> - функция обновляет мощность двигателя для поддержания заданной скорости вращения колеса через PID регулятор.
- <em>tickPrepare</em> - функция подгатавливает таймеры к работе.
- <em>tick</em> - функция для обработки прерывания. Выполняет расчёт угола поворота колеса за время после предыдущего измерения. Выполняет расчёт скорости вращения.
- <em>getAngleOffset</em> - возвращает угол поворота колеса за время после предыдущего измерения.
- <em>getLastSpeed</em> - возвращает последнюю рассчитанную скорость.

> :grey_exclamation: Для корректной работы энкодера и расчёта угла поворота и скорости вращения необходимо постоянно вызывать <em>tick</em>.<br />
> :grey_exclamation: Для избежания всплесков при запуске необходимо перед началом работы с мотором вызывать <em>tickPrepare</em>.<br />
> :grey_exclamation: Для поддержания скорости вращения мотора необходимо постоянно вызывать <em>updatePWM</em>.<br />
> :exclamation: Вызов данных функций в текущей версии происходит автоматически в рамках функций <em>timersStart</em> и <em>timersTick</em> в главном классе <em>[TwoWheeledRobot](#TwoWheeledRobot_desc)</em>

<a name="Velosity_desc">***Velosity***</a> - класс для хранения скоростей движения робота (лиейной и угловой) и угловых скоростей колёс дифференциального привода. Позволяет конвертировать угловую скорость из [rad/s] в частоту оборотов [rmp] и обратно. Позволяет конвертировать угловую скорость колёс [rad/s] в линейную скорость движения [m/s] и обратно.

<a name="Position_desc">***Position***</a> - класс для хранения положения и направления робота в пространстве. Включает в себя оценку локализации по колёсной одометрии.

_Основные методы:_
- <em>estCurrentPosition</em> - метод принимает на вход углы, на которые повернулись колёса, и параметры робота (ширина базы и радиус колеса) и рассчитывает текущее местоположение и ориентацию мобильного робота.

> :grey_exclamation: На данный момент колёсная одометрия рассчитывается в рамках класса <em>[Position](#Position_desc)</em>. Данные об углах поворта колёс для оценки местоположения берутся из класса <em>[MotorBlock](#MotorBlock_desc)</em>.

<a name="MotionController_desc">***MotionController***</a> - контроллер управления движением. Используется для оценки положения мобильного робота относительно реального местоположения и целевой координаты, а также для расчёта желаемых скоростей движения робота для достижения заданной координаты. Для расчёта желаемых скоростей использует контроллер управления дифференциальным приводом <em>[DiffDriveController](#DiffDriveController_desc)</em>.</br>
Возможности:
- Расчёт угола между направлением на целевую точку и текущим направлением движения робота.
- Расчёт расстояния между текущим положением робота и целевой позицией.
- Оценка достигаемости целевой позиции.
- Расчёт желаемых скоростей робота (линейная и угловая).
- Расчёт желаемых скоростей колёс дифференциального привода.

<a name="DiffDriveController_desc">***DiffDriveController***</a> - контроллер управления дифференциальным приводом. Основан на расчёте кривизны траектории.</br>
Возможности:
- Расчёт кривизны траектории движения робота.
- Расчёт желаемых скоростей робота (линейная и угловая) исходя из кривизны траектории и расстояния до целевого положения.
- Расчёт желаемых скоростей колёс дифференциального привода.

Разработанный алгоритм расчёта желаемых скоростей основан на следующей статье:</br>
О. И. Давыдов, А. К. Платонов, “Алгоритм управления дифференциальным приводом мобильного робота РБ-2”, Препринты ИПМ им. М. В. Келдыша, 2015, 025, 16 с.

<a name="MotionControllerByPID_desc">***MotionControllerByPID***</a> - контроллер управления движением. Является взаимно заменяемым для контроллера управления движением <em>[MotionController](#MotionController_desc)</em>. Отличается тем, что для расчёта желаемых скоростей используется контроллер управления дифференциальным приводом <em>[DiffDriveControllerByPID](#DiffDriveControllerByPID_desc)</em>.</br>

> :exclamation: Единственное отличие от <em>[MotionController](#MotionController_desc)</em> заключается в необходимости установить время для PID регулятора перед началом работы с классом с помощью функции _setDelay_.

<a name="DiffDriveControllerByPID_desc">***DiffDriveControllerByPID***</a> - контроллер управления дифференциальным приводом. Основан на PID регуляторе, на вход которого подаётся ошибка направления движения.</br>
Возможности:
- Расчёт желаемых скоростей робота (линейная и угловая).
- Расчёт желаемых скоростей колёс дифференциального привода.

<a name="MagneticLineSensor_desc">***MagneticLineSensor***</a> - класс для работы с самодельным линейным датчиком магнитной линии. Позволяет считывать значения с датчиков Холла на наличие магнитного поля с магниной ленты и визульно отображать информацию с помощью светодиодов на датчике.</br>

<img src="img/Датчик магнитной линии.png" width="300" title="Датчик магнитной линии">

[:arrow_up_small:Оглавление](#Оглавление)
*****
## Using
Clone the repository:

```bash
git clone https://github.com/industrial-robotics-lab/differential-drive-robot.git
```
Go to the project folder:
```bash
cd differential-drive-robot
```

For remote control, it is necessary that the robot (Raspberry Pi) and the PC have a connection to the same Wi-Fi network.
You need to find out the IP address of the Raspberry Pi, this can be done with the following command:
```bash
ifconfig
```
(section wlan0: inet \<IP-adress\>)

This IP address must be specified in the scripts **SocketServer.py** and **SocketClient.py** (variable HOST).

Start the server on Raspberry Pi:
```bash
python3 scripts/SocketServer.py
```


Run the client on PC:
```bash
python3 scripts/SocketClient.py
```

[:arrow_up_small:Оглавление](#Оглавление)
*****
## License
[MIT](https://choosealicense.com/licenses/mit/)

[:arrow_up_small:Оглавление](#Оглавление)
