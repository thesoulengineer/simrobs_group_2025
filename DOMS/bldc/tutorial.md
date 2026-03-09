# Запуск привода Cube Mars AK45-10

Страница с информацией о преводе https://www.cubemars.com/ru/product/ak40-10-robotic-actuator.html

### Отладка
Для отладки надо использовать программу [CubeMarsTool](https://img.cubemars.com/products/cubemars-product-parameter/CubeMarsTool_V3.1.3.zip) для Windows.

1. Подключить привод к блоку питания
2. К мотору подключить свисток R-LINK
3. Свисток к компьютеру
4. Включить блок питания
5. Запустить программу `Cubemarstool.exe`
6. Нажать кнопку `CONNECT` 
7. Во вкладке `MODE SWITCH` включить режим `MIT App`
8. Экспериментировать

### Заупуск кода

1. Создать окружение (venv или conda)
```
conda create --name myenv python=3.10
```
2. Активировать окружение и установить необходимый минимум 
```
pip install bitstring numpy matplotlib
```
3. Заклонить репу с [драйвером для управления приводом](https://github.com/dfki-ric-underactuated-lab/mini-cheetah-tmotor-python-can)
4. Пройти в папку c репой и установить ее
```
pip install -e .
```
5. В окружении поднять CAN:
   1. Обновить список пакетов
   ```
   sudo apt update -y
   ```
   2. Установить драйвер slcand
   ```
   sudo apt install can-utils
   ```
   3. Проверить установку
   ```
   which slcand
   ```
   Должен выбать путь, что-то типа /usr/bin/slcand
   4. Проверить USB устройство и куда оно подключилось
   ```
   ls /dev/ | grep ttyACM
   ```
   (например мне выдало ttyACM0)
   5. Создать подключение
   ```
   sudo slcand -o -c -s8 /dev/ttyACM0 can0
   ```
   6. Поднять устройство
   ```
   sudo ip link set up can0
   ```
   7. И наконец-то: 
   ```
   candump can0
   ```
6. Чтобы скрипты заработали, они должны находиться с папке с драйвером


# Starting the Cube Mars AK45-10 Actuator
Information page about the actuator: https://www.cubemars.com/product/ak40-10-robotic-actuator-58

### Debugging
For debugging, use the [CubeMarsTool](https://img.cubemars.com/products/cubemars-product-parameter/CubeMarsTool_V3.1.3.zip)  program for Windows.

1. Connect the actuator to the power supply
2. Connect the R-LINK dongle to the motor
3. Connect the dongle to the computer
4. Turn on the power supply
5. Run the `Cubemarstool.exe` program
6. Press the `CONNECT` button
7. In the `MODE SWITCH` tab, enable `MIT App` mode
8. Experiment!

### Running the Code
1. Create an environment (venv or conda)
```
conda create --name myenv python=3.10
```
2. Activate the environment and install the necessary minimum
```
pip install bitstring numpy matplotlib
```
3. Clone the repo with [the driver for controlling the actuator](https://github.com/dfki-ric-underactuated-lab/mini-cheetah-tmotor-python-can)
4. Navigate to the repo folder and install it
```
pip install -e .
```
5. Set up CAN in the environment:
   1. Update the package list
   ```
   sudo apt update -y
   ```
   2. Install the slcand driver
   ```
   sudo apt install can-utils
   ```
   3. Verify the installation
   ```
   which slcand
   ```
   It should show a path, something like /usr/bin/slcand
   4. Check the USB device and where it connected
   ```
   ls /dev/ | grep ttyACM
   ```
   (for example, it showed ttyACM0 for me)
   5. Create the connection
   ```
   sudo slcand -o -c -s8 /dev/ttyACM0 can0
   ```
   6. Bring up the device
   ```
   sudo ip link set up can0
   ```
   7. And finnaly
   ```
   candump can0
   ```
6. For the scripts to work, they must be located in the driver folder