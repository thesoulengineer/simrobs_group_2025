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
3. Пройти в папку c репой и установить ее
```
pip install -e .
```
4. В окружении поднять CAN:
   1. Обновить список пакетов — `sudo apt update -y`
   2. Установить драйвер slcand — `sudo apt install can-utils`
   3. Проверить установку — `which slcand`. Должен выбать путь, что-то типа /usr/bin/slcand
   4. Проверить USB устройство и куда оно подключилось — `ls /dev/ | grep ttyACM` (например мне выдало ttyACM0)
   5. Создать подключение — `sudo slcand -o -c -s8 /dev/ttyACM0 can0`
   6. Поднять устройство — `sudo ip link set up can0`
   7. Проверить: `candump can0`
5. Чтобы скрипты заработали, они должны находиться с папке с драйвером