# Installation instructions

Execute these from the "terminal" window of vscode on your laptop, or on the Raspberry Pi.

## Start with an update

```
sudo apt update 
sudo apt upgrade 

```

## Apt packages:

```
sudo apt install python3-pip 
sudo apt install python3-setuptools 
sudo apt install python3-wheel 
sudo apt install python3-numpy 
sudo apt install libcamera-dev 
sudo apt install python3-picamera2 
sudo apt install python3-aiohttp 

```

## Python packages:

on the pi, these should be installed as sudo apt install python3-<thing>

```
python3 -m pip install numpy --break-system-packages
python3 -m pip install robotpy --break-system-packages 
python3 -m pip install robotpy-cscore --break-system-packages
python3 -m pip install robotpy-apriltag --break-system-packages
python3 -m pip install opencv-python --break-system-packages

```

## Gyro

```
python3 -m pip install hidapi --break-system-packages
python3 -m pip install adafruit-blinka --break-system-packages
python3 -m pip install adafruit-circuitpython-lsm6ds --break-system-packages

```

## Quick Paste

you can also just paste this into the pi and type y a bunch in 2025

```
sudo apt update 
sudo apt upgrade 
sudo apt install python3-pip 
sudo apt install python3-setuptools 
sudo apt install python3-wheel 
sudo apt install python3-numpy 
sudo apt install libcamera-dev 
sudo apt install python3-picamera2 
sudo apt install python3-aiohttp 
python3 -m pip install numpy --break-system-packages
python3 -m pip install robotpy --break-system-packages 
python3 -m pip install robotpy-cscore --break-system-packages
python3 -m pip install robotpy-apriltag --break-system-packages
python3 -m pip install opencv-python --break-system-packages
python3 -m pip install hidapi --break-system-packages
python3 -m pip install adafruit-blinka --break-system-packages
python3 -m pip install adafruit-circuitpython-lsm6ds --break-system-packages

```