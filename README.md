# Dehydrator Automation

I wanted a food dehydrator that shut itself off when finished. Being finished is also hard to define, so I measure weight manually and the device measures the water content of the air. I am also interested in dropping the temperature so as to minimize cost, maintain a constant food temperature, or a similar objective.

Bill of materials

  - food dehydrator
  - raspberry pi pico
  - IR led transmitter module
  - IR plug (HENGMING HM-01K3)
  - 2xSHT31D
  - SSD1306 OLED 128x32
  - ... scale, stepper, copper tape (or foil) for capacitative buttons

TODO fritzing drawing, actual pictures

## Clone, build, load

```
git clone --recursive https://github.com/aavogt/dehydrator
mkdir build
cmake . -Bbuild
cmake --build build
sudo picotool load -f build/dehydrator.uf2
```
