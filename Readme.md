# RE

### LCD Pins

(Numbering for when Pins are on the right side)

| Pin# | Name  |
| ---- | ----- |
| 0    | T_IRQ |
| 1    | T_DO  |
| 2    | T_DIN |
| 3    | T_DCS |
| 4    | T_CLK |
| 5    | MISO  |
| 6    | LED   |
| 7    | SCK   |
| 8    | MOSI  |
| 9    | DC    |
| 10   | RST   |
| 11   | CS    |
| 12   | GND   |
| 13   | VCC   |


### Pico Pins

| Pin# | Name       |
|------|------------|
| 0    | LCD_DC     |
| 1    | LCD_RST    |
| 2    | LCD_SCK    |
| 3    | LCD_TX     |
| 5    | LCD_CS     |
| 6    | PIO_DATA_0 |
| 7    | PIO_DATA_1 |
| 8    | PIO_DATA_2 |
| 9    | PIO_DATA_3 |
| 10   | PIO_DATA_4 |
| 11   | PIO_DATA_5 |
| 12   | PIO_DATA_6 |
| 13   | PIO_DATA_7 |
| 22   | CAM_RST    |
| 25   | DBG_LED    |
| 26   | CAM_SDA    |
| 27   | CAM_SCL    |


### Datarates

The LCD is connected via SPI, which was proven to work with at least 40Mbps. The LCD has a size of $(w, h) = (320, 240)$, with a bit depth of $16$ for RGB565. The theoretical max. FPS can be calculated as a function of the SPI frequency $f$, LCD size $s$ and bit-depth $b$:

$$ \text{fps} = \frac{f}{s\cdot b}$$

So for our application with $f=40e6$, $s = 320 \cdot 240 =76800$ and $b=16$, the result is:

$$\text{fps} = \frac{40e6}{76800\cdot 16} \approx 32.5$$

The OV2640 is capable of up to 60 fps at the lowest resolution, called "CIF", which is 352x288 pixels in size. The few publically available OV2640 registers are really confusing though...

## OV2640
### Resolutions
UXGA = 1600x1200; SVGA = 800x600; CIF = 400x296

These sizes describe the window size that is possible for each of the resolutions. The capture window can be placed anywhere inside the sensor boundaries of 1632x1220.
The origin of the window is the top left corner which can be configured by the `OFFSET_X` and `OFFSET_Y` parameters, which can be found in following registers:
```c
OFFSET_X[10:0] = {VHYX[2:0], XOFFL[7:0]}
OFFSET_Y[10:0] = {VHYX[6:4], YOFFL[7:0]}
```
The window (frame) **size** is configured by multiple registers with diffuse meaning. There are (probably) two different methods to configure the frame size:
1) enable the whole sensor in the biggest resolution and only transmit a part of it
2) only enable a part of the sensor and transmit everything that is captured

Option 1) is easier as only a few registers have to be reprogrammed to change the resolution at runtime. Option 2) is probably what is needed for higher fps applications, as the biggest resolution is limited to 15 fps.

There are two sets of parameters which appear equivalent, (`H_SIZE`, `V_SIZE`) and (`HSIZE`, `VSIZE`). The difference is not entirely clear. The corresponding registers for the first tuple are:
```
H_SIZE[9:0] = {TEST[7], VHYX[3], HSIZE[7:0]}
V_SIZE[8:0] = {VHYX[7], VSIZE[7:0]}
```
This gives these parameters the following max. values: (1023, 511). The `H_SIZE` and `V_SIZE` parameters are described to be interpreted as "*size*/4".

The other set of parameters, (`HSIZE`, `VSIZE`), can be found in the following registers:
```
HSIZE[11:0] = {SIZE_L[5], HSIZE8[7:0], SIZEL[4:3]}
VSIZE[10:0] = {VSIZE8[7:0], SIZEL[2:0]}
```
The range here is (4095, 2047).
Setting the tuples (`H_SIZE`, `V_SIZE`) and (`HSIZE`, `VSIZE`) to equivalent values works, it was not tested what happens if these are not equivalent.

In addition to modifying these parameters, there are additional register which appear to modify only the output dimensions, using only these will *probably* cause issues when using a clock rate for higher fps.

There are parameters `OUTW` and `OUTH` with the following register locations:
```
OUTW[9:0] = {ZMHH[1:0], ZMOW[7:0]}
OUTH[8:0] = {ZMHH[2], ZMOH[7:0]}
```
These are again scaled by $4$, so the real output is 4 times bigger than the value of these parameters.

### Timing
XVCLK = 12MHz

$CLK = \begin{cases}2\cdot \frac{XVCLK}{DIV} &CLKRC[7]\\\frac{XVCLK}{DIV}&\neg CLKRC[7]\end{cases}$

$PCLK = \begin{cases} \frac{SYSCLK (48)}{R\_DVP\_SP} &YUV / RGB\\ \frac{SYSCLK (48)}{ (2*R\_DVP\_SP)} &RAW\end{cases}$

It is not clear what the 48 is supposed to mean and whether SYSCLK refers to CLK or some other clock signal.

There are no equations for framerate calculations, only these for slave mode:
#### UXGA
$T_{line} = 1922 \times T_{clk}$

$T_{frame} = 1248 \times T_{line}$
#### SVGA
$T_{line} = 1190 \times T_{clk}$

$T_{frame} = 672 \times T_{line}$
#### CIF
$T_{line} = 595 \times T_{clk}$

$T_{frame} = 336 \times T_{line}$

Here, $T_{clk}$ refers to the supplied MCLK in slave mode.


### Framerate

The "datasheet" lists a few different options for controlling the framerate:
1) CLKRC, so defining a clock prescaler
2) dummy pixels per line, so slowing down the individual line transmission by some amount
3) entire dummy lines which are transmitted with each frame

Option 1) seems to be the most sane option, while the other options appear to only be relevant for finer framerate adjustments. Unsurprisingly, there is no equation for framerate calculations in the normal (non-slave) mode.

The framerate can be measured with an interrupt on the VSYNC line, we document some framerates in the hopes of being able to figure out exact register settings later on:

Our live preview is supposed to show CIF with some high enough fps value to feel responsive, some experiments:
```c
General:
CIF; RGB565; XVCLK=12MHz; R_DVP_SP=R_DVP_SP_AUTO_MODE (auto PCLK freq.)

CLKRC=0x00 => 40ms frametime, 25 fps
CLKRC=0x01 => 80ms frametime, 12.5 fps
CLKRC=0x02 => 120ms frametime, 8.33 fps
CLKRC=0x03 => 160ms frametime, 6.25 fps

CLKRC=0x80 => 20ms frametime, 50 fps
CLKRC=0x81 => 40ms frametime, 25 fps

```
