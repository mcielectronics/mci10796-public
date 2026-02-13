# LoRa Breakout

![Perspectiva](assets/perspectiva.png)

**LoRa-Breakout** es una tarjeta diseñada por **[MCI Electronics](https://mcielectronics.cl/)** que facilita y flexibiliza el uso del módulo LoRa SX1276, permitiendo la transmisión de información de forma segura con controladores de 3,3 o 5V, como [Arduino Uno](https://mcielectronics.cl/shop/product/arduino-uno-r3-arduino-10230/) o [Raspberry Pi PICO](https://mcielectronics.cl/shop/product/kit-de-inicio-raspberry-pi-pico-30215/). Gracias a esta tarjeta es más sencillo conseguir todos los beneficios de LoRa, como su largo alcance y alta seguridad ante intercepción, siendo ideal para ser aplicada en IoT.

## Características principales
- Módulo LoRa: SX1276 (915 MHz)
- Antena: Integrada tipo Coil / Externa u.FL
- Conexiones: VIN, GND, EN, SCK, MISO, MOSI, CS, RST, DIO0 – DIO5
- Voltaje soportado: 3.3V – 5V
- Medidas: 32 x 48mm

## Conexiones

![Conexiones del módulo LoRa Breakout](assets/conexiones.png)

## Empezando a usar LoRa Breakout

### Requerimientos de hardware
- Computador.
- Arduino Nano, Raspberry Pi Pico u otro microcontrolador compatible.
- Cable USB compatible con nuestro microcontrolador.
- Cables Hembra Hembra
- Antena con conector Ipex

### Conexión de la antena

Para que el módulo pueda mandar mensajes estables a larga distancia es necesario conectarle una antena con conector Ipex. De lo contrario, la distancia de transmisión será demasiado corta e inestable.

![Antena siendo conectada a Lora Breakout](assets/ensamble_antena.png)

## Conexión usando Arduino

### Requerimientos de software con Arduino
- [Arduino IDE](https://support.arduino.cc/hc/en-us/articles/360019833020-Download-and-install-Arduino-IDE)
- Librería de la tarjeta (En caso de no usar una de las predeterminadas en Arduino IDE.)
- Librería [LoRa de Sandeepmistry](https://github.com/sandeepmistry/arduino-LoRa)

### Conexiones

Las conexiones cambian según qué microcontrolador se utilice. Lo importante es que:
- VIN se conecte 3.3V o 5V
- GND se conecte a GND
- SCK, MISO y MOSI se conecten a pins del mismo tipo
- CS, RST y D0 son programables con  LoRa.setPins(CS, RST, D0);, pero de forma predeterminada vienen los pins 10, 9 y 2 respectivamente.
- El pin EN sirve para apagar el módulo. No es necesario conectarlo
- El pin D0 solo es necesario conectar si se quiere utilizar el modo de recibir llamada.
- El resto de pins digitales sirven para controlar funciones internas del módulo y no son necesarias de conectar.

### Código transmisor

```cpp
#include <SPI.h>
#include <LoRa.h>

int counter = 0;

void setup() {
  //LoRa.setPins(2, 3, 4); // define pines CS, reset, DIO0. Pins actuales vienen por predeterminado en la librería.
  Serial.begin(9600);
  while (!Serial);

  Serial.println(" ");
  Serial.println("Diagnosis LoRa");

  if (!LoRa.begin(915E6)) {
    Serial.println("Fallo de inicialización, revisa las conexiones");
    while (!LoRa.begin(915E6));
  }
  Serial.println("LoRa conectado correctamente");
}

void loop() {
  Serial.print("Enviando paquete: ");
  Serial.println(counter);
  LoRa.beginPacket();
  LoRa.print(counter);
  LoRa.endPacket();
  counter++;
  delay(1000);
}
```

### Código receptor

```cpp
#include <SPI.h>
#include <LoRa.h>

void setup() {
  LoRa.setPins(2, 3, 4);
  Serial.begin(9600);
  while (!Serial);
  Serial.println(" ");
  Serial.println("Diagnostico LoRa");
  if (!LoRa.begin(915E6)) {
    Serial.println("Fallo de inicialización, revisa las conexiones");
    while (!LoRa.begin(915E6));
  }
  Serial.println("LoRa conectado correctamente");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("Paquete recibido '");
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }
    Serial.print("' con RSSI ");
    Serial.println(LoRa.packetRssi());
  }
}
```

## Conexión usando Raspberry Pi Pico

### Requerimientos de software con Raspberry Pi Pico
- [Thonny](https://thonny.org/)

### Conexiones

Las conexiones cambian según qué microcontrolador se utilice. Lo importante es que:
- VIN se conecte 3.3V o 5V
- GND se conecte a GND
- Todo el resto de pins se define en el código. En este caso usaremos:
     - SCK al pin 2
     - MOSI al pin 3
     - MISO al pin 4
     - CS al pin 5
     - RST al pin 6
     - DIO0 al pin 7

### Código diagnóstico

```python
from machine import Pin, SPI
import time

# Pines SPI
spi = SPI(0, baudrate=5000000, polarity=0, phase=0,
          sck=Pin(2), mosi=Pin(3), miso=Pin(4))
cs = Pin(5, Pin.OUT)
reset = Pin(6, Pin.OUT)

cs.value(1)
reset.value(1)

REG_VERSION = 0x42
REG_OPMODE = 0x01
REG_IRQ_FLAGS = 0x12

def write_register(address, value):
    cs.value(0)
    spi.write(bytearray([address | 0x80, value]))
    cs.value(1)

def read_register(address):
    cs.value(0)
    spi.write(bytearray([address & 0x7F]))
    result = spi.read(1)
    cs.value(1)
    return result[0]

def reset_module():
    print("🔄 Reiniciando SX1276...")
    reset.value(0)
    time.sleep(0.01)
    reset.value(1)
    time.sleep(0.01)

def diagnostico_sx1276():
    print("🦦 Iniciando diagnóstico SX1276...\n")
    reset_module()
    version = read_register(REG_VERSION)
    print(f"🔍 RegVersion (0x42): {hex(version)}")
    if version == 0x12:
        print("✅ SX1276 detectado correctamente.\n")
    else:
        print("❌ RegVersion inesperado. Verifica conexiones SPI y alimentación.\n")
        return
    print("🧪 Probando escritura/lectura del registro RegOpMode (0x01)...")
    test_values = [0x80, 0x81, 0x82]
    success = False
    for val in test_values:
        write_register(REG_OPMODE, val)
        time.sleep(0.1)
        res = read_register(REG_OPMODE)
        print(f"   Escribí {hex(val)} → Leí {hex(res)}")
        if res == val:
            success = True
    if success:
        print("✅ Escritura y lectura de RegOpMode funcionan correctamente.\n")
    else:
        print("⚠️ Falló la escritura/lectura de RegOpMode. Revisa MOSI, CS y lógica SPI.\n")
    irq_flags = read_register(REG_IRQ_FLAGS)
    print(f"📡 RegIrqFlags (0x12): {bin(irq_flags)}")
    print("\n🧑‍🔬 Diagnóstico completo.")

diagnostico_sx1276()
```
