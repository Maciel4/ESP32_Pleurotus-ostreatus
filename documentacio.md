Documentación Completa
Cultivo de Pleurotus ostreatus con automatización ESP32

1. Introducción
Este manual cubre desde la selección de la especie y la preparación del sustrato, hasta la automatización del microclima con ESP32, sensores, actuadores y Firebase. Está pensado para un pequeño invernadero casero que permita optimizar humedad, temperatura, ventilación e iluminación.

2. Especie recomendada
Pleurotus ostreatus (seta blanca)
- Alta resistencia y rápida colonización.
- Temperatura de crecimiento: 10–35 °C.
- Rendimiento esperado: +10 kg de hongos por 1 kg de micelio bajo condiciones óptimas.
- Bajo riesgo de contaminaciones si se mantiene higiene estricta.

3. Etapas de crecimiento
- Preparación de sustrato
- Inoculación (siembra)
- Incubación (colonización)
- Inducción de fructificación
- Cosecha

4. Preparación de sustrato
Ingredientes y cantidades
| Ingrediente | Cantidad | Nota | 
| Paja seca | 10 kg | Cortar en trozos de 3–5 cm | 
| Agua | 6–7 L | Para lograr 65–70 % de humedad final | 
| Cal viva o regulador de pH | 100–150 g | Ajustar pH a 6,5–7,5 | 

Procedimiento
- Remojar la paja en agua caliente con cal (60–80 °C) durante 1 h.
- Escurrir y enfriar a 25 °C.
- Ajustar el pH a ~7 con el regulador.
- En ambiente limpio, mezclar con el micelio (2–5 % del peso húmedo).
- Empaquetar en bolsas con cierre o tapón, compactar para eliminar aire.

5. Inoculación
- Entorno limpio, sin corrientes de aire fuertes.
- Temperatura: 22–26 °C.
- Higiene: manos y herramientas con alcohol.
- Proporción: 20–50 g de micelio por cada 1 kg de sustrato húmedo.
- Sellar las bolsas y perforar 2–4 orificios pequeños para intercambio de aire lento.

6. Incubación
- Temperatura: 24–28 °C.
- Humedad: 80–90 %.
- Luz: escasa o nula.
- Duración: 14–21 días hasta plena colonización.
- Ventilación mínima.

7. Inducción de fructificación
- Reducir a 16–20 °C.
- Aumentar ventilación (cambiar CO₂ por O₂).
- Humedad: 90–95 % con nebulizadores.
- Luz difusa 12 h diarias (400–800 lux).
- Aparecen primordios en 3–5 días.

8. Cosecha
- Cuando los sombrerillos miden 5–10 cm y los bordes se aplanan.
- Girar suavemente o cortar con cuchillo limpio.
- Rendimiento: hasta 10 kg de hongos por 1 kg de micelio.

9. Diseño de mini-invernadero
- Caja plástica o estantería (60 × 60 × 120 cm).
- Cubierta con lona transparente.
- Orificios con malla anti-insectos.
- Fuente de iluminación difusa o LEDs blancos.
- Espacio para 8–10 bolsas.

10. Sensores y actuadores
Sensores recomendados
| Parámetro | Sensor | Comunicación | 
| Temp/Humedad | DHT22 | Digital 1-Wire | 
|  | BME280 (alt.) | I2C / SPI | 
| Luz | BH1750 | I2C | 
| CO₂ (opcional) | MH-Z19B | UART | 
| Humedad suelo | Soil capacitivo | Analógico ADC | 
| Temp puntual | DS18B20 | Digital 1-Wire | 
| Calidad aire | MQ-135 (opt.) | Analógico ADC | 


Actuadores
- Ventilador PC (5 V/12 V).
- Nebulizador ultrasónico 5 V.
- Bomba de agua + boquillas nebulizadoras (5 V/12 V).
- Humidificador AC (opcional, requiere relé).

11. Conexión eléctrica
ESP32
  ├─ GPIO14 → Res220Ω → Gate MOSFET1 → Ventilador 12 V (–)
  ├─ GPIO27 → Res220Ω → Gate MOSFET2 → Nebulizador 5 V (–)
  ├─ GPIO21 ──> DHT22 data
  ├─ 3.3 V ──> Vcc (DHT22, BH1750 SDA/SCL, DS18B20)
  ├─ GPIO22 ←─ SCL (BH1750)
  ├─ GPIO21 ←─ SDA (BH1750)
  └─ GND ──> Tierra común

12 V → + Ventilador, + Nebulizador (si aplicase)


Notas
- Tierra común para todos los dispositivos y ESP32.
- Resistencias de 10 kΩ Gate→GND en cada MOSFET.
- Diodos flyback (1N4007) en motores para protección.

12. Ejemplo de código (PlatformIO / Arduino)
#include <DHT.h>
#include <Wire.h>
#include <BH1750.h>

#define DHTPIN 21
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

#define FAN_PIN 14
#define NEO_PIN 27

BH1750 lightMeter;

void setup() {
  Serial.begin(115200);
  dht.begin();
  lightMeter.begin();

  pinMode(FAN_PIN, OUTPUT);
  pinMode(NEO_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(NEO_PIN, LOW);
}

void loop() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  uint16_t lux = lightMeter.readLightLevel();

  Serial.printf("T: %.1f °C  H: %.1f %%  L: %u lx\n", t, h, lux);

  // Control Humedad
  if (h < 85.0) digitalWrite(NEO_PIN, HIGH);
  else digitalWrite(NEO_PIN, LOW);

  // Control Ventilación
  if (h > 95.0) digitalWrite(FAN_PIN, HIGH);
  else digitalWrite(FAN_PIN, LOW);

  delay(2000);
}
![alt text](image.png)


13. Integración con Firebase (próximos pasos)
- Instalar librería Firebase ESP32.
- Configurar credenciales en platformio.ini y código.
- Enviar lecturas periódicas (setFloat()) a rutas de base de datos.
- Crear panel en Firebase Console para ver gráficas en tiempo real.

14. Cronograma y checklist
| Tarea | Duración estimada | Hecho | 
| Preparar sustrato | 1 día | [ ] | 
| Inocular y sellar bolsas | 0.5 días | [ ] | 
| Incubación (24–28 °C) | 14–21 días | [ ] | 
| Construir mini-invernadero | 1 día | [ ] | 
| Conectar sensores y actuadores | 0.5 días | [ ] | 
| Programar ESP32 y probar | 1 día | [ ] | 
| Inducción de fructificación | 5 días | [ ] | 
| Primera cosecha | 1 día | [ ] | 

15. Mantenimiento y buenas prácticas
- Desinfectar al iniciar cada etapa.
- Control diario de humedad y temperatura.
- Revisar trampas para insectos.
- Documentar datos y ajustar parámetros.





