DroneUN · IAR-LARC 2026 · propuesta de electrónica para revisar

# Cableado del blimp con ESP32-S3-Zero

Cómo conectar los dos DRV8833, el GY-88, el VL53L1X y la ESP32-CAM al ESP32-S3-Zero, dónde poner el magnetómetro, cómo alimentar todo con las dos celdas 1S, y si vale la pena montar una segunda ESP32-CAM para tener visión estéreo.

9 de octubre de 2026Basado en las fotos del protoboard, de la báscula (61 g) y de las tres pruebas de empuje

Resumen

## Lo que hay que saber

Conexiones

Las ocho señales de los motores salen del borde derecho del S3-Zero (GP13 a GP7, más el pad GP14). El GY-88 y el VL53L1X comparten el bus I²C en GP4 y GP5. La ESP32-CAM habla por UART en GP2 y GP6, y GP1 mide la batería de motores. Así no se usan TX, RX ni el pin de arranque GP3. Todas las conexiones están en la figura 1 y en la tabla.

Alimentación

El MB102 no regula con celdas de 1S. Una celda va a los motores y otra a la lógica, con un convertidor step-up de 5 V para la cámara y el S3-Zero. El 3,3 V del S3-Zero solo alcanza para los dos sensores. Hay que comprar el step-up de 5 V y el diodo Schottky D1 (≈ 1–2 g), los fusibles F1 (≈ 7,5 A) y F2 (≈ 3 A), los interruptores S1 (8 A o más) y S2 (2 A o más), los capacitores y resistencias de la tabla, y una celda de 20 C o más para los motores: a fondo piden ≈ 6 A, y la etiqueta de las HJ 702035P no dice su tasa de descarga.

Magnetómetro

En tu protoboard no cumple por poco: el magnetómetro queda a ≈ 9,6 cm del DRV8833 #2, y a ≈ 1,3 cm de la ESP32-CAM. Con los dos drivers lado a lado en la placa final queda a ≈ 12 cm de ellos (figura 3). Además, trencen los cables de motor y de la cámara, y calibren la brújula con todo montado.

Visión estéreo

No vale la pena. El globo tiene tamaño conocido (50–60 cm), así que una sola cámara, calibrada con un globo real, mide la distancia igual o mejor que dos ESP32-CAM, y dos placas separadas no toman las fotos al mismo tiempo. Un lente gran angular de 120° amplía el campo de visión casi tanto como una segunda cámara, sin sumar peso ni consumo, pero con la mitad de precisión en la distancia.

Empuje

Un motor, probado solo con la fuente de laboratorio, dio 14,7 gf con 3,98 V (prueba rotulada «95 %») y 16,8 gf con 4,13 V (rotulada «100 %»); girando al revés daría ≈ 74 % (por confirmar). Falta confirmar si la fuente iba directo al motor. Si fue así, al pasar por el DRV8833 se pierden ≈ 0,6 V y, según un cálculo sin medir, quedan ≈ 12 gf por motor con la celda llena y ≈ 8–9 gf cuando baja a 3,7 V. En el simulador, con 16,8 gf completa todas las vueltas; con 11–13 gf deja sin completar 4–7 de 30, y con 7–9 gf, 9–22 de 30. Hay que recuperar empuje y reajustar el controlador.

Conexiones

## Cómo se conecta todo al ESP32-S3-Zero

La figura 1 es el esquema eléctrico: qué pin va con qué pin y de dónde sale la energía. La figura 2 muestra dónde está cada pin en el S3-Zero, para soldar sin equivocarse, y la tabla repite las conexiones y los componentes en forma de lista. Los nombres de los pines de los DRV8833, del GY-88 y de la cámara son los de las placas más comunes de cada tipo: compárenlos con la serigrafía de las suyas. Antes de soldar, revisen sus placas como dice la sección siguiente, «Antes de soldar». En el celular, las tablas anchas se deslizan hacia los lados.

Figura 1 · Diagrama de conexiones

batería (3,0–4,2 V) 5 V del step-up 3,3 V tierra PWM a los drivers motores I²C SDA I²C SCL UART medición de batería control opcional

Mirando el blimp desde atrás, hacia donde apunta la cámara: FL y FR son los motores delanteros izquierdo y derecho y empujan el aire hacia atrás; RL y RR son los traseros izquierdo y derecho y empujan el aire hacia abajo. Los símbolos ⏚ son la tierra común. * G_AD0 va a GND solo si la serigrafía de su GY-88 dice G_AD0. Cada DRV8833 mueve un motor de avance y uno de altura para repartir el calor. Las líneas punteadas son opcionales. En el celular, deslicen el diagrama hacia los lados. Al final de la página está la misma figura en PNG.

Figura 2 · Dónde soldar cada cable en el S3-Zero Vista desde arriba, con el USB-C hacia arriba. El GND es el único pad cuadrado, en el dibujo y en la placa. Los pads rotulados TX y RX no se usan: la cámara va por GP2 y GP6. Los pads en blanco quedan libres: TX imprime texto al arrancar, TX y RX arrancan con pull-up, y el 3 es un pin de arranque del chip (si quieren, sirve para medir la celda de la lógica con su propio divisor). El pad 14 es pequeño y está en la cara de arriba, dentro de la placa: hay que soldarle un cable fino.

### Orden físico de los pines de cada placa

| Placa | Orden de los pines | Cuidado |
| --- | --- | --- |
| DRV8833 (módulo de 12 pines) | EEP · OUT1 · OUT2 · OUT3 · OUT4 · ULT en un lado; IN4 · IN3 · GND · VCC · IN2 · IN1 en el otro, leídos desde el mismo extremo: EEP queda frente a IN4 y ULT frente a IN1 | Los nombres suelen estar impresos por debajo: lean VCC y GND en la placa y márquenlos antes de montarla (en la foto 2 las placas están giradas 180°). VCC y GND quedan en medio de las entradas: no conecten los cuatro IN seguidos sin mirar la serigrafía |
| ESP32-CAM, fila del 5V | 5V · GND · IO12 · IO13 · IO15 · IO14 · IO2 · IO4 | IO12 debe quedar libre: si está en alto al encender, la cámara no arranca |
| ESP32-CAM, fila del 3V3 | 3V3 · IO16 · IO0 · GND · VCC · U0R · U0T · GND/R | VCC es una salida, no una entrada. En su placa (foto 2) el último pad dice GND/R: en estos clones puede ir al reset, así que no lo usen como tierra |
| GY-88 | VIN · 3V3 · GND · SCL · SDA · M_DRDY · G_AD0 · G_INT | 3V3 es la salida de su regulador; aparte del pad GND, solo G_AD0 se conecta a tierra |
| VL53L1X «VL53L0/1XV2» (la de la foto) | VIN · GND · SCL · SDA · GPIO1 · XSHUT | La placa «TOF400C-VL53L1X» trae SDA y SCL al revés |

Este orden de pines está tomado de las placas más comunes de cada tipo. Si la serigrafía de la suya dice otra cosa, manda la serigrafía.

### Tabla de conexiones

| Desde | Hacia | Nota |
| --- | --- | --- |
| Potencia de los motores |  |  |
| Batería de motores + | fusible F1 (rápido, ≈ 7,5 A) → interruptor S1 (≥ 8 A) → VCC de los dos DRV8833 | En el módulo, VCC y GND son pines vecinos y no hay protección contra inversión: usen un conector con polaridad fija y midan la polaridad antes de conectar. Cables de 20–22 AWG y cortos entre batería, F1, S1 y drivers; 24 AWG basta para cada motor. F1 abre si hay un corto; un PTC es demasiado lento |
| Batería de motores − | punto común de tierra, junto a los DRV8833 | De ahí sale un cable de tierra a cada módulo; la corriente de los motores no pasa por el cable del S3 |
| C1, 100–220 µF, 6,3 V o más | entre VCC y GND, junto a los drivers | Electrolítico: la franja marca el negativo. Además, cada módulo debe tener su capacitor de 10 µF junto al chip; el módulo común ya lo trae |
| Señales a los drivers |  |  |
| S3 GP13 | DRV8833 #1 · IN1 | motor FL |
| S3 GP12 | DRV8833 #1 · IN2 | motor FL |
| S3 GP11 | DRV8833 #1 · IN3 | motor RL |
| S3 GP10 | DRV8833 #1 · IN4 | motor RL |
| S3 GP9 | DRV8833 #2 · IN1 | motor FR |
| S3 GP8 | DRV8833 #2 · IN2 | motor FR |
| S3 GP7 | DRV8833 #2 · IN3 | motor RR |
| S3 GP14 (pad pequeño) | DRV8833 #2 · IN4 | motor RR |
| EEP y ULT de cada DRV8833 | sin conectar | EEP suele traer un pull-up a VCC (47 kΩ o 33 kΩ): mídanlo con la placa apagada y comprueben por continuidad que EEP va al pin 1 del chip y ULT al pin 8. Si EEP va al pin 1 y no tiene pull-up, únanlo a VCC con un cable; si va al pin 8, los nombres están cambiados y no deben unirlo a VCC. ULT avisa fallas y es opcional |
| Motores |  |  |
| DRV8833 #1 · OUT1 / OUT2 | motor FL (empuja adelante) | Si gira al revés, intercambien sus dos cables |
| DRV8833 #1 · OUT3 / OUT4 | motor RL (empuja arriba) |  |
| DRV8833 #2 · OUT1 / OUT2 | motor FR (empuja adelante) |  |
| DRV8833 #2 · OUT3 / OUT4 | motor RR (empuja arriba) |  |
| Lógica, sensores y cámara |  |  |
| Batería de la lógica + | fusible F2 (rápido, ≈ 3 A) → interruptor S2 (≥ 2 A) → entrada del step-up de 5 V | El step-up no tiene protección contra inversión: midan la polaridad antes de conectar. F2 es obligatorio con el MT3608, que no tiene protección contra cortocircuito |
| Batería de la lógica − y GND del step-up | punto común de tierra |  |
| GND del S3, del GY-88, del VL53L1X y de la cámara | punto común de tierra | En la cámara, usen el GND que está junto al pin 5V, no el último pad de la otra fila (en su placa dice GND/R) |
| Salida del step-up (5 V) | ESP32-CAM 5V | La cámara no se alimenta del S3 |
| C2, 470 µF, 10 V o más | entre los 5 V del step-up y GND, junto a la cámara | Electrolítico: la franja marca el negativo. Evita que la cámara se reinicie en los picos |
| Salida del step-up (5 V) | ánodo de D1; cátodo de D1 (el lado con la raya) → S3 5V | D1 impide que el USB alimente el step-up y la cámara; aun así, no conecten el USB con S2 cerrado |
| S3 3V3 | GY-88 VIN · VL53L1X VIN | Solo los sensores: es salida, nunca entrada |
| GY-88 G_AD0 | GND | Fija la dirección del MPU-6050 en 0x68. Solo si la serigrafía dice G_AD0 (algunos vendedores escriben G_ADO): en placas tipo GY-87 ese pad es INTA, una salida, y no se conecta |
| GY-88 3V3 | sin conectar | Es la salida de su propio regulador |
| S3 GP4 | SDA del GY-88 y del VL53L1X | Bus I²C compartido. Conecten por el nombre impreso: en algunas placas ToF el orden de SDA y SCL cambia |
| S3 GP5 | SCL del GY-88 y del VL53L1X |  |
| S3 GP2 | resistencia de 470 Ω → ESP32-CAM IO13 | El S3 transmite, la cámara recibe |
| S3 GP6 | resistencia de 470 Ω → ESP32-CAM IO14 | La cámara transmite, el S3 recibe. Las resistencias protegen los pines si chocan al arrancar |
| Divisor 100 kΩ + 100 kΩ | batería de motores (después de S1) → GP1 → GND | 4,2 V se leen como 2,1 V; pongan un capacitor de 100 nF de GP1 a GND |
| S3 GP16 (pad pequeño) | VL53L1X XSHUT | Recomendado. Configúrenlo como salida de drenador abierto: escriban 0 para apagar o reiniciar el sensor y pásenlo a entrada para encenderlo. Antes, con la placa alimentada y XSHUT suelto, midan que quede en alto (≈ 2,8–3,3 V); si no, pongan 10 kΩ de XSHUT a 3V3 |
| S3 GP15 (pad pequeño) | VL53L1X INT (pad GPIO1 del sensor) | Opcional: es la interrupción del sensor; se puede dejar sin conectar y leer por I²C |

### Notas para el firmware

- **PWM:** usen los 8 canales LEDC del ESP32-S3, a 20 kHz y 10 bits. Lo más simple y seguro es el decaimiento rápido: IN1 con PWM e IN2 en 0 para avanzar (al revés, para retroceder). Así, ciclo útil 0 = motor apagado, igual que en el firmware actual. El decaimiento lento (un pin en alto y PWM invertido en el otro) es más lineal, pero invierte la lógica: si el pin con PWM queda en 0 mientras el otro está en alto, el motor va a fondo. Si lo usan, configuren la salida invertida (así, con ciclo 0 el pin PWM queda en alto y, con los dos en alto, el motor frena), escriban los dos pines de cada motor seguidos y hagan que el arranque y el paro de emergencia dejen los dos IN iguales. Suban el PWM en rampa, unas decenas de milisegundos, para no disparar la protección de 2 A del DRV8833.
- **Arranque seguro:** las entradas del DRV8833 tienen pull-down interno de 150 kΩ y los pines GP7–GP14 arrancan sin pull-up. Mientras el chip arranca, antes de que el firmware configure los pines, los motores quedan quietos y el blimp solo se hunde despacio. Después, la seguridad depende del firmware: prueben el arranque y el paro de emergencia sin hélices.
- **Batería baja:** GP1 lee la mitad del voltaje de la celda de motores (divisor de 100 kΩ + 100 kΩ). Si el promedio de 1 s, ya multiplicado por 2, baja de ≈ 3,3 V con los motores casi quietos o de ≈ 3,0 V con los motores a fondo (ciclo útil promedio por encima de ≈ 70 %), apaguen los motores y dejen que el blimp baje; no los vuelvan a encender hasta cargar o cambiar la celda.
- **TX (GP43) y RX (GP44) quedan libres:** el TX imprime texto cada vez que el chip arranca y los dos arrancan con pull-up, así que no sirven para motores. La cámara va por UART1, que el ESP32-S3 puede sacar por GP2 y GP6.
- **GY-88:** al arrancar, escaneen el bus buscando la brújula en 0x1E (HMC5883L), 0x0D (QMC5883L) o en 0x2C y 0x30 (otros clones). Si no aparece en ninguna, puede estar detrás del MPU-6050, como en las placas tipo GY-87 y HW-290: activen el modo bypass (0x6B = 0x00, 0x6A = 0x00, 0x37 = 0x02) y vuelvan a escanear. Identifiquen el chip por sus registros de ID, no solo por la dirección.
- **Programar la ESP32-CAM:** no tiene USB. Prográmenla antes de conectarla al blimp, o con S2 abierto y su pin 5V desconectado del step-up: el 5 V del adaptador y el del step-up nunca deben quedar unidos. Usen un adaptador USB-serie con señales TX y RX de 3,3 V. Si tiene un pin de 5 V aparte, úsenlo para el 5V de la cámara; si su puente elige 3,3 o 5 V para las señales y para ese pin a la vez, déjenlo en 3,3 V y alimenten la cámara con una fuente de 5 V externa, nunca con el step-up del blimp, uniendo las tierras. Conecten TX → U0R, RX → U0T, 5 V → 5V y GND → el GND que está junto al 5V (no el último pad de la otra fila, que en su placa dice GND/R). Unan IO0 con GND, enciendan, carguen el programa, quiten el puente de IO0 y reinicien la placa con su botón RST.
- **Si usan Arduino:** el perfil del S3-Zero no define pines I²C y sus pines SPI por defecto no existen en esta placa. Usen siempre los números de GPIO directamente.
- **UART con la cámara:** 115 200–921 600 baudios, con paquetes cortos: encabezado, número de cuadro, marca de tiempo, ángulos horizontal y vertical del globo, diámetro en píxeles, confianza y checksum. En la cámara, RX = IO13 y TX = IO14, y la ranura microSD debe quedar vacía porque usa esos pines. El S3 debe ignorar lo que no tenga encabezado: al arrancar, IO14 puede dar pulsos o bytes sueltos antes de que la cámara configure su UART (el texto de arranque de la cámara sale por U0T, no por IO14). En el código de la cámara pasen los pines 13 y 14 de forma explícita: los pines por defecto de los UART del ESP32 chocan con la PSRAM de la ESP32-CAM.

### Notas de armado

- Trencen cada par de cables de motor y el par de cada batería.
- C1 (100–220 µF) va junto a los drivers, y conviene un capacitor cerámico de 100 nF en los terminales de cada motor para bajar el ruido de las escobillas.
- Bus I²C: cables de 15 cm o menos hasta el GY-88; empiecen a 100 kHz y suban a 400 kHz si el bus está limpio. El GY-88 y el VL53L1X traen sus propias resistencias de pull-up: con las placas apagadas, midan la resistencia entre SDA y 3,3 V con todo conectado; si baja de ≈ 1 kΩ, quiten las resistencias de pull-up de una de las placas; si sube de ≈ 4 kΩ, quédense en 100 kHz.
- En la placa «VL53L0/1XV2», quiten la película amarilla que protege el lente (en la «TOF400C-VL53L1X», la tapa negra es la ventana y se queda). No dejen ninguna pieza de la góndola dentro de un cono de ≈ 40° bajo el sensor: lo que entra en ese cono falsea la altura.
- Con el nombre ToF400C se venden dos placas distintas. La de la foto dice «VL53L0/1XV2» y sus pads van VIN, GND, SCL, SDA, GPIO1, XSHUT. Si la suya dice «TOF400C-VL53L1X» (negra, con tapa ovalada), van VIN, GND, SDA, SCL, INT, SHUT. Conecten cada cable por el nombre impreso, no por la posición.
- No pongan cables, batería ni metal sobre el extremo del S3-Zero opuesto al USB-C: ahí está su antena. Saquen los cables de los pads 14, 15 y 16 hacia el lado del USB-C, no por encima de la antena.
- La corriente de los motores no debe pasar por los contactos de un protoboard: aguantan ≈ 1 A.
- Si los cables a los drivers quedan largos, una resistencia de 47 kΩ de cada entrada IN a GND da más margen para que los motores no se muevan al arrancar.
- En la ESP32-CAM tomen la tierra del pin GND que está junto al 5V. En su placa (foto 2), el último pad de la fila del 3V3, junto a U0T, dice «GND/R»: en estos clones puede ir al reset, y si es así, conectarlo a tierra no deja arrancar la placa. Midan continuidad entre los GND antes de soldar.
- El LED RGB del S3-Zero (GP21) sirve para mostrar estados sin cables extra.
- Si un motor empuja al revés, intercambien sus dos cables en el DRV8833 (o inviertan el sentido en el firmware).

Antes de soldar

## Revisar las placas y encender por primera vez

### Identifiquen sus placas

- **S3-Zero.** Junto a los botones de la placa de la foto 2 dice «B» y «R»: es la revisión actual, con el LED RGB en GP21, y el mapa de la figura 2 vale para ella. Para confirmarlo en la suya, denle vuelta: la revisión actual tiene atrás una sola columna de 8 pads (45, 42, 41, 40, 39, 38, 18 y 17). Si dice BOOT y RESET y tiene dos columnas de pads atrás, es la versión vieja, con el LED en GP10: no usen GP10; conecten IN4 del DRV8833 #1 (motor RL) a GP15 y dejen sin conectar el INT del ToF, que se puede leer por I²C.
- **DRV8833.** Si los nombres de los pines no se ven por encima, están impresos por debajo: denle vuelta a la placa, lean VCC y GND y márquenlos antes de montarla, porque con la polaridad invertida se dañan el chip y su capacitor. Con la placa apagada, midan que EEP tenga su pull-up a VCC (≈ 47 kΩ o 33 kΩ) y, en modo continuidad, que EEP llegue al pin 1 del chip (el del punto) y ULT al pin 8. Lean también lo que está impreso en el chip: debe decir «DRV8833» (un «8833PW» solo aguanta 0,5 A). Y revisen que no tenga resistencias de 0,15 Ω entre los pines de sensado del chip (3 y 6) y GND: limitarían cada motor a ≈ 1,3 A y sumarían ≈ 0,2 V de caída (≈ 9–10 gf por motor).
- **GY-88.** El GY-88 tiene una sola fila de 8 pads: VIN, 3V3, GND, SCL, SDA, M_DRDY, G_AD0, G_INT (algunos vendedores escriben G_ADO, con letra O). Aparte del pad GND, solo el pad G_AD0, el séptimo contando desde VIN, se conecta a tierra. Si su placa dice FSYNC, INTA y DRDY, es una GY-87: ese pad es INTA, una salida, y no se conecta.
- **VL53L1X.** La foto 2 muestra la placa «VL53L0/1XV2». Al encenderla, lean el registro 0x010F: un VL53L1X real responde 0xEACC.
- **Celda de los motores.** La etiqueta de las HJ 702035P de la foto 1 solo dice «3.7V 380mAh»: no da la tasa de descarga ni dice si traen placa de protección. Sin la ficha del fabricante, a esas celdas no les pidan ≈ 6 A (≈ 16 C): las celdas de 380 mAh que sí traen ficha, como las de Electrokit y EEMB, son de 2 C (≈ 0,76 A). Para los motores consigan una celda de 380 mAh de 20 C o más con ficha, cables de 20–22 AWG en lugar de los finos de fábrica, y un conector e interruptor S1 de 8 A o más.

### Primer encendido, en este orden

1. Con las dos celdas desconectadas, midan con el multímetro la polaridad de cada celda en su conector y márquenla. En modo continuidad, comprueben que no haya corto entre VCC y GND de cada DRV8833, entre 5V y GND (en la salida del step-up, en la cámara y en el S3) ni entre 3V3 y GND del S3-Zero.
2. Prueben el step-up solo, sin nada conectado a su salida. La primera vez, aliméntenlo con la fuente de laboratorio a 3,7 V y con el límite de corriente en ≈ 1 A (con menos, el arranque puede bajar el voltaje y parecer una falla del módulo); si no tienen, usen la celda de la lógica y S2. Midan 5,0 V a la salida (el MT3608 se ajusta con su trimmer; el Pololu da 5 V fijos con SEL unido a VIN). Apaguen la fuente (o abran S2), y solo entonces conecten la salida del step-up al resto.
3. Preparen un programa de prueba con los pines de la tabla de conexiones. No carguen todavía el firmware del repositorio: board_config.h y el driver de motores tienen los pines de la placa ESP-FLY (motores en GPIO7, 4, 3 y 1; I²C en GPIO5 y GPIO6; batería en GPIO2), que en este cableado son otras señales. Con ese firmware el escaneo no encuentra nada y «FL» movería el motor RR.
4. Con S1 y S2 abiertos, conecten solo el USB del S3-Zero, carguen el programa de prueba y escaneen el I²C (SDA en GP4, SCL en GP5). Deben aparecer 0x29 (ToF), 0x68 (MPU-6050), 0x77 (barómetro) y la brújula en 0x1E o 0x0D, o en 0x2C o 0x30 en otros clones. Si la brújula no aparece, sigan la nota del GY-88 en «Notas para el firmware». En esta prueba no inicien el UART1 (GP2 y GP6): la cámara está apagada y recibiría corriente por sus pines de datos.
5. Desconecten el USB y cierren S2. Midan ≈ 5,0 V en el pin 5V de la cámara, ≈ 4,6–4,7 V en el pad 5V del S3 y 3,3 V en su pad 3V3, con la punta negra sujeta con un caimán: el 5V y el GND de la cámara son pines vecinos, y un resbalón los pone en corto. Después de dos minutos, el regulador del S3 y el de la cámara deben estar tibios, no calientes.
6. Primero sin hélices: cierren S1 y hagan girar cada motor solo, al 20 % durante unos 2 s, en el orden FL, FR, RL y RR. Cada nombre debe mover el motor que indica la figura 1, sin ruidos raros, y ningún DRV8833 debe calentarse. Si tienen la fuente de laboratorio, esta primera vez alimenten los drivers con ella (4,0 V, límite de 1,5 A) en lugar de la celda.
7. Después, con las hélices puestas, el blimp sujeto y las manos y los cables lejos de las hélices, repitan cada motor al 20 % con una tira de papel delante: FL y FR deben soplar hacia atrás, y RL y RR hacia abajo. Si uno sopla al revés, abran S1 e intercambien los dos cables de ese motor.
8. Antes de volar, adapten el firmware del repositorio a este cableado: los pines nuevos en board_config.h y un driver de motores con dos entradas por motor, porque la ESP-FLY usa un MOSFET por motor en vez de puentes H. Si usan su modo de prueba de motores, al terminar dejen BLIMP_BOOT_MOTOR_ID_MODE en 0.

Alimentación

## Dos celdas y un step-up de 5 V

El MB102 no sirve con celdas 1S

Sus reguladores AMS1117 son lineales y necesitan ≈ 1,1–1,3 V por encima de la salida. Además, según el esquemático del módulo YwRobot que copian los clones, el de 3,3 V toma su entrada de la salida del de 5 V, y la entrada tiene un diodo 1N4007 que resta otros 0,8 V (confirmen con un multímetro que su clon es igual). Con una celda de 3,0–4,2 V ninguna salida regula: por el conector del adaptador, el riel de 3,3 V queda en ≈ 1 V con la celda llena y cerca de 0 V con la celda baja. Con dos celdas en serie solo regula por encima de ≈ 6,5–7 V y convierte más de la mitad de la energía en calor. Déjenlo para el banco de pruebas, con un adaptador de 7–9 V y poca carga: a 12 V y 0,3 A, su regulador de 5 V disiparía ≈ 1,9 W y se apagaría por temperatura.

### Cómo queda en la figura 1

- **Celda 1 → motores.** Va directo al VCC de los dos DRV8833, que aceptan 2,7–10,8 V, con C1 de 100–220 µF junto a los drivers.
- **Celda 2 → lógica.** Pasa por un convertidor step-up (elevador de voltaje) de 5 V. Esos 5 V alimentan la ESP32-CAM por su pin 5V y, a través del diodo Schottky D1, el pin 5V del S3-Zero.
- **3,3 V solo para los sensores.** El pin 3V3 del S3-Zero es la salida de su regulador ME6217, que viene en un encapsulado diminuto y se calienta rápido. Alcanza para el GY-88 y el VL53L1X (≈ 25 mA), no para la cámara: el mismo regulador ya alimenta al S3, así que no le cuelguen más carga.
- **Una sola tierra.** Las dos baterías, el step-up, los drivers y los módulos comparten GND, unidos en un punto. El S3-Zero tiene un solo pad GND, así que la corriente de los motores no debe pasar por él.
- Separar motores y lógica evita que los picos de los motores hagan caer el voltaje del ESP32 y lo reinicien (brownout), la falla típica de los montajes con una sola batería.

### Por qué el step-up y el diodo

- La ESP32-CAM trae un regulador AMS1117 en su pin 5V que necesita ≈ 4,5 V como mínimo (AI-Thinker especifica 5 V); una celda 1S no alcanza. Y su pin 3V3 no aguanta la celda directa: 4,2 V superan el máximo absoluto de 3,6 V del ESP32.
- El pin 5V del S3-Zero está unido directo al USB-C, sin diodo ni fusible. Si el step-up alimentara ese pin sin diodo y alguien enchufara el USB, los 5 V del computador y los del step-up quedarían unidos. D1 impide que el USB alimente el step-up y la cámara, pero no que el step-up le devuelva algo de corriente al puerto: no conecten el USB con S2 cerrado.
- Nunca alimenten el S3-Zero por el pin 3V3: el regulador se daña si su salida queda por encima de su entrada.
- Sirve un convertidor step-up de 5 V que entregue al menos 0,8 A con 3,0 V de entrada. Opciones: un Pololu S9V11F3S5C3 o S9V11F3S5 con su pin SEL unido a VIN (0,7 g; según la gráfica de Pololu da ≈ 0,9 A con 3,0 V, necesita 3 V para arrancar y trae protección contra cortocircuito; la versión C3 además corta a 3 V y protege la celda), o un módulo MT3608 ajustado a 5,0 V con su trimmer (más barato, pero sin gráfica ni peso publicados y sin protección contra cortocircuito: midan la salida antes de conectar nada, pruébenlo con carga, miren su temperatura y no lo usen sin el fusible F2). El Pololu U1V11F5 se queda corto: entrega ≈ 0,5–0,8 A. D1 puede ser un SS14 o un 1N5817; con D1, el S3-Zero recibe ≈ 4,6–4,7 V, dentro de su rango de 3,7–6 V.

### Consumo aproximado

| Carga | Batería | Típico | Picos |
| --- | --- | --- | --- |
| Cuatro motores a fondo | motores | — | ≈ 6 A |
| ESP32-S3 con ESP-NOW, más GY-88 y VL53L1X | lógica | ≈ 150 mA | ≈ 400 mA |
| ESP32-CAM procesando, sin Wi-Fi (estimado) | lógica | ≈ 100–150 mA | ≈ 300 mA |
| ESP32-CAM con el flash apagado (dato de AI-Thinker, que no dice con qué carga; con video por Wi-Fi se han medido 100–250 mA) | lógica | ≈ 180 mA | 300–500 mA |
| LED de flash de la cámara (GPIO4), si se enciende | lógica | + 130 mA | — |

Corrientes de la lógica calculadas con hojas de datos, en su riel de 3,3 V; no son mediciones, así que midan el consumo real con todo encendido. La fila del S3 supone ESP-NOW escuchando todo el tiempo. La lógica le pide ≈ 0,45–0,65 A a su celda según la carga (1,2–1,7 C: son 5 V × 0,33 A divididos por la eficiencia del step-up), con picos de 1–2 A. Con 380 mAh dura más de media hora, pero una celda estándar de 2 C queda justa: la de la lógica también conviene que sea de alta descarga, o pongan un capacitor de 100 µF o más en la entrada del step-up, y no enciendan el flash. Los motores son el problema: ≈ 6 A son ≈ 16 C para una celda de 380 mAh. Las celdas estándar de ese tamaño son de 2 C (≈ 0,76 A) y con 6 A caerían a ≈ 2–3,4 V (estimado con 0,15–0,30 Ω de resistencia interna): por debajo de 2,75–3,0 V la celda se daña, y a ≈ 2,5 V el DRV8833 se bloquea. La etiqueta de las HJ 702035P no dice su tasa de descarga: para los motores hace falta una celda de 20 C o más con ficha del fabricante, y un conector e interruptor de 8 A o más; el JST-PH blanco típico aguanta 2 A.

Protección de las celdas

- Ninguna de las dos celdas tiene corte propio en este diseño: el DRV8833 apaga a ≈ 2,5 V y el step-up sigue funcionando hasta vaciar la celda. Revisen si cada celda trae placa de protección y lean su chip: un DW01, el más común, solo corta a ≈ 2,4 V, por debajo del límite de daño, y puede disparar por sobrecorriente a ≈ 3 A según sus MOSFET, o sea que podría cortar los motores a fondo.
- **Celda de motores.** El corte por batería baja lo hace el firmware (ver «Batería baja» en las notas para el firmware). Pongan un fusible rápido de ≈ 7,5 A (F1) en el + de la celda, lo más cerca posible de ella y antes de S1. Un corto da ≈ 10–25 A según la resistencia interna de la celda y los cables: elijan un fusible que, según su curva, abra en menos de un segundo con 20 A y en pocos segundos con 10 A, y dejen corto y firme el tramo batería–F1–S1. F1 trabaja cerca de su límite (6 A de 7,5 A): si se quema en vuelo sin que haya un corto, no pongan uno más grande a ciegas; midan la corriente real de los motores a fondo. Un PTC no sirve: uno de 7 A tarda hasta 17,5 s en abrir con 35 A, y a 40 °C solo sostiene 5,8 A, menos que los ≈ 6 A de los motores a fondo.
- **Celda de la lógica.** El firmware no puede cortarla, porque el S3 y la cámara cuelgan de ella a través del step-up, que sigue dando 5 V hasta que la entrada baja a ≈ 2 V. Usen el Pololu S9V11F3S5C3, igual al S9V11F3S5 pero con corte fijo a 3 V (no vuelve a arrancar hasta 3,4 V), o una celda con protección que corte a 2,75 V o más, y abran S2 al aterrizar. Ese corte se mide en la entrada y con carga: con picos de 1–2 A la celda puede bajar de 3 V aunque en reposo marque 3,4–3,6 V, y el S3 y la cámara se reiniciarían en pleno vuelo, así que cambien la celda antes de llegar ahí. Si además quieren medirla, GP3 es la única entrada ADC1 libre: necesita su propio divisor de 100 kΩ + 100 kΩ con 100 nF, después de S2 (GP3 es pin de arranque, pero con los eFuses de fábrica el chip lo ignora).
- **Cortos en la lógica.** Pongan un fusible rápido de ≈ 3 A (F2) en el + de la celda de la lógica, antes de S2. Con el MT3608 es obligatorio: un step-up básico no tiene protección contra cortocircuito, y un corto entre 5V y GND pone la celda en corto a través de su bobina y su diodo. El Pololu sí la tiene.

Distribución

## Dónde va el magnetómetro

La regla de 10 cm entre el magnetómetro y los motores o drivers es un buen mínimo: ArduPilot pide al menos 10 cm entre la brújula y los cables de corriente continua y las baterías, y recomienda montarla en un mástil. Con 15–20 cm habría más margen (criterio nuestro, no una norma). Tu protoboard no cumple los 10 cm por poco. Medí la foto del protoboard usando el paso de 2,54 mm entre filas. El chip del magnetómetro (en el GY-88, fila ≈ 59) queda a **≈ 9,6 cm** del chip del DRV8833 #2 (fila ≈ 21,5) y a ≈ 8,8 cm del borde de esa placa. Le faltan ≈ 4 mm, y la posición sale de una foto con ±2,5 mm de incertidumbre. Con los dos drivers lado a lado en la placa final, el magnetómetro queda a ≈ 12 cm del más cercano (panel derecho de la figura 3).

Hay dos tipos de perturbación. Los imanes de los motores dan del orden de 5–11 µT a 10 cm (estimado, sin medir), pero no se mueven respecto al sensor, así que la calibración los quita. Lo que cambia con el acelerador o con la carga de la cámara es el campo de las corrientes: el de los cables se controla trenzándolos, y el de las placas, con distancia. La prueba final es en el banco: con todo montado, la lectura no debería moverse más de ≈ 0,5 µT (≈ 1° de rumbo) al prender los motores ni al encender y apagar la cámara.

Figura 3 · Tu protoboard a escala, y la propuesta Posiciones tomadas de la foto (filas cada 2,54 mm, ±1 fila de incertidumbre). Lo sombreado está a menos de 10 cm del chip de algún DRV8833. El chip del magnetómetro, en verde, queda apenas dentro del círculo del DRV8833 #2. A la derecha, las mismas piezas con los dos drivers lado a lado en el extremo de potencia: el magnetómetro queda a ≈ 11,9 cm, fuera de los dos círculos. Los motores también deben quedar a 10 cm o más. En los dos paneles el chip del magnetómetro queda a ≈ 1,3 cm del borde de la ESP32-CAM (ver «Qué perturba la brújula»).

### Para ganar margen

- En la placa final, pongan los dos DRV8833 uno al lado del otro, como en el panel derecho de la figura 3. En el protoboard no se puede porque cada uno necesita el canal central.
- O alejen el GY-88 2–3 cm más, con una extensión corta de la placa o una varilla de carbono; así también queda más lejos de la cámara.
- Midan hasta el chip del magnetómetro, no hasta el borde del GY-88.
- Los cuatro motores también cuentan: en la góndola tienen que quedar a ≥ 10 cm de ese chip.

### Qué perturba la brújula

- **La ESP32-CAM y el step-up.** Su corriente cambia con la carga (≈ 0,1–0,5 A, y 130 mA más si se enciende el flash), y la calibración no quita lo que cambia. En la figura 3 el chip del GY-88 queda a ≈ 1,3 cm del borde de la ESP32-CAM; con los 2–3 cm extra de «Para ganar margen» queda a ≈ 3–4 cm. Si en la prueba de banco la lectura se mueve más de ≈ 0,5 µT al encender la cámara, aléjenlo más. Trencen también el par de 5 V de la cámara y pongan el step-up lejos del GY-88.
- **Los cables pesan más que la distancia.** Con los ≈ 6 A de los cuatro motores, un par trenzado (ida y vuelta juntos) genera ≈ 0,2 µT a 10 cm; un cable suelto con la misma corriente, ≈ 12 µT. En Bogotá el campo horizontal es de 26,8 µT, así que 0,5 µT ya mueven el rumbo ≈ 1°. Trencen cada par de motor y el de cada batería, y llévenlos por el lado contrario al GY-88.
- Piezas de acero cerca del sensor: la ranura de la tarjeta SD y el blindaje de la ESP32-CAM, el conector USB-C, tornillos, las pestañas de níquel de las baterías. Como no se mueven, la calibración las corrige, pero solo si se calibra con todo montado en su sitio.
- **El coliseo.** El acero de la estructura y las varillas del piso desvían el campo, y el error cambia con la posición y la altura: en un edificio de concreto y acero se midieron 45° de error promedio. Antes de confiar en el rumbo magnético, recorran la cancha a 2 m de altura con el sensor y miren si la intensidad (≈ 29,9 µT) y la inclinación (≈ 26,5°) se mantienen.

Segunda cámara

## ¿Dos ESP32-CAM para visión estéreo?

Recomendación

No monten visión estéreo con dos cámaras. Usen una sola ESP32-CAM al frente, que mida la distancia por el tamaño del globo después de calibrarla con un globo real en el tiempo de práctica. Para ampliar el campo de visión, un lente gran angular pesa y gasta menos que una segunda cámara, pero reduce a la mitad la precisión de la distancia por tamaño.

La razón es que el objetivo tiene tamaño conocido. Los globos de la competencia son esferas de mylar rojo de 22 pulgadas, de 50 a 60 cm de diámetro (Regla 21). Con una sola cámara, la distancia sale del tamaño aparente del globo: Z = f · D / w, donde D es el diámetro real y w lo que mide en píxeles. A 4 m un globo de 55 cm ocupa unos 42 píxeles de ancho en QVGA (unos 34 si el lente ve 66° en horizontal), así que equivocarse en 1 píxel es un error de ≈ 2,4 % (≈ 3 %).

Una cámara, por el tamaño del globo (±1 a ±2 px) Dos ESP32-CAM, 15 cm, sin sincronía Estéreo ideal, 15 cm de línea base Estéreo ideal, 30 cm de línea base

Figura 4 · Error de distancia al globo Cámara OV2640 en QVGA (320 × 240), f ≈ 308 px (lente de 66° en diagonal; si ve 66° en horizontal, f ≈ 246 px y los errores de una cámara y del estéreo ideal suben × 1,25). «Estéreo ideal» supone disparidad con ±0,5 px de error y las dos fotos tomadas en el mismo instante. «Sin sincronía» suma el desfase entre dos placas independientes: 33 ms a 10 °/s, o ±11 ms a 29 °/s. En los dos casos la imagen se desplaza casi igual, ≈ 1,7–1,8 px. Esa curva usa la aproximación lineal, que da el mismo error hacia los dos lados. El error real es asimétrico: a 10 m la distancia puede salir 6,6 m mayor o 2,9 m menor, según el signo del desfase, frente a ±4,0 m en la curva.

Ver los valores en tabla

| Distancia | 1 cámara ±1 px | 1 cámara ±2 px | Estéreo ideal 30 cm | Estéreo ideal 15 cm | 2 ESP32-CAM reales |
| --- | --- | --- | --- | --- | --- |
| 2 m | 0,02 m | 0,05 m | 0,02 m | 0,04 m | 0,16 m |
| 4 m | 0,09 m | 0,19 m | 0,09 m | 0,17 m | 0,64 m |
| 6 m | 0,21 m | 0,43 m | 0,19 m | 0,39 m | 1,44 m |
| 8 m | 0,38 m | 0,76 m | 0,35 m | 0,69 m | 2,55 m |
| 10 m | 0,59 m | 1,18 m | 0,54 m | 1,08 m | 3,99 m |

### Por qué no compensa

- **No mide mejor.** Con la línea base que cabe en la góndola (10–15 cm), el estéreo ideal tiene un error parecido o mayor que el de una sola cámara calibrada. Con 30 cm apenas la iguala, y necesita una barra rígida: 0,1° de torsión entre las dos cámaras ya agrega 0,5 px de error. La envolvente no sirve de soporte porque se deforma, y una barra larga y rígida con cámaras en los extremos obliga a revisar la Regla 17, que pide diseñar las superficies externas para minimizar el riesgo de reventar los globos.
- **Las dos placas no se pueden sincronizar.** En la ESP32-CAM el pin PWDN del sensor va a tierra, el RESET no llega a ningún GPIO y no hay pin de disparo, así que cada placa captura a su ritmo (≈ 44 cuadros/s en QVGA). Aun emparejando el cuadro más cercano quedan ±11 ms de desfase. Si el blimp gira a 29 °/s (la velocidad de búsqueda del controlador), esos ±11 ms desplazan la imagen ≈ 1,7 px, y la distancia sale entre un 16 % menor y un 23 % mayor a 5 m con 15 cm de línea base, según qué placa capture primero y hacia dónde gire el blimp.
- **No hay dónde calcularlo a bordo.** El ESP32 de la cámara no alcanza para estéreo denso (≈ 61 millones de operaciones por cuadro QVGA). Un portátil lo hace en milisegundos, y la Regla 1 permite calcular fuera del blimp (la Regla 15 solo exige que los sensores vayan a bordo), pero habría que mandarle dos videos por Wi-Fi con 0,1–0,3 s de retraso, en la misma banda que el enlace de control.
- **Suma peso y gasta energía.** Otra ESP32-CAM pesa 7–10 g más la barra, consume ≈ 180 mA a 5 V y ocupa otro UART del S3-Zero.
- **Los que ya lo resolvieron midieron por tamaño.** El equipo de Bonn, que reventó los cinco globos en menos de dos minutos de vuelo en las tres pruebas de MBZIRC 2020 (quedó 5.º en el reto 1 y 2.º en el Gran Reto), llevaba una cámara estéreo RealSense D415 y aun así calculó la distancia con el radio conocido del globo, porque la profundidad salía ruidosa. El blimp de referencia del reglamento (MochiSwarm) también usa una sola cámara.

### Mejor uso de lo que tienen

- **Una sola ESP32-CAM al frente, procesando a bordo.** Que busque el rojo en QVGA, calcule centro y diámetro del globo, y mande por UART ángulo, tamaño, confianza y la marca de tiempo del cuadro a 10–20 Hz. Sin Wi-Fi gasta menos y no depende de la red. La detección de color de Espressif (ESP-WHO) no corre en este ESP32, así que es código propio; hay que medir cuántos cuadros por segundo logra.
- **Calibren la distancia en el tiempo de práctica (Regla 23).** Con un globo real a 2, 3 y 4 m se obtiene K = f · D, y la distancia sale como Z = K / w. Así el error baja a ≈ 2–4 % entre 3 y 5 m si el borde del globo se mide con ±1 px (≈ 3,5–7 % con ±2 px; el rango depende del lente) y si el globo de la calibración mide lo mismo que los de la competencia: cada centímetro de diferencia en el diámetro suma ≈ 1,8 % (sin calibrar, el tamaño de los globos varía ≈ ±9 %), y el controlador solo necesita una distancia aproximada: el toque se confirma por contacto.
- **Para ver más, un lente gran angular.** El simulador supone una cámara de 90 × 70°. El lente estándar ve entre 55 × 43° y 66 × 52°, según el módulo, porque los vendedores no dicen si sus 66° son en diagonal o en horizontal. Mídanlo con una regla: a 1 m de distancia, si la imagen cubre ≈ 1,04 m de ancho, el campo es de ≈ 55° en horizontal; si cubre ≈ 1,30 m, es de ≈ 66°. Un módulo OV2640 de 120° (≈ 96–108° en horizontal, según la lente) se cambia en el mismo conector, pero no sale gratis: la distancia focal baja a la mitad (≈ 115–145 px en QVGA en vez de ≈ 245–310 px), así que el globo se ve ≈ 2 veces más pequeño y solo supera los 10 px hasta ≈ 6,5–8 m, y el error de distancia por tamaño se duplica (≈ 4–8 % entre 3 y 5 m con ±1 px). Además, estos lentes suelen tener distorsión de barril, que cambia el tamaño aparente del globo según dónde caiga en la imagen. Si lo usan, midan su campo con la regla, calibren K con el globo en varios puntos de la imagen y corrijan la distorsión con un tablero de ajedrez. Si lo que más importa es la precisión de la distancia, usen el lente estándar y amplíen el campo con una segunda cámara girada.
- **Si usan la segunda ESP32-CAM, que sea para ampliar el campo de visión,** no para estéreo: dos cámaras giradas ±27° ven ≈ 109° de ancho.
- **Flujo óptico mirando al piso** (PMW3901, ≈ 1 g, SPI) daría la velocidad sobre el piso para frenar a tiempo. Es opcional y hay que probarlo antes en la cancha: necesita textura en el piso, más de ≈ 60 lux y que el piso no brille.

Receta para detectar un globo de mylar rojo

1. Fijen ganancia, exposición y balance de blancos después de calentar la cámara, y pongan el filtro de parpadeo en 60 Hz (la red eléctrica de Colombia).
2. Filtren por tono: el rojo está en los dos extremos (0–10 y 170–179 en la escala de OpenCV), con saturación mínima ≈ 90 y brillo mínimo ≈ 40.
3. Los reflejos del mylar salen blancos y abren huecos en la mancha: ciérrenlos con una operación morfológica de tamaño proporcional al globo.
4. Quédense con la mancha más grande, ajústenle un círculo y descarten las que no se parecen a un círculo.
5. Suavicen entre cuadros y prueben con fotos reales del coliseo, a varios ángulos y con la iluminación de la competencia.

Pruebas de empuje

## Qué dicen las tres pruebas

| Prueba | Montaje en la foto | Voltaje | Corriente | Potencia | Empuje | gf/W |
| --- | --- | --- | --- | --- | --- | --- |
| 1 · «ineficiente», 95 % | motor sin voltear, hélice arriba (gira al revés) | 3,97 V | 1,245 A | 4,94 W | 10,9 gf | 2,21 |
| 2 · «eficiente», 95 % | motor volteado, hélice abajo (sentido normal) | 3,98 V | 1,445 A | 5,75 W | 14,7 gf | 2,56 |
| 3 · 100 % | igual que la 2 | 4,13 V | 1,561 A | 6,45 W | 16,8 gf | 2,61 |

Cómo leí las fotos: en cada una gira solo el motor delantero izquierdo (las otras hélices se ven nítidas) y la báscula marca positivo, o sea que el aire sale hacia arriba. En la prueba 1 el motor está sin voltear, con la hélice arriba; en la 2 y la 3 está volteado, con la hélice abajo. Para que el aire suba en los dos montajes, en la 1 la hélice tiene que girar al revés. Si «ineficiente» quería decir otra cosa, por ejemplo el modo de decaimiento del DRV8833, avísenme y lo corrijo. Los rótulos «95 %» y «100 %» llegaron con las fotos, y en ellas no se ve ningún driver entre la fuente y el motor. El 95 % de 4,13 V son 3,92 V, cerca de los 3,98 V medidos, así que el «95 %» podría ser un PWM (y entonces el voltaje medio en el motor fue menor que el que marca la fuente) o solo otro ajuste de la fuente; por eso la pregunta del final.

### Lo que significa

- **La reversa da ≈ 74 %** del empuje hacia adelante (10,9 frente a 14,7 gf). Alcanza para frenar antes de salirse de la zona de competencia (Regla 7), que es la principal razón para usar puentes H.
- **La Regla 13 no es el límite.** Cuatro motores a fondo darían ≈ 67 gf si todos rinden como el medido y van directo, y ≈ 24–48 gf a través del DRV8833 (6–12 gf por motor), según la carga y la tasa de descarga de la celda. El problema es el contrario: el empuje alcanza justo.
- **El DRV8833 cuesta empuje y se calienta.** Pierde ≈ 0,6 V a 1,45 A: ≈ 12 gf por motor a fondo con la celda llena y de baja resistencia, y ≈ 8–9 gf cuando la celda baja a 3,7 V (cálculo sin medir: supone que el empuje crece con el cuadrado del voltaje que llega al motor. Las pruebas 2 y 3 sugieren que crece más rápido, pero no son comparables, porque una fue rotulada «95 %»; si fuera así, quedarían ≈ 10 gf con la celda llena y ≈ 6 gf cerca de 3,7 V. Solo la curva completa lo decide). Con los dos canales del mismo chip a esa corriente se apaga por temperatura en ≈ 1 minuto; ≈ 1,2 A por canal es lo que aguanta de forma continua. Por eso en la figura 1 cada driver lleva un motor de avance y uno de altura, que casi nunca van a fondo a la vez.

### Lo que falta medir

- **Repetir la prueba a través del DRV8833.** Los caimanes de la fuente parecen ir directo a los cables del motor. El driver tiene ≈ 0,4 Ω entre sus dos transistores; a 1,45 A pierde ≈ 0,6 V, así que, según el cálculo, hay que esperar ≈ 12 gf por motor con la celda llena y ≈ 8–9 gf hacia 3,7 V, no 14,7–16,8. Midan el voltaje en los terminales del motor y no en la fuente: los cables largos con caimanes restan algo.
- La curva completa: empuje y corriente al 10, 20 … 100 % de PWM, en los dos sentidos. El controlador necesita la curva, no solo el máximo.
- Con la batería real: una celda de 380 mAh baja de voltaje bajo carga, y el empuje baja con ella.

### El simulador con el empuje medido y estimado

El código del controlador supone 35 gf por motor, un valor provisional. Corrí el simulador del repositorio cambiando solo el empuje máximo por motor en la escena: 16,8 gf es lo medido, y los valores menores son estimaciones de lo que llega a través del DRV8833 según el estado de la celda. El simulador le pasa ese máximo al mezclador del controlador, pero las ganancias y los límites de avance y de giro siguen calculados para 35 gf. Todo lo demás quedó igual: el controlador actual (sin reversa), el sensor ToF, cinco semillas de ruido, --max-time 220 y envolvente de 0,78 m.

| Empuje máximo por motor | Recorrido 1 | Mejor vuelta | Recorrido 2 | Mejor vuelta |
| --- | --- | --- | --- | --- |
| 35 gf (valor provisional del código) | 15 / 15 | 22,4–27,0 s | 15 / 15 | 22,3–27,1 s |
| 16,8 gf (medido con la fuente a 4,13 V) | 15 / 15 | 23,6–28,2 s | 15 / 15 | 26,3–32,3 s |
| 13 gf (DRV8833 con la celda llena, si no baja de voltaje bajo carga: caso más favorable) | 12 / 15 | 26,8–34,1 s\* | 14 / 15 | 26,8–29,4 s |
| 11 gf (DRV8833 con la celda llena, con algo de resistencia interna) | 12 / 15 | 27,6–40,2 s | 11 / 15 | 28,1–28,9 s† |
| 9 gf (DRV8833 con la celda hacia 3,7 V, estimado) | 9 / 15 | 28,9–31,1 s‡ | 12 / 15 | 29,2–46,4 s |
| 7 gf (celda baja o de baja descarga, estimado) | 0 / 15 | — | 8 / 15 | 34,5–42,4 s§ |

Vueltas completas de 15 posibles por recorrido (5 semillas × 3 vueltas) y rango de la mejor vuelta entre semillas. * Una semilla solo completó una vuelta, de 113 s. † Una semilla no completó ninguna vuelta; el rango es de las otras cuatro. ‡ Dos semillas tuvieron mejores vueltas de 81 y 115 s; el rango es de las otras tres. § Una semilla no completó ninguna vuelta. Ninguna corrida activó el modo de falla.

Con 16,8 gf el blimp completa todas las vueltas; la mejor vuelta de cada semilla cambia entre −0,2 y +9,7 s respecto de 35 gf (en promedio +1,9 s en el recorrido 1 y +5,8 s en el 2). Ya con 13 gf quedan sin completar 4 de 30 vueltas; con 11 gf son 7, con 9 gf son 9 y con 7 gf son 22 (con 7 gf no termina ninguna en el recorrido 1). Es decir, incluso con la celda llena (≈ 11–13 gf) se pierden vueltas: el controlador está ajustado para 35 gf y se satura. Con el DRV8833 y una celda de 380 mAh es probable volar buena parte del tiempo en 7–10 gf. Hay que medir el empuje real, recuperar empuje en el hardware y volver a ajustar el controlador.

Cómo recuperar empuje con lo que tienen

Usen un DRV8833 en modo paralelo para cada motor de avance y un MOSFET para cada motor de altura. Esto cambia la figura 1 y la tabla: primero midan el empuje real a través del driver y revisen el esquema con el equipo antes de soldar.

1. Comprueben cada DRV8833 antes de unir nada. Sin motores, con VCC a 3–4,2 V (mejor de la fuente de laboratorio, con límite de ≈ 0,3 A) y GND conectado, midan contra GND que EEP esté en alto (≈ VCC); si no, únanlo a VCC solo si ya comprobaron que va al pin 1 del chip. Con la fuente apagada cada vez que muevan el cable (IN3 queda pegado a GND), unan con un cable aislado el pin VCC con IN1 y con IN3, con IN2 e IN4 sueltos: midiendo contra GND, OUT1 y OUT3 deben dar ≈ VCC, y OUT2 y OUT4 ≈ 0 V. Pasen el cable a IN2 e IN4: debe pasar lo contrario. Si algo no coincide, no usen esa placa en paralelo.
2. Unan IN1 con IN3, IN2 con IN4, OUT1 con OUT3 y OUT2 con OUT4, con cable aislado: los puentes de entrada pasan por encima de GND y VCC, y los dos de salida se cruzan. Así la resistencia del driver baja a la mitad (≈ 0,3 V de caída a 1,45 A) y el chip aguanta el doble de corriente.
3. Conecten el motor de avance (FL en el #1, FR en el #2) entre OUT1 + OUT3 y OUT2 + OUT4, y quiten de OUT3 y OUT4 el motor de altura: si queda ahí, gira junto con el de avance.
4. Usen un GPIO por cada par de entradas: GP13 a IN1 + IN3 y GP12 a IN2 + IN4 en el #1, y GP9 a IN1 + IN3 y GP8 a IN2 + IN4 en el #2. Desconecten de los drivers los cables de GP11, GP10, GP7 y GP14; GP11 y GP7 pasan a manejar los MOSFET.
5. Los motores de altura no necesitan reversa, porque el blimp vuela pesado y baja solo: cada uno va con un MOSFET de canal N con resistencia garantizada a 2,5 V de compuerta y 3 A o más (el SI2300 de la placa ESP-FLY o un AO3400A), que pierde centésimas de voltio. En el encapsulado SOT-23 de los dos, el pin 1 es la compuerta, el 2 la fuente y el 3 el drenador. Conexiones: GPIO (GP11 para RL, GP7 para RR) → 100 Ω → compuerta; compuerta → 10–100 kΩ → fuente, para que arranque apagado; fuente → punto común de tierra junto a los drivers, con su propio cable; drenador → un cable del motor, y el otro cable del motor → + de la batería de motores, después de S1. En paralelo con el motor va un diodo Schottky de 2 A y 30 V o más (SS34 o 1N5822), con la raya hacia el + de la batería.
6. Si cambian fuente y drenador, el diodo interno del MOSFET deja el motor girando siempre; si invierten el Schottky, la batería queda en corto cada vez que el motor se enciende. Antes de conectar el motor, con el multímetro en modo diodo: punta roja en el drenador y negra en el + de la batería, el Schottky debe marcar ≈ 0,2–0,4 V, y con las puntas al revés, nada. Después, con S1 cerrado y sin PWM, cada motor de altura debe quedar quieto (compruébenlo sin hélice); si gira solo, abran S1: fuente y drenador están cambiados. La primera vez que le den PWM, alimenten el circuito con la fuente de laboratorio a 4,0 V y límite de 1,5 A.

Peso

## Los 61 g de la báscula

En la foto de la báscula veo dos baterías HJ 702035P, el MB102, el S3-Zero, los dos DRV8833, la ESP32-CAM, el VL53L1X, una tira de pines y lo que parecen los cuatro motores con hélices. No veo el GY-88. Con masas típicas de cada pieza, la cuenta cuadra:

| Pieza | Masa típica | En la propuesta |
| --- | --- | --- |
| 2 baterías HJ 702035P, 380 mAh | ≈ 8 g c/u | la de la lógica se queda; la de los motores, solo si es de 20 C o más (la etiqueta no lo dice) |
| MB102 | ≈ 10 g (sin dato publicado) | sale |
| ESP32-CAM con cámara | 7–10 g | se queda |
| 4 motores con hélices y cables | ≈ 4 g c/u (por diferencia) | se quedan |
| ESP32-S3-Zero con pines | 2–3 g | se queda |
| 2 DRV8833 con pines | ≈ 1,7 g c/u | se quedan |
| VL53L1X (ToF400C) | ≈ 1–1,5 g | se queda |
| GY-88 (no aparece en la foto) | ≈ 2–3 g | entra |
| Step-up 5 V, D1, C1, C2 y resistencias | ≈ 1,5–3 g | entran |

Quitar el MB102 y sumar el GY-88, el step-up y los componentes pequeños deja la electrónica ≈ 5 g más liviana con el Pololu, sin contar S1, S2, los conectores, los fusibles ni el cable. El MT3608 no tiene peso publicado: si lo usan, pésenlo. Una segunda ESP32-CAM sumaría 7–10 g más la barra; en Bogotá cada gramo pide ≈ 1,3 L más de helio (0,776 g de sustentación por litro). Pesen cada pieza: casi ninguna de estas masas está medida.

Pendiente

## Preguntas abiertas

1. **Las pruebas de empuje.** ¿Giraba un solo motor en cada foto? ¿En la prueba 1 la hélice iba al revés? ¿Qué quería decir «95 %»: había un PWM, un MOSFET o el DRV8833 entre la fuente y el motor, o la fuente iba directo?
2. **La celda de los motores.** ¿Qué tasa de descarga tienen las HJ 702035P y traen placa de protección? La etiqueta no lo dice: pidan la ficha al vendedor o compren una celda de 20 C o más con ficha.
3. **Dónde van los motores en la góndola.** Con esas posiciones se verifica la regla de 10 cm para los cuatro motores, no solo para los drivers.
4. **El lente de la cámara.** El ángulo de visión cambia cuántos píxeles ocupa el globo y cuánto tiempo se mantiene a la vista.
5. **Una curva de empuje a través del DRV8833:** de 10 % a 100 % de PWM, en los dos sentidos, con la batería real.
6. **Pesar cada pieza y la góndola terminada**, con cables y estructura.

Para compartir

## Las dos figuras en imagen

El diagrama de conexiones y el mapa de pines del S3-Zero en PNG, con fondo claro, para mandarlos por chat o imprimirlos.

![Diagrama de conexiones del blimp: las dos baterías 1S, los dos DRV8833 con los cuatro motores, el ESP32-S3-Zero, el GY-88, el VL53L1X, la ESP32-CAM y el step-up de 5 V.](cableado-blimp-diagrama.png)

Diagrama de conexionesPNG de 2100 × 2040 px, con la leyenda de colores.

![Mapa de pines del ESP32-S3-Zero visto desde arriba, con la función de cada pad.](cableado-blimp-pines-s3zero.png)

Pines del ESP32-S3-ZeroPNG de 1650 × 1050 px.

De dónde salen los datos

## Fuentes

- Reglamento IAR-LARC 2026: Reglas 1, 7, 13, 15, 17, 21 y 23.
- ESP32-S3-Zero: [wiki de Waveshare](https://www.waveshare.com/wiki/ESP32-S3-Zero) y [esquemático](https://files.waveshare.com/wiki/ESP32-S3-Zero/ESP32-S3-Zero-Sch.pdf); [hoja de datos del ESP32-S3](https://documentation.espressif.com/esp32-s3_datasheet_en.pdf).
- DRV8833: [hoja de datos de TI](https://www.ti.com/lit/ds/symlink/drv8833.pdf); [módulo EEP/ULT (Components101)](https://components101.com/modules/drv8833-dual-h-bridge-motor-driver-module); [prueba térmica de Pololu](https://www.pololu.com/product/2130); MOSFET [AO3400A](https://www.aosmd.com/res/datasheets/AO3400A.pdf) y [Si2300DS](https://www.vishay.com/docs/65701/si2300ds.pdf).
- GY-88: [esquemático (AAU)](https://satlab.aau.dk/stem/jensd/doc/aau-cansat/gy80gy87gy88gy91/gy88schematics.jpg); [HMC5883L](https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf); campo magnético de Bogotá con el [modelo WMM2025](https://geomag.bgs.ac.uk/data_service/models_compass/wmm_calc.html); [interferencia magnética](https://ardupilot.org/copter/docs/common-magnetic-interference.html) y [montaje de la brújula](https://ardupilot.org/copter/docs/common-installing-3dr-ublox-gps-compass-module.html) (ArduPilot).
- VL53L1X: [hoja de datos de ST](https://www.st.com/resource/en/datasheet/vl53l1x.pdf); [placa de referencia de Pololu](https://www.pololu.com/product/3415).
- ESP32-CAM: [esquemático V1.6](https://github.com/SeeedDocument/forum_doc/raw/master/reg/ESP32_CAM_V1.6.pdf); [medición de cuadros por segundo (2025)](https://arxiv.org/abs/2505.24081); [biblioteca esp32-camera](https://github.com/espressif/esp32-camera); [plano del módulo OV2640 de 120°](https://resource.heltec.cn/download/HT-HC32/Datasheet/OV2640-120.pdf).
- Estéreo frente a tamaño conocido: [equipo de Bonn, MBZIRC 2020](https://arxiv.org/abs/2010.14913); [MochiSwarm](https://arxiv.org/abs/2503.03077); [par estéreo de ESP32-CAM a corta distancia](https://doi.org/10.3390/app142311444).
- Alimentación: [esquemático del MB102](https://alessandra-flammini.unibs.it/EG_FED/EG_AA2324/FE/Laboratorio/Datasheet%20componenti/alimentatore%20per%20breadboard%20mb102-ps.pdf); [AMS1117](https://files.waveshare.com/upload/b/b8/AMS1117-datasheet.pdf); [ME6217](https://storage.googleapis.com/graviton-electric-symbols/document_assets/lcsc/1912111437_MICRONE-Nanjing-Micro-One-Elec-ME6217C33M5G_C427602.pdf); [Pololu U1V11F5](https://www.pololu.com/product/2562); [Pololu S9V11F3S5](https://www.pololu.com/product/2872) y [S9V11F3S5C3](https://www.pololu.com/product/2873); [cortocircuito en un step-up (Analog Devices)](https://www.analog.com/en/resources/design-notes/short-circuit-protection-for-boost-regulators.html); [PTC Bourns MF-R](https://www.bourns.com/docs/Product-Datasheets/mfr.pdf); celdas de 380 mAh de [Electrokit](https://www.electrokit.com/en/batteri-lipo-3.7v-380mah) y [EEMB](https://eemb.oss-accelerate.aliyuncs.com//uploads/20230311/c4a12d3b1ed4dcb5554e48cbff1a5b62.pdf) (2 C); [conector JST PH](https://www.jst-mfg.com/product/pdf/eng/ePH.pdf) (2 A); diodos [SS14](https://www.vishay.com/docs/88746/ss12.pdf) y [1N5817](https://www.vishay.com/docs/88525/1n5817.pdf); [DW01A](https://components101.com/ics/dw01a-battery-protection-ic).
- Simulación: el simulador MuJoCo del repositorio DroneUN. No se cambió el código del controlador: el simulador le pasa a su mezclador el empuje máximo de cada escena, y las ganancias y los límites de avance y de giro siguen calculados para 35 gf.