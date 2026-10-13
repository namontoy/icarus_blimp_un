DroneUN · IAR-LARC 2026 · respuesta al boceto de las tres distribuciones

# ¿Dónde van los cuatro motores?

Comparé las tres distribuciones del boceto por peso, por maniobrabilidad y por lo que le hacen a la brújula. La maniobrabilidad sale de 298 vuelos en el simulador del proyecto, con el mismo controlador que va en el ESP32. Lo de la brújula sale de un modelo del campo magnético de cada motor, alimentado con las corrientes de esos vuelos. Después probé cuatro ideas más: motores reversibles, motores en el ecuador del sobre, un par coaxial y un motor lateral.

10 de octubre de 2026Sigue a la página [«Cableado del blimp»](https://claude.ai/artifact/2ziS7A1xhWmaybdrXCBDzU)Supuestos: góndola de 14 × 6 cm, hélices de 40 mm, 12 gf por motor

Resumen

## Lo que salió

Recomendación

El boceto 3 con la barra en la parte de atrás de la góndola, al lado de los dos DRV8833, y con los motores de avance más afuera. Los cuatro motores van en una sola barra transversal: M1 y M2 (altura) a 5,5 cm del centro y M3 y M4 (avance) a 20 cm, o a 14 cm como mínimo. La brújula y la cámara quedan solas en el frente, a 12,3 cm del motor más cercano (figura 5). Simulé esta misma distribución: completó las 60 vueltas de 60 sin salirse de la cancha, con mejores vueltas de 25,4 y 26,6 s, ≈ 2 s menos por vuelta que el mejor de los tres bocetos. Usando además la reversa de M3 y M4 para girar y frenar bajó ≈ 2,5 s más («Segunda ronda»).

Peso

No decide. Varilla y cables pesan entre 1,7 y 3,9 g según la distribución. La diferencia más grande, 2,1 g, pide 2,7 L más de helio, unos 3 mm más de diámetro en un sobre de 0,78 m. Pesa más la caída de tensión en los cables largos: con 28 AWG, el motor más lejano de los bocetos 1 y 2 pierde ≈ 5 % de empuje; en la propuesta, con 26 AWG para M3 y M4, ≈ 3 %.

Maniobrabilidad

Lo que cambia el vuelo es qué tan separados van M3 y M4 del centro; dónde va cada pareja casi no importa. Con 12 gf por motor y M3 y M4 a 14 cm o más, todos los vuelos completaron sus tres vueltas en los dos recorridos, y a 20 cm la mejor vuelta bajó a 25,1 s en el rectángulo y 26,4 s en el ocho. Con 10,5 cm o menos se perdieron vueltas en el rectángulo. Por eso los bocetos 1 y 2, con 12 y 11 cm, salieron mejor que el 3, con 10,5 cm. La propuesta, con la barra atrás y M3 y M4 a 20 cm, voló igual de bien que el barrido: todas las vueltas y la menor dispersión de todas las opciones.

Brújula

Decide la distancia al motor más cercano, porque el campo de un motor cae con el cubo de la distancia. Con la brújula en el frente de la góndola, el error de rumbo que la calibración no quita (percentil 95 durante los vuelos) es ≈ 2,9° en el boceto 1, 0,9° en el 2, 1,4° en el 3 y 0,3° en la propuesta. Son estimaciones: la prueba de la sección «Pruebas» da el número real.

Segunda ronda

De las cuatro ideas extra, la que vale es usar la reversa de M3 y M4, que los DRV8833 ya permiten: girar en el lugar y frenar después de cada toque bajó la mejor vuelta ≈ 10 %, de 25,3 y 26,8 s a 23,0 y 23,9 s con las mismas semillas. Con reversa, una barra de 14 cm rinde casi igual que la de 20 cm. Los motores en el ecuador del sobre, el par coaxial y el motor lateral no mejoraron el tiempo.

Cuidado con

En el boceto 1, M2 queda delante de la cámara. En el boceto 3 tal como está dibujado, las hélices de M1 y M4, y las de M2 y M3, se chocan: necesitan unos 5 cm entre ejes. En cualquier distribución, las hélices de altura deben quedar unos 4 cm o más por debajo del sobre.

Punto de partida

## Cómo leí el boceto

Leí ○ como un motor con la hélice horizontal, que empuja hacia arriba, y ⊤ como un motor con la hélice vertical vista de canto, que empuja hacia adelante. Así, en los tres bocetos M1 y M2 controlan la altura, y M3 y M4 el avance y el giro: para girar, uno de los dos empuja más que el otro. Es el mismo reparto que usa hoy el controlador: M4 es FL, M3 es FR, M1 es RL y M2 es RR. Si los símbolos querían decir otra cosa, me avisan y repito el análisis.

Para comparar a escala supuse la placa final de la página de cableado: una góndola de 14 × 6 cm con los dos DRV8833 atrás y el GY-88 adelante, hélices de 40 mm y motores de 3,5 g con hélice. Cambié dos cosas del dibujo. En el boceto 1 puse M1 y M2 a la misma distancia del centro, 12 cm, porque el controlador les da el mismo empuje a los dos y, si uno está más lejos, la nariz se inclina. En el boceto 3 separé los motores para que las hélices no se toquen. Y agregué una cuarta opción: el boceto 3 con la barra 5 cm más atrás, al lado de los drivers.

### Boceto 1

Cruz. M1 y M2 (altura) en la cola y la nariz, a 12 cm; M3 y M4 (avance) en una barra transversal, a ±12 cm.

### Boceto 2

Cuatro brazos. M1 y M2 (altura) adelante, a ±11 cm; M3 y M4 (avance) atrás, a ±11 cm.

### Boceto 3

Una barra por el centro. M1 y M2 (altura) a ±5,5 cm; M3 y M4 (avance) a ±10,5 cm.

### Boceto 3, barra atrás

La misma barra, 5 cm más atrás, junto a los drivers.

Figura 1 · Las cuatro opciones, a escala Vista desde arriba, con el frente hacia arriba. Círculo punteado: hélice de un motor de altura. Rectángulo con raya: motor de avance con la hélice vista de canto. Rombo verde: el chip del magnetómetro, en el frente de la placa, detrás de la cámara. Cuadros grises: los dos DRV8833. Las varillas son de carbono de 2 mm.

Peso

## La estructura pesa poco en cualquier caso

Supuse varilla de carbono maciza de 2 mm (4,9 g/m) y, para cada motor, un par de cables de 28 AWG (2,5 g/m el par) que sale de los drivers y va por el borde de la góndola hasta el motor. Los motores y sus soportes pesan lo mismo en todas las opciones, así que no los sumé. La propuesta lleva tubo de carbono de 3 mm (6,1 g/m) y 26 AWG para M3 y M4, que es lo que usa la fila resaltada.

| Distribución | Varilla | Cables de motor | Total | Helio extra | Cable más largo | Empuje de ese motor |
| --- | --- | --- | --- | --- | --- | --- |
| Boceto 1 | 48 cm · 2,4 g | 59 cm · 1,5 g | **3,8 g** | +2,7 L | 20 cm | −5,2 % |
| Boceto 2 | 44 cm · 2,2 g | 62 cm · 1,6 g | **3,7 g** | +2,5 L | 20 cm | −5,3 % |
| Boceto 3 | 21 cm · 1,0 g | 48 cm · 1,2 g | **2,2 g** | +0,6 L | 14 cm | −3,8 % |
| Boceto 3, barra atrás | 21 cm · 1,0 g | 28 cm · 0,7 g | **1,7 g** | +0,0 L | 10 cm | −2,5 % |
| Propuesta: barra atrás a 20 cm | 44 cm · 2,7 g | 47 cm · 1,2 g | **3,9 g** | +2,7 L | 19 cm | −3,1 % |

En Bogotá cada gramo pide ≈ 1,3 L más de helio. Los 2,1 g que separan la opción más pesada de la más liviana son 3 mm más de diámetro del sobre, y en el simulador eso no se nota en el tiempo de vuelta. La última columna importa más. A 1,2 A, un par de 28 AWG pierde ≈ 0,05 V por cada 10 cm de largo, y el empuje cae en porcentaje cerca del doble que la tensión. Si algún cable de motor pasa de 15 cm, usen 26 AWG.

Maniobrabilidad

## Qué dice el simulador

Corrí el simulador MuJoCo del repositorio con el controlador real, el mismo código que va en el ESP32, y cambié solo los motores: dónde va cada uno, su masa y la de varillas y cables, y la separación de M3 y M4 que el controlador usa para calcular el giro. Cada distribución voló los dos recorridos con 10 semillas de ruido de sensores: 300 s simulados por vuelo, altura con el ToF y 12 gf por motor, que es lo que espero a través del DRV8833 con la celda llena. Cada semilla es un sorteo distinto del ruido, así que se comparan las nubes de puntos de la figura 2 y no un vuelo contra otro.

| Distribución | Recorrido 1 · rectángulo | Recorrido 2 · ocho | Vueltas sin salirse | Girando a fondo | Nariz arriba, media | Cabeceo, RMS |  |  |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Vueltas | Mejor vuelta | Vueltas | Mejor vuelta |  |  |  |  |  |
| Simulador actual | 25 de 30 | 28,3 s (27–111) | 28 de 30 | 28,9 s (28–171) | 51 de 53 | 40 % | 3,4° | 2,1° |
| Boceto 1 | 30 de 30 | 28,3 s (27–52) | 30 de 30 | 28,4 s (26–30) | 60 de 60 | 28 % | 3,6° | 2,2° |
| Boceto 2 | 30 de 30 | 28,5 s (27–31) | 29 de 30 | 28,6 s (28–31) | 53 de 59 | 36 % | 3,7° | 2,2° |
| Boceto 3 | 22 de 30 | 34,9 s (28–144) | 30 de 30 | 28,8 s (27–31) | 50 de 52 | 42 % | 3,6° | 2,2° |
| Boceto 3, barra atrás | 24 de 30 | 31,8 s (27–51) | 30 de 30 | 28,1 s (27–29) | 52 de 54 | 37 % | 4,1° | 2,2° |
| Propuesta: barra atrás a 20 cm | 30 de 30 | 25,4 s (25–26) | 30 de 30 | 26,6 s (25–28) | 60 de 60 | 9 % | 4,4° | 2,5° |

Mejor vuelta: mediana de las 10 semillas, con el rango entre paréntesis. «Vueltas sin salirse»: vueltas completas en las que el sobre nunca cruzó el borde de la cancha de 10 × 5 m (Regla 7). «Girando a fondo»: parte del vuelo con un motor de avance al máximo y el otro apagado.

Figura 2 · Mejor vuelta de cada vuelo simulado Cada punto es un vuelo (una semilla) y la raya vertical es la mediana. Los triángulos son vuelos cuya mejor vuelta pasó de 60 s, y los círculos vacíos, vuelos sin ninguna vuelta completa. 16,8 gf es lo que dio un motor en la fuente de laboratorio, sin el DRV8833. Pasen el cursor por un punto para ver la semilla y cuántas vueltas completó.

Con 16,8 gf por motor, lo que dio el motor en la fuente de laboratorio, las cuatro opciones de los bocetos completaron 72 de 72 vueltas, con medianas de la mejor vuelta entre 25,7 y 32,7 s. Con solo 3 semillas por opción esas diferencias no alcanzan para ordenarlas. La propuesta completó 30 de 30, con medianas de 24,0 y 24,1 s.

### Por qué importa la separación

Los motores solo empujan hacia adelante. Para girar, el controlador sube un motor de avance y baja el otro, así que el blimp gira y a la vez avanza. El giro es empuje por distancia al centro: con M3 y M4 más separados, el mismo giro pide menos diferencia de empuje y empuja menos al blimp hacia adelante mientras gira. Con 12 gf y 10,5 cm, el controlador pasó el 42 % del vuelo con un motor de avance al máximo y el otro apagado, es decir, pidiendo más giro del que había. Con 20 cm, el 9 %.

### Lo que casi no cuenta

- Dónde va la barra, adelante, al centro o atrás: el giro solo depende de la distancia lateral de M3 y M4. El boceto 3 y la barra atrás, con la misma separación, completaron 22 y 24 vueltas de 30 en el rectángulo y 30 de 30 en el ocho, una diferencia dentro del ruido.
- Motores de altura fuera del centro: la góndola cuelga ≈ 42 cm bajo el centro del sobre y endereza el blimp. La nariz va en promedio entre 3,4° y 4,4° arriba en todas las opciones, un poco más con la barra atrás por el peso de los motores, y el cabeceo varía ≈ 2,2° RMS en todas.
- La masa en brazos largos: la inercia de giro cambia menos de 3 % entre opciones, porque la del sobre domina.

Figura 3 · Qué tan separados van M3 y M4 Boceto 3 con M3 y M4 a distintas distancias del centro, 12 gf por motor, semillas 0 a 5 en los dos recorridos. 10,5 cm es la opción de la figura 1. El controlador recibe la separación de cada caso.

| M3 y M4 a | Vueltas · rect. | Mejor · rect. | Vueltas · ocho | Mejor · ocho | Girando a fondo |
| --- | --- | --- | --- | --- | --- |
| 6 cm | 15 de 18 | 36,1 s | 17 de 18 | 30,9 s | 58 % |
| 8 cm | 17 de 18 | 29,5 s | 18 de 18 | 33,7 s | 50 % |
| 10,5 cm | 13 de 18 | 33,4 s | 18 de 18 | 28,8 s | 42 % |
| 14 cm | 18 de 18 | 26,9 s | 18 de 18 | 27,6 s | 17 % |
| 20 cm | 18 de 18 | 25,1 s | 18 de 18 | 26,4 s | 9 % |

Desde 14 cm, los 12 vuelos de cada punto completaron sus tres vueltas, con poca dispersión, y la mejor vuelta siguió bajando hasta 20 cm, el máximo que probé. Por debajo de 14 cm los resultados se dispersan y aparecen vueltas perdidas, la señal de un giro que se queda corto; que 8 cm saliera mejor que 10,5 cm en el rectángulo es parte de esa dispersión. Más separación cuesta poco: pasar de 10,5 a 20 cm suma ≈ 2 g entre tubo y cables. Recomiendo 20 cm, con tubo de carbono de 3 mm y cables de 26 AWG para M3 y M4, o 14 cm como mínimo si se quedan con la varilla de 2 mm. La última fila de la tabla de arriba y de la figura 2 es esa propuesta ya armada, con la barra atrás.

Brújula

## Lo que la calibración no quita

Calibrar la brújula con el blimp armado quita el campo fijo de los imanes de los motores y de las piezas de acero, porque giran con el sensor. No quita el campo que cambia con la corriente: cada motor genera uno proporcional a lo que consume, y sus cables otro. Ese campo cambia en cada maniobra, y es el que muestra la figura 4.

### Boceto 1

Frente de la góndola: 2,9° · mejor punto: 0,3°

### Boceto 2

Frente de la góndola: 0,9° · mejor punto: 0,4°

### Boceto 3

Frente de la góndola: 1,4° · mejor punto: 0,8°

### Propuesta: barra atrás a 20 cm

Frente de la góndola: 0,3° · mejor punto: 0,2°

Figura 4 · Error de rumbo según dónde vaya el magnetómetro

menos de 0,5°0,5–1°1–2°2–5°más de 5°

Cada color es el error de rumbo que tendría el chip en ese punto, a 1 cm sobre la varilla: el percentil 95 de los vuelos a 12 gf, en la peor orientación del blimp. El rombo verde es el frente de la placa, donde lo pone la página de cableado, y el círculo verde es el mejor punto dentro de la góndola. En la tabla, ese punto va en centímetros (adelante; izquierda) desde el centro de la placa. Pasen el cursor por el mapa para leer el valor.

| Distribución | Motor más cercano al frente | Frente de la placa | Centro | 4 cm delante de la placa | Mejor punto en la góndola |
| --- | --- | --- | --- | --- | --- |
| Boceto 1 | 6,0 cm | **2,9°** | 0,7° | 42,7° | 0,3° en (−1; −3) cm |
| Boceto 2 | 11,0 cm | **0,9°** | 0,8° | 0,6° | 0,4° en (5; −2) cm |
| Boceto 3 | 8,1 cm | **1,4°** | 6,4° | 0,4° | 0,8° en (−7; −1) cm |
| Boceto 3, barra atrás | 12,3 cm | **0,4°** | 2,0° | 0,2° | 0,3° en (7; 2) cm |
| Propuesta: barra atrás a 20 cm | 12,3 cm | **0,3°** | 1,9° | 0,1° | 0,2° en (7; 2) cm |

### El modelo

Cada motor es un imán de 1,5 × 10⁻³ A·m² por amperio, orientado en la peor dirección posible, más su par de cables sin trenzar, a 1,5 mm entre conductores. El número del imán lo estimé con la constante de par de un motor de 7 mm y el campo típico de su imán; no está medido. La corriente de cada motor sale del empuje de cada instante de los vuelos simulados: 0,12 A en vacío más ≈ 0,09 A por gf, de la prueba de empuje (1,45 A a 14,7 gf). El error es el ángulo que ese campo le suma a los 26,8 µT horizontales de Bogotá.

Los imanes dominan. Un motor a 1,2 A genera ≈ 0,36 µT a 10 cm en su eje, que son ≈ 0,8°, y ≈ 2,9 µT a 5 cm, ≈ 6°. Los cables solo cuentan a 1–3 cm del chip. Por eso el mapa se parece tanto a «distancia al motor más cercano».

Qué tan seguro es

El número del imán puede estar errado por un factor de 3 hacia arriba o hacia abajo, y con él todos los grados de la tabla. El orden entre distribuciones no cambia, porque depende de las distancias. Como referencia, un motor de avión sin escobillas mucho más grande (batería de 3 celdas, hélice de 12 pulgadas) movió ≈ 2 µT el campo a 15 cm al subir el acelerador a la mitad. La primera prueba de la sección «Pruebas» da el número real en diez minutos.

Lo que el simulador no ve

## Detalles de armado

### La cámara, en el boceto 1

M2 va en la nariz, entre 3 y 7 cm delante de la cámara y a su misma altura. Desde la cámara, esa hélice y su motor ocupan una franja de ≈ 40° de ancho en el centro de la imagen, justo donde aparece el globo. Habría que bajar M2 unos 5 cm o adelantar la cámara.

### Hélices que se chocan, en el boceto 3

En el dibujo, M1 está pegado a M4 y M2 a M3. Con hélices de 40 mm, un disco horizontal y uno vertical se cruzan si sus ejes quedan a menos de 4 cm. Dejé 5 cm: M1 y M2 a 5,5 cm del centro, justo por fuera del borde de la góndola, y M3 y M4 a 10,5 cm o más.

### Hélices de altura bajo el sobre

Una hélice muy cerca de una superficie que le tapa la entrada de aire mide más empuje en la báscula. Aquí ese aumento no ayudaría: la misma succión jala el sobre hacia abajo, y el sobre va pegado al blimp. Con el modelo de un estudio de hélices pequeñas bajo un techo, para una hélice de 40 mm el efecto vale ≈ 2 % a 2 cm de separación y ≈ 0,5 % a 4 cm, así que con 4 cm entre la hélice y el sobre queda descartado. Poner M1 y M2 más afuera casi no ayuda: a 11 cm del centro, el fondo del sobre solo sube 1,6 cm.

### Equilibrio de la góndola

Con la barra atrás, ≈ 18 g de motores, tubo y cables quedan 5 cm detrás del centro de la placa. Cuelguen la góndola del sobre en su punto de equilibrio, o adelanten la celda de la lógica. El empuje de M1 y M2 queda unos centímetros detrás del centro, y en el simulador eso cambia el cabeceo medio en 0,5°.

### Rigidez de la varilla

Con un motor de avance empujando 12 gf en la punta, una varilla de carbono maciza de 2 mm se dobla ≈ 0,7 mm a 10,5 cm, ≈ 1,6 mm a 14 cm y ≈ 5 mm a 20 cm, donde además puede vibrar con la hélice. Un tubo de carbono de 3 mm (2 mm por dentro) es cuatro veces más rígido: ≈ 1,2 mm a 20 cm, por ≈ 0,5 g más en toda la barra.

### Reglas

Todas las opciones cumplen la Regla 14 (caja de 1 × 1 × 1,5 m). Con las hélices a menos de 25 cm del centro, el sobre de 39 cm de radio toca el globo y el cable antes que cualquier hélice, lo que cuida las Reglas 16 y 17. Si alargan los brazos, mantengan las hélices dentro de la sombra del sobre.

Propuesta

## La barra atrás, con medidas

Figura 5 · Distribución recomendada Vista desde arriba, a escala, con el frente hacia arriba. La línea punteada va del chip del magnetómetro al motor más cercano.

1. Un tubo de carbono de 3 mm de 44 cm, transversal, en la parte de atrás de la placa, debajo de los dos DRV8833. Si prefieren la varilla maciza de 2 mm, acorten los brazos a 14 cm.
2. M1 (izquierda) y M2 (derecha) con la hélice horizontal, a 5,5 cm del centro, por fuera del borde de la góndola. M4 (izquierda) y M3 (derecha) con la hélice vertical, a 20 cm del centro. Entre el eje de un motor de altura y el del motor de avance de su lado, al menos 5 cm.
3. Cada DRV8833 mueve los dos motores de su lado, como en la página de cableado: el #1 a M4 y M1 (FL y RL) y el #2 a M3 y M2 (FR y RR). Así los cables de M1 y M2 miden ≈ 5 cm y los de M3 y M4 ≈ 19 cm; para esos dos usen 26 AWG. Trencen cada par.
4. El GY-88 y la cámara van en el frente. La batería de motores y su cable, atrás, junto a los drivers.
5. Cuelguen la góndola del sobre en su punto de equilibrio y dejen 4 cm o más entre las hélices de M1 y M2 y el sobre.
6. En el controlador, `front_half_span_m` pasa a 0,20 m, la distancia de M3 y M4 al centro. Hoy vale 0,10 m.

Segunda ronda

## Cuatro ideas más, en el simulador

Probé las cuatro ideas sobre la propuesta (barra atrás, M3 y M4 a 20 cm), con 12 gf por motor y 6 semillas en cada recorrido. Las ideas 1 y 4 necesitan cambios en el controlador: los hice en una copia aparte del código, compilada solo para estas pruebas, y no tocan el repositorio. La propuesta aparece con las mismas 6 semillas, para comparar parejo.

### 1 · Motores de avance reversibles

La misma propuesta. M3 y M4 empujan hacia adelante o hacia atrás.

### 2 · Avance en el ecuador del sobre

M3 y M4 a 40 cm del centro, a la altura del centro del sobre.

### 3 · Par coaxial de altura

M1 y M2 en el mismo eje vertical, en sentidos opuestos.

### 4 · Motor lateral

Un motor de altura y uno lateral reversible; M3 y M4 reversibles.

Figura 6 · Las cuatro ideas Vistas desde arriba, con el frente hacia arriba, salvo la 2, vista desde atrás. Flechas verdes: motores que empujan en los dos sentidos. Línea naranja punteada en la 2: el cable de cada motor, que baja por el sobre hasta la góndola.

| Variante | Recorrido 1 · rectángulo | Recorrido 2 · ocho | Vueltas sin salirse | Girando a fondo | Nariz arriba, media | Brújula al frente |  |  |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Vueltas | Mejor vuelta | Vueltas | Mejor vuelta |  |  |  |  |  |
| Propuesta (base) | 18 de 18 | 25,3 s (25–26) | 18 de 18 | 26,8 s (26–27) | 36 de 36 | 9 % | 4,4° | 0,3° |
| 1a · Reversibles, solo giro | 18 de 18 | 24,9 s (25–25) | 18 de 18 | 26,1 s (26–27) | 36 de 36 | 5 % | 4,2° | 0,3° |
| 1b · Reversibles + freno | 18 de 18 | 23,0 s (23–24) | 18 de 18 | 23,9 s (24–24) | 36 de 36 | 6 % | 4,1° | 0,3° |
| 1b con M3 y M4 a 14 cm | 18 de 18 | 23,3 s (23–25) | 18 de 18 | 24,5 s (24–25) | 36 de 36 | 10 % | 3,9° | 0,3° |
| 1b con M3 y M4 a 10,5 cm | 18 de 18 | 23,7 s (23–25) | 18 de 18 | 24,9 s (24–25) | 36 de 36 | 16 % | 3,9° | 0,4° |
| 2 · Avance en el ecuador | 18 de 18 | 26,3 s (25–27) | 18 de 18 | 25,9 s (24–28) | 35 de 36 | 3 % | 0,5° | 0,3° |
| 3 · Par coaxial | 18 de 18 | 25,4 s (25–26) | 18 de 18 | 26,7 s (26–28) | 36 de 36 | 9 % | 4,4° | 0,9° |
| 3′ · Mismo sentido de giro | 18 de 18 | 25,7 s (25–26) | 18 de 18 | 25,9 s (25–27) | 36 de 36 | 10 % | 4,5° | 0,3° |
| 4 · Motor lateral (con 1b) | 18 de 18 | 23,6 s (23–24) | 18 de 18 | 24,3 s (24–25) | 36 de 36 | 5 % | 3,8° | 0,9° |

Mejor vuelta: mediana de 6 semillas, con el rango entre paréntesis. «Brújula al frente»: error de rumbo, percentil 95, con el chip en el frente de la placa.

Figura 7 · Mejor vuelta de cada vuelo, segunda ronda Cada punto es un vuelo y la raya vertical, la mediana. Las dos filas «1b · 14 cm» y «1b · 10,5 cm» son la idea 1 con la barra más corta.

### 1 · Reversibles: la que más ayuda

Girar en el lugar, con un motor hacia adelante y otro hacia atrás, baja la vuelta apenas ≈ 0,5 s (1a). Lo que más pesa es frenar: si después de cada toque M3 y M4 empujan hacia atrás 0,7 s en vez de apagarse 1 s, la mejor vuelta baja 2,3 s en el rectángulo y 2,9 s en el ocho (1b), ≈ 10 %, sin perder ninguna vuelta. Los rangos de 1b y de la propuesta ni se tocan.

Con reversa la barra larga importa mucho menos. A 14 cm, 1b da 23,3 y 24,5 s; a 10,5 cm, 23,7 y 24,9 s, y completa todo. Sin reversa, esos mismos 10,5 cm perdían vueltas.

El DRV8833 ya permite la reversa, así que no cuesta piezas; cuesta trabajo en el controlador: el mezclador y el estado que sigue a cada toque. El freno del prototipo es simple (reversa fija, ≈ 10 gf entre los dos motores, sin medir la velocidad) y supone que la reversa da el 74 % del empuje, que sale de una sola foto y hay que medir a través del DRV8833.

### 2 · Ecuador del sobre: no vale la pena

Con M3 y M4 a 20 cm ya sobra giro; a 40 cm el tiempo girando a fondo baja de 9 % a 3 %, pero la vuelta salió 0,9 s más lenta en el rectángulo y 0,9 s más rápida en el ocho, dentro del ruido. Lo que sí gana es una nariz casi nivelada, 0,5° contra 4,4°, porque el empuje sale a la altura del centro del sobre, y eso estabiliza la cámara.

A cambio: ≈ 5 g más de cables y soportes, ≈ 7 % menos de empuje en esos motores por 65 cm de cable de 24 AWG (ya incluido en la simulación), motores pegados a una tela que se deforma, y hélices que sobresalen ≈ 5 cm del sobre, que es lo primero que tocaría un cable o un globo de costado (Reglas 16 y 17).

### 3 · Par coaxial: igual en vuelo, peor para la brújula

Vuela como la propuesta (25,4 y 26,7 s), aun con 20 % menos de empuje de altura, porque la altura nunca estuvo limitada por empuje: solo carga los 6 gf de lastre y subidas cortas. Empeora la brújula: con M1 y M2 juntos en la línea central, a 11 cm del chip, sus campos se suman y el error sube a 0,9°, contra 0,3°. Se arregla corriendo el par 2–3 cm hacia atrás o el GY-88 hacia adelante.

La ventaja es mecánica: un solo soporte y una barra que solo carga M3 y M4. En los estudios con hélices más grandes, un par coaxial rinde 16–27 % menos que dos hélices sueltas, y la mejor separación entre las dos hélices es ≈ 0,2 diámetros, ≈ 8 mm con hélices de 40 mm. Vale si les simplifica el armado; midan el par en el soporte de empuje antes.

Hacer girar las dos hélices de cada pareja en el mismo sentido (3′) no cambió nada medible (25,7 y 25,9 s): el giro que provoca equivale a décimas de gf. Alternar los sentidos sigue siendo buena práctica, pero no es urgente.

### 4 · Motor lateral: descartado

Sumado a 1b, el motor lateral dejó la vuelta 0,6 s más lenta en el rectángulo y 0,4 s en el ocho. Sin una medida de la velocidad lateral, solo puede corregir con la posición del globo en la imagen, y el giro reversible ya alinea rápido. Además cuesta un motor de altura (quedan 6 gf para subir) y empeora la brújula, 0,9°, por los dos motores en el centro de la barra.

Qué cambia en la propuesta

La geometría se queda: barra atrás, M1 y M2 a 5,5 cm, M3 y M4 a 20 cm. Lo que se suma es usar la reversa de M3 y M4, para girar en el lugar y para frenar después de cada toque. Si hacen eso, 14 cm con varilla maciza de 2 mm pierde solo ≈ 0,3–0,6 s por vuelta frente a los 20 cm con tubo. Antes de cambiar el controlador hay que medir la reversa real a través del DRV8833 (segunda prueba de la lista de abajo).

Pendiente

## Pruebas antes de pegar la barra

1. **Brújula contra un motor, unos 10 minutos.** Pongan el GY-88 sobre la mesa, lejos de cualquier pieza de hierro, y un motor con hélice en el soporte de empuje a 5, 8, 10 y 15 cm del chip del magnetómetro. A cada distancia, lean el campo 5 s con el motor apagado y 5 s al 30, 60 y 100 % de PWM, y anoten la corriente. Repitan con el motor de costado. La diferencia entre encendido y apagado, dividida por la corriente, es el número que este modelo supone: ≈ 0,3 µT por amperio a 10 cm en el peor caso. Si sale mucho más alto, la brújula necesita más distancia o una compensación por corriente en el firmware.
2. **Empuje en reversa, a través del DRV8833.** Con la celda de motores real y el driver en el circuito, midan el empuje de un motor de avance de 10 a 100 % de PWM en los dos sentidos. El 74 % de reversa sale de una sola foto, sin driver y con la hélice que parecía girar al revés; de ese número depende el freno de la segunda ronda.
3. **Hélice de altura bajo el sobre.** En el soporte de empuje, fijen un cartón plano al mismo soporte del motor, 2, 4 y 6 cm por encima de la hélice. Como el cartón va sujeto al soporte, la báscula mide el empuje neto, igual que en el blimp. Comparen con la prueba sin cartón.
4. **Si prueban el par coaxial:** monten dos motores uno sobre otro en el soporte, con hélices de giro opuesto separadas ≈ 8 mm, y comparen el empuje del par con el doble del de un motor solo, a la misma tensión.
5. **Pesar la barra armada** con los cuatro motores, las hélices, los soportes y los cables, para cerrar el balance de helio.

De dónde salen los números

## Cómo se hizo

- Simulador: el MuJoCo del repositorio DroneUN con el controlador compilado de `core/`, sin cambios en su código. Por distribución cambié la posición y la masa de cada motor (3,5 g), de las varillas y de los cables, y le pasé al controlador la separación de M3 y M4 y su límite de giro (empuje máximo × separación). La masa total y el lastre de 6 gf no cambian. Sobre de 0,78 m, góndola de 170 g como en el resto del simulador, altura con el ToF, 300 s por vuelo. En total, 298 vuelos.
- El simulador ve la posición de cada empuje, las masas, la inercia y el péndulo de la góndola. No ve el aire entre hélices, ni entre hélice y sobre, ni la caída de tensión en los cables, ni campos magnéticos. Eso está en «Lo que el simulador no ve» y en el modelo de la brújula.
- Brújula: campo horizontal de Bogotá 26,8 µT (WMM2025, el mismo de la página de cableado). Error de rumbo = arcoseno del campo horizontal de los motores sobre 26,8 µT, en la peor orientación del blimp; percentil 95 sobre el tiempo de vuelo de cada vuelo y mediana entre vuelos.
- Hélice cerca de una superficie: [Hsiao y Chirarattananon, efecto techo en rotores pequeños](https://arxiv.org/abs/1905.04632); [experimentos y modelo del efecto techo con hélices de dron (Caltech)](https://authors.library.caltech.edu/records/qpmpg-p1h18).
- Motor y magnetómetro: [medición con un motor sin escobillas a 15 cm (Universidad de Minnesota)](https://dept.aem.umn.edu/~uav/repository/export/mag.pdf); [interferencia magnética y compensación por corriente (ArduPilot)](https://ardupilot.org/copter/docs/common-magnetic-interference.html).
- Segunda ronda: copia del controlador, aparte del repositorio, con tres cambios que se activan al compilar. Reversibles: M3 y M4 entre −74 % y 100 % del empuje, el giro como par puro y su límite subido en la misma proporción. Freno: tras cada toque, 0,7 s de reversa (≈ 10 gf entre los dos) en vez de 1 s con motores apagados. Lateral: un control proporcional-derivativo sobre la distancia lateral al globo (0,15 N/m y 0,30 N·s/m). En el simulador: reversa limitada al 74 %, par coaxial al 80 % del empuje, cables de 65 cm al 93 %, y el giro de cada hélice como 0,004 N·m por newton de empuje en la variante 3′. 6 semillas por recorrido.
- Pares coaxiales: [Universidad de Southampton](https://eprints.soton.ac.uk/497886/) y [Universidad de Stavanger](https://uis.brage.unit.no/uis-xmlui/handle/11250/2679599).
- Reglamento IAR-LARC 2026: Reglas 7, 13, 14, 16 y 17.