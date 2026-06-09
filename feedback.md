# Reunión 8/6/26

## Opciones de comunicación: 
Se discute el uso de módulos LoRa (que no requieren pago de abono pero tienen alcance 
limitado y necesitan un receptor) frente a tarjetas SIM (que ofrecen mayor alcance pero 
tienen costo mensual). El profesor sugiere armar un prototipo de transmisor y receptor LoRa 
y probar su alcance real en el barrio.

## Definición de "Casos de Uso": 
El profesor enfatiza la necesidad de plantear situaciones reales para determinar cómo debe funcionar 
el dispositivo. Entre los casos mencionados están:

### Zona segura (Geofencing): 
Definir un perímetro usando 4 puntos GPS (como la casa o un grupo de casas vecinas). 
Si el gato está dentro, el dispositivo no hace nada y ahorra batería. Si sale, comienza a transmitir.

### Gato inmovilizado: 
Si el gato está dentro de la zona segura pero no se mueve (por ejemplo, 
está herido o atrapado), el usuario debería poder enviar un SMS para "despertar" el dispositivo 
y que este le envíe la ubicación.

### Reporte de batería: 
El collar debería enviar un reporte periódico (ej. una vez al día) con el nivel de batería para 
saber cuándo recargarlo.

## Gestión de la batería: 
Se destaca que tener el GPS y el transmisor encendidos constantemente consumirá la batería muy rápido. 
Se proponen soluciones como usar el GPS solo cada cierto tiempo para verificar si el gato sigue en la zona segura, 
o mantener el módulo GSM apagado y encenderlo periódicamente para revisar si hay solicitudes del dueño.

## Planificación del proyecto: Se revisa el diagrama de Gantt, enfocándose en la primera semana 
(dedicada a investigación y compra de componentes como GPS, módulos LoRa y RFID). Se menciona la 
posibilidad de probar primero con un sistema de menor alcance (RFID) para entender el funcionamiento antes 
de pasar a la solución definitiva.

## Trabajo en equipo: 
Al igual que con el grupo anterior, el profesor insiste en que los estudiantes dividan las tareas y no trabajen 
todos juntos en lo mismo para ser más eficientes.
