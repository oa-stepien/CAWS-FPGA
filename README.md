# CAWS-FPGA
Commercial Aircraft's Warning System on FPGA (Field Programming Gateway Array). In this project on Zybo Z7-10

![imagen](https://user-images.githubusercontent.com/47353378/111675292-418b2800-881d-11eb-9ab2-35903576ac43.png)

## Intro [EN/ES]
### EN
First of all, this project was made completly on spanish, so... apologize if there is any traduction mistake in the project and readme (at start I wasn't thinking to upload it). Most of the code use variables in english making it universal, but the Memory or Thesis is in Spanish. Also, will be provided the .jar project, memory and code in this project.

This project was made as Master's Degree Thesis for my Degree at Distributed and Embebbed Systems Software at ETSISI,UPM (Universidad Politecnica de Madrid), Spain. The objective or goal is create a warning system for commercial aircrafts using the FPGA technology, due to its multiple applications on IoT Systems. The warning system use an FreeRTOS distro compatible for the board used, Digilent Zybo Z7-10, and require some "devices" as accelerometer, potentiometer, etc... 
> "Which will be indicated with more detail later"

The tools used are mostly from [Xilinx](https://www.xilinx.com/) as Vivado 2019.4 or Vitis 2019.4, which use and instalation would be skipped, and its not relevant. For any support you are free to make suggestions, any change or forked project I would be glad to be noticed.

Thanks.

### ES
Bueno, esto va a ser más sencillo, el proyecto no fue realizado inicialmente para ser subido a Github, por eso la introducción en ingles es algo rara, al igual que esta misma, pero ésta última es más por una valoración personal, podría ser un proyecto curioso. El código se ha intentado usar el Ingles, pero la memoria esta sólo en español. Tambien, se aporta en el directorio el proyecto .jar, la memoria y el codigo.

Este proyecto fue realizado cómo Proyecto Fin de Master (PFM) para mi curso en el máster de Software en Sistemas Distribuidos y Empotrados en la ETSISI, UPM (Universidad Politecnica de Madrid), España. El objetivo o meta es hacer un sistema de advertencia para aeronaves comerciales usando al tecnología FPGA, que ofrece muchos beneficios en el sector de la informatica y el IT (y el IoT). El sistema de advertencia es una distribución de FreeRTOS compatible con la placa utilizada, Digilent Zybo Z7-10, y requiere de algunos dispositivos o sensores como acelerómetros, potenciometros, etc...
> "Se detallará despues"

Las herramientas son en gran parte de [Xilinx](https://www.xilinx.com/) como lo son Vivado 2019.4 o Vitis 2019.4, el uso e instalción será omitido al ser irrelevante en este proyecto. Para cualquier comentario y/o sugerencia eres libre de hacer, y cualquier cambio o proyecto anidado se valoraría si fuese informado.

Gracias.

## Idea EN
Obviously, the acronym "CAWS-FPGA" is a *bad* traslation for "Sistema de Advertencia para Aeronaves Comerciales sobre FPGA" the title in spanish, but I though it would be nice the english name "Commercial Aircraft's Warning System on FPGA", also is equivalent the tile "Warning System for Commercial Aircraft on FPGA", but **WSCA-FPGA** is not very *cool*.

The idea, as I said before, is make an Real-Time System using and FPGA board and FreeRTOS. The prototipe made is very reduced to be more easy to show and limitations in the sensors (more an budget limitation). The prototipe made might be useful on drone that aircraft. 
The need of this system is created by some "hazards" that occur with the actual Warning System **EGPWS**, which is not understimated, only require some "update", the CAWS-FPGA offer an complementary system to the EGPWS, which major lack of "height knowledge" and "presence at cockpit". 

The sensors and devices used are:
1. Zybo Z7-10
2. 2x Protoboard (in the *prototype*)
3. Digilent KEYPAD "PMOD_KYPD" (for some functions)
4. Presence
    1. 2x IR Sensor FC-51 (for Pilot presence)
    2. Movement IR Sensor (for Crew presence)
5. Tilt
    1. MPU6050 GY-521 (inclination and accelerometer)
6. Height
    1. Ultrasonic Sensor HC-SR04 (height up to 2-3 m)
    2. Need an ESP8266 to get 5V for the Ultrasonic sensor
7. Speed
    1. Sliding Potentiometer "237C-50KB"
8. Responses
    1. PMOD LED module
    2. Buzzer
    3. RGB LED
    4. SD Card

* Also:
    * PMOD Cable to the Pmod-KYPD
    * (Obviously) Cables and MicroUSB cables
    * Resistors
    * Level shifter from 5V to 3.3V
    * An Amazon cardboard box ? No, I needed something to see the dimensions.

## Idea ES
Obviamente, el acronimo del proyecto en Github de CAWS-FPGA es uan muy  mala traducción del titulo orignal "Sistema de Advertencia para Aeronaves Comerciales sobre FPGA", pero supuse que tiene más gancho el acronimo en ingles de **CAWS-FPGA** en lugar de **SAAC-FPGA**

La idea, como ya comenté previamente, es hacer un sistema en tiempo real usando una placa FPGA y FreeRTOS. El prototipo realizado es muy pequeño y compacto para que sea más fácil de enseñar y ver, tambien por algunas limitaciones en los sensores (limitacion de presupuesto y condiciones). El prototipo realizado puede ser más útil en el ámbito de los drones que de la aviación, pero es lo que se podía hacer.
La necesidad de este sistema se debe a algunos "peligros" que existen con el actual sistema **EGPWS**, el cual no se infravalora, sólo requiere de una "actualización", SAAC-FPGA ofrece un sistema complementario al EGPWS, en él cual se echa en falta "conocmiento de altitud" y "presencia en cabina".

Los sensores y dispositivos son:
1. Zybo Z7-10
2. 2x Protoboard (en el *prototipo*)
3. Digilent KEYPAD "PMOD_KYPD" (para algunas funciones adicionales)
4. Presencia
    1. 2x Sensor IR FC-51 (para la presencia del piloto)
    2. Sensor de movimiento por IR (para presencia de la tripulación)
5. Inclinación
    1. MPU6050 GY-521 (inclinación y acelerómetro)
6. Altitud
    1. Sensor de Ultrasonidos HC-SR04 (altura hasta 2-3 m)
    2. Necesidad de un ESP8266 para conseguir 5V para el sensor de ultrasonidos
7. Velocidad
    1. Potenciometro Deslizante "237C-50KB"
8. Respuesta
    1. Modulo PMOD LED
    2. Zumbador
    3. LED RGB
    4. Tarjeta SD

* Tambien:
    * Cables PMOD al Pmod-KYPD
    * (Obviamente) Cables y cables MicroUSB
    * Resistencias
    * "Level shifter" de 5V a 3.3V
    * ¿Una caja de cartón de Amazon ? No, sólo era necesaria para conocer las dimensiones.

## Future/Futuro
This project and README file would be updated as soon as I can, and giving better info and working details.

Este proyecto y archivo README serán actualizados lo antes posible, dando mejor información y detalles del funcionamiento.

## Contact
Twitter: [@o_stepien](https://twitter.com/o_stepien)
