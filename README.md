# Line-Follower-Robot-
Line Follower Robot [SBMI]

https://www.youtube.com/watch?v=MadvVoRH2LQ&t=3s

___________

Componentes: 

-Arduino Uno (ATMega328)

-HC-05 (módulo bluetooth)

-TB6612FNG (drive dos motores)

-Motores DC

-Tracker sensor infrared line

___________

Modos: (alterados com a app)

-Manual - usando a app, é possível controlar o robot usando as setas (extra: se houver obstaculo à frente robot para)

-Seguidor de linha - a partir de array de sensores infravermelhos, o robot segue uma linha preta (3cm). Controlo realizado com PID (PD). Bifurcações são decididas pelo piloto com as setas da app

-Seguidor de paredes - brevemente...

___________

Todo o código é feito usando os registers do microcontrolador. Não há nenhuma função Arduino.

Desenvolvido em Visual Studio Code com a extensão PlatformIO (AVR)


![5](https://user-images.githubusercontent.com/78873689/116214672-d4818100-a73e-11eb-98e8-b776c66e3dfb.jpg)



