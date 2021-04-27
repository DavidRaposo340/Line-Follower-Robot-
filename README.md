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

Todo o código é feito usando os registers do microcontrolador. Não há nenhuma função do Arduino.

Desenvolvido em Visual Studio Code com a extensão PlatformIO (AVR)

![app](https://user-images.githubusercontent.com/78873689/116214622-c9c6ec00-a73e-11eb-80b9-3068d071aa8b.png)
![6](https://user-images.githubusercontent.com/78873689/116214660-d2b7bd80-a73e-11eb-9fff-e29c1b5a6521.jpg)
![1](https://user-images.githubusercontent.com/78873689/116214664-d3505400-a73e-11eb-8e22-883d5376b8c0.jpg)
![2](https://user-images.githubusercontent.com/78873689/116214665-d3e8ea80-a73e-11eb-8bed-e9f2992ca580.jpg)
![3](https://user-images.githubusercontent.com/78873689/116214666-d3e8ea80-a73e-11eb-99cf-b2680a9fb9c6.jpg)
![4](https://user-images.githubusercontent.com/78873689/116214669-d4818100-a73e-11eb-8435-780b13e3ff6e.jpg)
![5](https://user-images.githubusercontent.com/78873689/116214672-d4818100-a73e-11eb-98e8-b776c66e3dfb.jpg)


