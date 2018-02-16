# Kyle XY and Jesse XX

## Manipulador
O manipulador é um AL5A da Lynxmotion e é controlado usando uma SSC-32 (ATMEGA168). 

A SSC-32 é uma controladora de servos que pode controlar até 32 servos simultaneamente. Nesse proejeto a placa será conectada a um PC104 via cabos serial DB9. 

A SSC-32 pode ser usada com apenas uma fonte fazendo com que a tensão da placa e a dos servos seja a mesma de 6V, mas devido a resistência das conexões, ou a outros fatores na eletrônica, a tensão pode cair e dar resetar a SSC-32. Sabendo que a placa funciona melhor com duas fontes, assim será feito nesse projeto.

Há várias formas de comandos para controlar a SSC-32, segue abaixo as mais usadas:

> #5 P1600 T1000

Nesse formato será enviado a posição 1600 ao servo 5 e o movimento terá duração de 1s.

> #5 P1600 #10 P750 T2500

O comando acima move os dois servos começando e terminando o movimento no mesmo instante. O movimento dura 2.5s.

> #5 P1600 S750

Esse comando move o servo 5 para a posição 1600 com uma taxa de 750uS por segundo. Para que se entenda melhor, com uma velocidade de 100uS por segundos o servo moverá 90° em 10s.

### Imagem da SSC-32:
![alt text](https://github.com/lara-unb/Kyle_XY_Jesse_XX/blob/master/doc/img/SSC32_0.jpg?raw=true)


O manipulador tem 6 graus de liberdade.

#### Servos Do Manipulador

>>>	Base(1º grau):	Hitec HS- 755HB

>>>	(2º grau):		Hitec HS-755HB

>>>	(3º grau):		Hitec HS-485HB

>>>	(4º grau):		Hitec HS-422

>>>	(5º grau):		Hitec HS-485HB

>>>	Garra (6º grau):	Hitec HS-645MG

### Imagem do AL51:
![alt text](https://github.com/lara-unb/Kyle_XY_Jesse_XX/blob/master/doc/img/AL5A.jpg?raw=true)
