# Kyle XY and Jesse XX

## Manipulador
O manipulador é um AL5A da Lynxmotion e é controlado usando uma SSC-32 (ATMEGA168). 

A SSC-32 é uma controladora de servos que pode controlar até 32 servos simultaneamente. Nesse proejeto a placa será conectada a um PC104 via cabos serial DB9. 

A SSC-32 pode ser usada com apenas uma fonte fazendo com que a tensão da placa e a dos servos seja a mesma de 6V, mas devido a resistência das conexões, ou a outros fatores na eletrônica, a tensão pode cair e dar resetar a SSC-32. Sabendo que a placa funciona melhor com duas fontes, assim será feito nesse projeto.

### Imagem da SSC-32:
![alt text](https://drive.google.com/file/d/1JhQsBEOVd2AlfN-__LYJlYWf3SMs5ELt/view?usp=sharing)
