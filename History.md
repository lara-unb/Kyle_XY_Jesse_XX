#**History**

## Participantes: Felipe
## __21-09-2017__
	
>	Início. 

## __22-09-2017__

>	Desmontar Teco e limpar, separando peça a peça... 
	Reorganizar estrutura...

>>	### Garra:

		Garra Teco está faltando uma conexão em parafuzo - Normal;
		Parafuzo espanado no terceiro servo;
		Base do primeiro servo encontra-se frouxa;	DONE!

>>	### CPU:

		Desmontado parte a parte. Aparentemente está tudo inteiro, ou seja, muito provável de que estejam 
		funcionando cada compartimento;

>>	### Base:

		Motores traseiros tem uma diferença nas conexões em relação
		aos dianteiros. Apresenta uma conexão a mais com cabo quadruplo de cores amarelo, preto, verde e vermelho;

		Motor esquerdo dianteiro tem o fio quadruplo encaixado com o fio amarelo mais próximo do fio vermelho de alimentação;

		O esquerdo segue o mesmo esquema;


>>>	###_Situação Final: Demosntado componete a componente, limpos e separados. E passo a passo foi registrado em imagens e textos;_

## __30-09-2017__ 

>>	Feito testes no manipulador através de um microcontrolador _Teensy 3.2_, servo a servo; 

## __01-10-2017__
	
>	Organizar arquivos compactados para poder começar a trabalhar;

>>	Selecionado arquivos de pastas compactadas;
	Depois de analisar, foi escolhido os que contém mais informações. Dos backups, os mais recentes;

## __02-10-2017__

>> Usando o Teensy pode-se manipular através de um algoritmo mais de um servo motor;

>> Teve apenas o problema da força pra se levantar o braço; Servo 2 e 3;

## __03-10-2017__

>	Reunião com o Prof. Geovany

>>	Me abordou algumas informações sobre o funcionamento do Hardware e Softerware do Tico e Teco, explicando o funcionamento dos componentes e 		respostas esperadas por eles. Falou também sobre como poderia fazer o teste ligando-os. Explicou um pouco e deixou pra pequisar a respeito 		de SATA e PATA, que são os tipos de HDs;

## __04-10-2017__

>	Terminei a limpeza dos robôs e deixei eles pré-montados para começar a trabalhar na implementação dos Softerwares e funcionamento;

## __07-10-2017__ 

>	Liguei o Tico fazendo o teste do seu PC;
	Apresentou estar funcionando bem;
	Salvei a pasta Home do Tico em meu Laptop;

## __08-10-2017__

>	Liguei o Teco fazendo o teste em seu PC;
	Apresentou estar funcionando bem;
	Apenas iniciou o Xubuntu e ficou com a tela de início apenas com acesso pelo terminal. Mas pra acessar as pastas, apenas pelo terminal;
	

## Participantes: Gabriel e Felipe

##__23-10-2017 ... 27-10-2017__

>	(Gabriel) Aprendendo tudo que o Felipe descobriu e tentando entender o funcionamento do sistema com as PCB.

##__30-10-2017 ... 14-11-2017__
>	Duas reuniões com o professor Geovany sendo a primeira no dia 07 segunda no dia 14.
	Na primeira o houve uma introdução básica do projeto explicando melhor o funcionamento. 
	Foi passado como tarefa a análise do hardware geral pra decidir se era possível substituir as PCB's por uma SSC-32
	Na segunda foi decidido a substituição das PCB's pela SSC-32.
	Das 19:00hrs do dia 13 até as 04:00hrs do dia 14 ficamos aprendendo sobre os servos dos manipuladores e sobre a SSC-32. Testamos o funcionamento de tudo através de um Arduíno uno.  

##__15-11-2017__
>	Foi criado pelo professor Geovany o diretório no GitHub onde colocamos todas informações até o momento na branch Master.

##__15-01-2018 ... 26-01-2018__
>	Estudando sobre sobre o antigo funcionamento dos robôs, básico dos tutoriais de ROS.

##__29-01-2018 ... 31-01-2018__
>	Aprendendo sobre SSC-32 e manipuladores.

##__01-02-2018__
>	Tentativa e sucesso em comunicar SSC-32 via ROS com um notebook. Para isso foi usado ROS driver encontrado em https://github.com/smd-ros-devel/lynxmotion_ssc32.

## ... __15/02/2018__
>	SSC-32 não executava pulso completo até chegar a posição desejado. Problema resolvido: A placa não funciona muito bem com apenas uma fonte para ela e os servos.

##__16/02/2018__
>	Programa basico de mandar comandos via serial feito. 
