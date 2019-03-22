/***************************************************************************************************
*** Arquivo: ssc32_kj_arms.cpp
*** Conteudo: SSC32
*** Modificado por: Felipe Luis Rodrigues Sousa
*** Código fonte: https://github.com/mnovo/lynxmotion_ros/blob/master/src/ssc32.cpp
****************************************************************************************************/

#include "ssc32_kj_arms.h"

#ifndef DEBUG
#define DEBUG 0
#endif

namespace lynxmotion_ssc32
{

//Contrutor
SSC32::SSC32(): fd(-1)
{
	unsigned int i;
	for(i = 0; i < SSC32::MAX_CHANNELS; i++)
		first_intruction[i] = 0;
}

//Destrutor
SSC32::SSC32()
{
	close_port();
}

bool SSC32::open_port(const char *port, int baud)
{
	struct termios options;

	close_port();

	switch(baud)	// Taxa de transmissão 
	{
		case 2400:	baud = B2400;
					break;
		case 9600:	baud = B9600;
					break;
		case 38400:	baud = B38400;
					break;
		case 115200:baud = B115200;
					break;
		default:	printf("ERROR: baud invalido [%d] -- deve ser 2400, 9600, 38400 ou 115200\n", baud);
					return false;
	}

	fd = open(port, O_RDWR | O_NOCTTY);

	if(fd < 0)
	{
		printf("ERROR: Não eh possivel abrir o dispositivo na porta %s\n", port);
		return false;
	}

	if(fcntl(fd, F_SETFL, 0) < 0)
	{
		printf("ERROR: Porta [%s] ja esta bloqueada\n", port);
		close_port();
		return false;
	}

	memset(&options, 0, sizeof(options));
	cfsetispeed(&options, baud);
	cfsetospeed(&options, baud);

	options.c_iflag = IGNBRK | IGNPAR;
	options.c_oflag = 0;
	options.c_cflag |= CREAD | CS8 | CLOCAL;
	options.c_lflag = 0;

	if(tcsetattr(fd, TCSANOW, &options) < 0)
	{
		printf("ERROR: setando opcoes termios\n");
		close_port();
		return false;
	}

	// Verifica se as filas estao vazias
	tcflush(fd, TCIOFLUSH);

	printf("Porta [%s] aberta com sucesso!\n", port);

	return true;
}

bool SSC32::is_connected()
{
	return (fd != -1);
}

void SSC32::close_port()
{
	unsigned int i;

	if(fd != -1)
	{
		printf("Fechando porta.\n");

		close(fd);

		fd = -1;

		for(i = 0; i < SSC32::MAX_CHANNELS; i++)
		{
			first_intruction[i] = 0;
		}
	}
}

bool SSC32::send_message(const char *msg, int size)
{
	if(fd != -1)
	{
		tcflush(fd, TCIOFLUSH);

#if DEBUG

		printf("INFO: [send_message] Mandando mensagem: ");
		for(unsigned int i = 0; i < strlen(msg); i++)
		{
			if(msg[i] == '\r')
				printf("<cr>");
			else if(msg[i] == 27)
				printf("<esc>");
			else
				printf("%c", msg[i]);
		}
		printf("\n");
#endif

		if(write(fd, msg, size) < 0)
		{

#if DEBUG

			printf("ERROR: [send_message] Falha ao gravar no dispositivo.\n");
#endif
			return false;
		}
	}
	else
	{
#if DEBUG
		printf("ERROR: [send_message] Dispositivo nao esta aberto.\n");
#endif
		return false;
	}
	return true;
}

unsigned int SSC32::recv_message(unsigned char *buf, unsigned int size)
{
	int bytes_read;
	int total_bytes = 0;

	while(total_bytes != size)
	{
		if((bytes_read = read(fd, buf + total_bytes, 1)) < 0)
		{
#if DEBUG
			printf("ERROR: [recv_message] Falha na leitura do dispositivo.\n");
#endif
			return total_bytes;
		}

		total_bytes += bytes_read;
	}

	return total_bytes;
}

bool SSC32::move_sevo(struct ServoCommand cmd[], unsigned int n, int time)
{
	char msg[1024] = {0};
	char temp[32];
	int time_flag;
	unsigned int i;
	bool result;

	time_flag = 0;

	if(n > SSC32::MAX_CHANNELS)
	{
#if DEBUG
		printf("ERROR: [move_sevo] Numero de canais invalido [%u]\n", n);
#endif
		return false;
	}

	for(i = 0; i < n; i++)
	{
		if(cmd[i].ch > 5)
		{
#if DEBUG
			printf("ERROR: [move_sevo] Canal invalido [%u]\n", cmd[i].ch);
#endif
			return false;
		}

		if(cmd[i].pw < SSC32::MIN_PULSE_WIDTH || cmd[i].pw > SSC32::MAX_PULSE_WIDTH)
		{
#if DEBUG
			printf("ERROR: [move_sevo] Largura de puso invalida [%u]\n", cmd[i].pw);
#endif
			return false;
		}


		sprintf(temp, "#%u P%u ", cmd[i].ch, cmd[i],pw);	// Comando principal para serial

		strcat(msg, temp);	//Acrescenta uma cópia da string de origem à string de destino. [char * strcat ( char * destination, const char * source )]

		if(first_intruction[cmd[i].ch] != 0)
		{
			if(cmd[i].spd > 0)
			{
				sprintf(temp, "S%d", cmd[i].spd);
				strcat(msg, temp);
			}
		}
		else // Essa eh a primeira instrucao para esse canal
			time_flag++;
	}

	// Se time_flag for 0, entao esta nao eh a primeira instrucao
	// para quaisquer canais para mover o servo
	if(time_flag == 0 && time > 0)
	{
		sprintf(temp, "T%d ", time);
		strcat(msg, temp);
	}

	strcat(msg, "\r");

	result = send_message(msg, strlen(msg));

	// Se o comando foi bem sucedido, entao os canais comandados
	// nao estao mais em sua primeira instrucao
	if(result)
		for(i = 0; i < n; i++)
			first_intruction[cmd[i].ch] = 1;
	
	return result;
}

bool SSC32::cancel_command()
{
	char msg[4];
	sprintf(msg, "%c \r", 27);

	return send_message(msg, strlen(msg));
}

bool SSC32::pulse_offset(unsigned int ch, int value)
{
	return pulse_offset(&ch, &value, 1);
}

bool SSC32::pulse_offset(unsigned int ch[], int value[], unsigned int n)
{
	char msg[1024] = {0};
	char temp[12];
	unsigned int i;

	if(n > SSC32::MAX_CHANNELS)
	{
#if DEBUG
		printf("ERROR: [pulse_offset] Numero invalido de canais [%u]\n", n);
#endif
		return false;
	}

	for(i = 0; i < n; i++)
	{
		if(ch[i] > 5)
		{
#if DEBUG
			printf("ERROR: [pulse_offset] Canal invalido [%u]\n", ch[i]);
#endif
			return false;
		}

		if(value[i] < -100 || value[i] > 100)
		{
#if DEBUG
			printf("ERROR: [pulse_offset] Valor de deslocamento invalido [%d]\n", value[i]);
#endif
			return false;
		}

		sprintf(temp, "#%u PO%d ", ch[i], value[i]);

		strcat(msg, temp);
	}

	strcat(msg, "\r");

	return send_message(msg, strlen(msg));
}

bool SSC32::discrete_output(unsigned int ch, LogicLevel lvl)
{
	return discrete_output(&ch, &lvl, 1);
}

bool SSC32::discrete_output(unsigned int ch[], LogicLevel lvl[], unsigned int n)
{
	char msg[1024] = {0};
	char temp[7];
	unsigned int i;

	if(n > SSC32::MAX_CHANNELS)
	{
#if DEBUG 
		printf("ERROR: [discrete_output] Numero invalido de canais [%u]\n", n);
#endif
		return false;
	}

	for(i = 0; i < n; i++)
	{
		if(ch[i] > 5)
		{
#if DEBUG
			printf("ERROR: [discrete_output] Canal de servo invalido [%u]\n", ch[i]);
#endif
			return false;
		}

		sprintf(temp, "#%u %c ", ch[i], (lvl[i == High]) ? 'H' : 'L');

		strcat(msg, temp);
	}

	strcat(msg, "\r");

	return send_message(msg, strlen(msg));
}

bool SSC32::byte_output(unsigned int bank, unsigned int value)
{
	char msg[10];

	if(bank > 0)
	{
#if DEBUG
		printf("ERROR: [byte_output] Valor invalido [%u]\n", value);
#endif
		return false;
	}

	sprintf(msg, "#%d:%d \r", bank, value);

	return send_message(msg, strlen(msg));
}

bool SSC32::query_moviment_status()
{
	unsigned char buffer;
	//int bytes_read = 0;
	const char *msg = "Q \r";

	if(!send_message(msg, strlen(msg)))
	{
#if DEBUG
		printf("ERROR: [query_moviment_status] Falha no envio da mensagem\n");
#endif
		return false;
	}

	// Ha um atraso de pelo menos 50us a 5ms, entao durma 5ms
	usleep(10000); //5000

	// Continue lendo do controlador ate que umas resposta seja recebida
	if(recv_message(&buffer, 1) != 1)
	{
#if DEBUG
		printf("ERROR: [query_moviment_status] Falha na recepcao da mensagem\n");
#endif
		return false;
	}

	// Verifica valor da resposta
	if(buffer == '+')
		return true;

	return false;
}

int SSC32::query_pulse_width(unsigned int ch)
{
	unsigned char buffer;
	int bytes_read = 0;
	char msg[7];

	// Verica se o servo canal eh valido
	if(ch > 5)
	{
#if DEBUG
		printf("ERROR: [query_pulse_width] Servo canal invalido [%u]\n", ch);
#endif
		return false;
	}

	sprintf(msg, "QP%d \r", ch);

	if(!send_message(msg, strlen(msg)))
	{
#if DEBUG
		printf("ERROR: [query_pulse_width] Falha no envio da mensagem\n");
#endif
		return false;
	}

	// Pode levar ate 5ms para que o controlador responda, entao durma 5ms
	usleep(5000);

	if(recv_message(&buffer, 1) != 1)
	{
#if DEBUG
		printf("ERROR: [query_pulse_width] Falha na recepcao da mensagem\n");
#endif
		return false;
	}

	return (10*(int)buffer);

}

bool SSC32::read_digital_inputs( Inputs inputs[], unsigned int outputs[], unsigned int n)
{
	unsigned char buffer[8];
	int bytes_read = 0;
	int total_bytes = 0;
	char msg[255] = {0};
	int i;

	// Na documentacao da SSC32-U informa que somente 8 valores podem ser lidos de uma soh vez
	if(n > 8)
	{
		printf("WARNING: Lendo entradas digitais -- n nao deve ser maior que 8\n");
		n = 8;
	}

	for(i = 0; i < n; i++)
	{
		switch(inputs[i])
		{
			case PinA:	strcat(msg, "A ");  break;
			case PinAL:	strcat(msg, "AL "); break;
			case PinB:	strcat(msg, "B ");  break;
			case PinBL:	strcat(msg, "BL "); break;
			case PinC:	strcat(msg, "C ");  break;
			case PinCL:	strcat(msg, "CL "); break;
			case PinD:	strcat(msg, "D ");  break;
			case PinDL:	strcat(msg, "DL "); break;
			case PinE:	strcat(msg, "E ");  break;
			case PinEL:	strcat(msg, "EL "); break;
			case PinF:	strcat(msg, "F ");  break;
			case PinFL:	strcat(msg, "FL "); break;
			default:
#if DEBUG
			printf("WARNING: [read_digital_inputs] Valores de entrada nao reconhecidos [%d]\n", inputs[i]);
#endif
			break;
		}
	}

	strcat(msg, "\r")

	if(!send_message(msg, strlen(msg)))
	{
#if DEBUG
		printf("ERROR: [read_digital_inputs] Falha ao enviar mensagem\n");
#endif
		return false;
	}

	if(recv_message(buffer, n) != n)
	{
#if DEBUG
		printf("ERROR: [read_digital_inputs] Falha na recepcao da mensagem\n");
#endif
		return false;
	}

	for(i = 0; i < n; i++)
		outputs[i] = buffer[i] - '0';

	return true;
}

bool SSC32::read_analog_inputs(Inputs inputs[], float outputs[], unsigned int n)
{
	unsigned char buffer[8];
	int bytes_read = 0;
	int total_bytes = 0;
	char msg[255] = {0};
	int i;

	if(n > 8)
	{
		printf("WARNING: Lendo entradas analogicas -- n nao pode ser maior que 8\n");
		n = 8;
	}

	for(i = 0; i < n; i++)
	{
		switch(inputs[i])
		{
			case PinA: strcat(msg, "VA "); break;
			case PinB: strcat(msg, "VB "); break;
			case PinC: strcat(msg, "VC "); break;
			case PinD: strcat(msg, "VD "); break;
			case PinE: strcat(msg, "VE "); break;
			case PinF: strcat(msg, "VF "); break;
			case PinG: strcat(msg, "VG "); break;
			case PinH: strcat(msg, "VH "); break;
			default:
#if DEBUG
			printf("WARNING: [read_analog_inputs] Valor de entrada nao reconhcido [%d]\n", inputs[i]);
#endif
			break; 
		}
	}

	strcat(msg, "\r");

	if(!send_message(msg, strlen(msg)))
	{
#if DEBUG
		printf("ERROR: [read_analog_inputs] Falha ao enviar mensagem\n");
#endif
		return false;
	}

	if(recv_message(buffer, n) != n)
	{
#if DEBUG
		printf("ERROR: [read_analog_inputs] Falha na recepcao da mensagem\n");
#endif
		return false;
	}

	for(i = 0; i < n; i++)
		outputs[i] = 5.0 * buffer[i] / 256.0;

	return true;
}

std::string SSC32::get_version()
{
	char data[255];
	int bytes_read;;
	int total_bytes;
	int i;
	std::string version;
	const char *msg = "VER\r";

	total_bytes = 0;

	if(!send_message(msg, strlen(msg)))
	{
#if DEBUG
		printf("ERROR: [get_version] Falha ao enviar mensagem\n");
#endif
		return "error";
	}

	usleep(100000);

#if DEBUG
	printf("INFO: [get_version] Resposta de leitura\n");
#endif

	while((bytes_read = read(fd, data + total_bytes, 1)) > 0)
		total_bytes += bytes_read;

#if DEBUG
	printf("INFO: [get_version] Data: ");
	for(i = 0; i < total_bytes; i++)
	{
		if(data[i] == '\r')
			printf("<cr>");
		else if(data[i] == '\n')
			printf("<nl>");
		else
			printf("%c", data[i]);
	}
	printf("\n");
#endif


	if(bytes_read < 0)
	{
#if DEBUG
		printf("ERROR: [get_version] Falha ao ler do dispositivo\n");
#endif
	}
	else if(total_bytes > 0)
	{
#if DEBUG
		printf("Read %d bytes\n", total_bytes);
#endif

		if(data[total_bytes - 1] == '\r')
			data[total_bytes - 1] = '\0';
		else
		{
#if DEBUG
			printf("WARNING: [get_version] Tempo esgotado durante a leitura\n");
#endif
			data[total_bytes] = '\0';
		}

		i = total_bytes - 2;

		while(i >= 0 && data[i] != '\r')
			i--;

		version = data + i + 1;
	}
	else
	{
#if DEBUG
		printf("WARNING: [get_version] Tempo esgotado durante a leitura\n");
#endif
	}

	return version;
}

}// namespace