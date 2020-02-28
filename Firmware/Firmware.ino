/*
Modbus serial - RTU Slave Arduino Sketch
Este exemplo é de domínio público
Testado na IDE 1.0.1
Baseado na biblioteca de Juan Pablo Zometa : jpmzometa@gmail.com
http://sites.google.com/site/jpmzometa/
and Samuel Marco: sammarcoarmengol@gmail.com
and Andras Tucsni.
As funções do protocolo MODBUS implementadas neste código:
3 - Read holding registers;
This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
The functions included here have been derived from the
Modicon Modbus Protocol Reference Guide
which can be obtained from Schneider at www.schneiderautomation.com.
This code has its origins withpaul@pmcrae.freeserve.co.uk (http://www.pmcrae.freeserve.co.uk)
who wrote a small program to read 100 registers from a modbus slave.
*/
/*
* configure_mb_slave(baud, parity, tx_en_pin)
*
* configuração dos parametros da porta serial.
*
* baud: taxa de transmissão em bps (valores típicos entre 9600, 19200... 115200)
* parity: seta o modo de paridade:
* 'n' sem paridade (8N1); 'e' paridede impar (8E1), 'o' paridade par (8O1).
* tx_en_pin: pino do arduino que controla a transmissão/recepção em uma linha RS485.
* 0 or 1 desliga esta função (para rede RS232)
* >2 para uma rede multiponto.
*/
void configure_mb_slave(long baud, char parity, char txenpin);
/*
* update_mb_slave(slave_id, holding_regs_array, number_of_regs)
*
* verifica se há qualquer pedido válido do mestre modbus. Se houver,
* executa a ação solicitada
*
* slave: endereço do escravo (arduino) (1 to 127)
* regs: uma matriz com os holding registers. Eles começam no endereço 1 (mestre ponto de
vista)
* Regs_size: número total de holding registers.
* Retorna: 0 se não houver pedido do mestre,
* NO_REPLY (-1) se nenhuma resposta é enviada para o mestre
* Caso um código de exceção (1 a 4) em algumas exceções de modbus
* O número de bytes enviados como resposta (> 4) se OK.
*/
int update_mb_slave(unsigned char slave, int *regs, unsigned int regs_size);

/* Parâmetros Modbus RTU de comunicação, o Mestre e os escravos devem usar os mesmos
parâmetros */

enum
{
  COMM_BPS = 9600, /* baud rate */
  MB_SLAVE = 1,    /* endereço do escravo modbus */
  PARITY = 'n', /* paridade */
  TXEN = 6      /* para utilizar com rede RS-485 */
};

/* registros do escravo (holding registers)
*
* Aqui ficam ordenados todos os registros de leitura e escrita
* da comunicação entre o mestre e o escravo (SCADA e arduino)
*
*/
enum
{
  MB_PWM_1, 
  MB_AD_1, 
  MB_PWM_1_C,     
  MB_AD_1_C,
  MB_REGS    /* número total de registros do escravo */
};
int regs[MB_REGS];
int contador = 0;

//Pinagem
int ledPin = 13;    //PB5
int PullPin = 3;    //PD3
int AnalogPin = A0; //PC0

unsigned long wdog = 0;        /* watchdog */
unsigned long tprev = 0;       /* tempo anterior do último comando*/

volatile byte pulseCount = 0;  
unsigned long oldTime = 0;

void setup()
{
  /* configura cominicação modbus
  * 9600 bps, 8N1, RS485 network */
  configure_mb_slave(COMM_BPS, PARITY, TXEN);
  pinMode(ledPin, OUTPUT);
}
void loop()
{
  digitalWrite(ledPin, HIGH);
  /* verifica se há solicitações do mestre */
  update_mb_slave(MB_SLAVE, regs, MB_REGS); //essa funcao nao aguarda a chegada, mas faz a verificacao recorentenmente
    
}

void leitura_sensores(){//Funcao chamada na leitura de registradores
    digitalWrite(ledPin, LOW);
    //Sensor de vazao
    attachInterrupt(digitalPinToInterrupt(PullPin), pulseCounter, FALLING);
    oldTime = millis();
    while((millis() - oldTime) < 1000)    // Aguarda 1 segundo, ou usar delay ??
    {
      if(oldTime > millis()) //O contador resetou
        break;
    }
    detachInterrupt(digitalPinToInterrupt(PullPin));
    regs[MB_PWM_1] = pulseCount;
    regs[MB_PWM_1_C] = pulseCount;
    
    //Sensor de pressao
    int ad = analogRead(AnalogPin);
    regs[MB_AD_1] = ad;
    regs[MB_AD_1_C] = ad;

    pulseCount = 0;
}

void pulseCounter()
{
  // Increment the pulse counter
  pulseCount++;
}
/****************************************************************************
* INICIO DAS FUNÇÕES ESCRAVO Modbus RTU
****************************************************************************/
/* variaveis globais */
unsigned int Txenpin = TXEN; /*Definir o pino usado para colocar o driver
RS485 em modo de transmissão, utilizado
somente em redes RS485 quando colocar em 0
ou 1 para redes RS232 */
/* Lista de códigos de função modbus suportados. Se você implementar um novo, colocar o seu
código de função aqui! */
enum
{
  FC_READ_REGS = 0x03 //Read contiguous block of holding register (Ler um blococontíguo de registos)
};
/* Funções suportadas. Se você implementar um novo, colocar seu código em função nessa
matriz! */
const unsigned char fsupported[] = {
    FC_READ_REGS
};
/* constantes */
enum
{
  MAX_READ_REGS = 0x7D,
  MAX_WRITE_REGS = 0x7B,
  MAX_MESSAGE_LENGTH = 256
};
enum
{
  RESPONSE_SIZE = 6,
  EXCEPTION_SIZE = 3,
  CHECKSUM_SIZE = 2
};
/* código de exceções */
enum
{
  NO_REPLY = -1,
  EXC_FUNC_CODE = 1,
  EXC_ADDR_RANGE = 2,
  EXC_REGS_QUANT = 3,
  EXC_EXECUTE = 4
};
/* posições dentro da matriz de consulta / resposta */
enum
{
  SLAVE = 0,
  FUNC,
  START_H,
  START_L,
  REGS_H,
  REGS_L,
  BYTE_CNT
};
/*
CRC
INPUTS:
buf -> Matriz contendo a mensagem a ser enviada para o controlador mestre.
start -> Início do loop no crc do contador, normalmente 0.
cnt -> Quantidade de bytes na mensagem a ser enviada para o controlador mestre
OUTPUTS:
temp -> Retorna byte crc para a mensagem.
COMMENTÁRIOS:
Esta rotina calcula o byte crc alto e baixo de uma mensagem.
Note que este CRC é usado somente para Modbus, não em Modbus PLUS ou TCP.
****************************************************************************/
unsigned int crc(unsigned char *buf, unsigned char start,
                 unsigned char cnt)
{
  unsigned char i, j;
  unsigned temp, temp2, flag;
  temp = 0xFFFF;
  for (i = start; i < cnt; i++)
  {
    temp = temp ^ buf[i];
    for (j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp = temp >> 1;
      if (flag)
        temp = temp ^ 0xA001;
    }
  }
  /* Inverter a ordem dos bytes. */
  temp2 = temp >> 8;
  temp = (temp << 8) | temp2;
  temp &= 0xFFFF;
  return (temp);
}
/***********************************************************************
*
* As seguintes funções constroem o frame de
* um pacote de consulta modbus.
*
***********************************************************************/
/*
* Início do pacote de uma resposta read_holding_register
*/
void build_read_packet(unsigned char slave, unsigned char function,
                       unsigned char count, unsigned char *packet)
{
  packet[SLAVE] = slave;
  packet[FUNC] = function;
  packet[2] = count * 2;
}
/** Início do pacote de uma resposta preset_multiple_register
*/
void build_write_packet(unsigned char slave, unsigned char function,
                        unsigned int start_addr,
                        unsigned char count,
                        unsigned char *packet)
{
  packet[SLAVE] = slave;
  packet[FUNC] = function;
  packet[START_H] = start_addr >> 8;
  packet[START_L] = start_addr & 0x00ff;
  packet[REGS_H] = 0x00;
  packet[REGS_L] = count;
}

/*
* Início do pacote de uma resposta excepção
*/
void build_error_packet(unsigned char slave, unsigned char function,
                        unsigned char exception, unsigned char *packet)
{
  packet[SLAVE] = slave;
  packet[FUNC] = function + 0x80;
  packet[2] = exception;
}
/*************************************************************************
*
* modbus_query( packet, length)
*
* Função para adicionar uma soma de verificação para o fim de um pacote.
* Por favor, note que a matriz pacote deve ser de pelo menos 2 campos mais do que
* String_length.
**************************************************************************/
void modbus_reply(unsigned char *packet, unsigned char string_length)
{
  int temp_crc;
  temp_crc = crc(packet, 0, string_length);
  packet[string_length] = temp_crc >> 8;
  string_length++;
  packet[string_length] = temp_crc & 0x00FF;
}
/***********************************************************************
*
* send_reply( query_string, query_length )
*
* Função para enviar uma resposta a um mestre Modbus.
* Retorna: o número total de caracteres enviados
************************************************************************/
int send_reply(unsigned char *query, unsigned char string_length)
{
  unsigned char i;
  if (Txenpin > 1)
  { // coloca o MAX485 no modo de transmissão
    UCSR0A = UCSR0A | (1 << TXC0);
    digitalWrite(Txenpin, HIGH);
    delayMicroseconds(3640); // aguarda silencio de 3.5 caracteres em 9600bps
  }
  modbus_reply(query, string_length);
  string_length += 2;
  for (i = 0; i < string_length; i++)
  {
    Serial.write(byte(query[i]));
  }
  if (Txenpin > 1)
  { // coloca o MAX485 no modo de recepção
    while (!(UCSR0A & (1 << TXC0)))
      ;
    digitalWrite(Txenpin, LOW);
  }
  return i; /* isso não significa que a gravação foi bem sucedida */
}
/***********************************************************************
*
* receive_request( array_for_data )
*
* Função para monitorar um pedido do mestre modbus.
*
* Retorna: Número total de caracteres recebidos se OK
* 0 se não houver nenhum pedido
* Um código de erro negativo em caso de falha
***********************************************************************/
int receive_request(unsigned char *received_string)
{
  int bytes_received = 0;
  /* FIXME: não Serial.available esperar 1.5T ou 3.5T antes de sair do loop? */
  while (Serial.available())
  {
    received_string[bytes_received] = Serial.read();
    bytes_received++;
    if (bytes_received >= MAX_MESSAGE_LENGTH)
      return NO_REPLY; /* erro de porta */
  }
  return (bytes_received);
}
/*********************************************************************
*
* modbus_request(slave_id, request_data_array)
*
* Função que é retornada quando o pedido está correto
* e a soma de verificação está correto.
* Retorna: string_length se OK
* 0 se não
* Menos de 0 para erros de exceção
*
* Nota: Todas as funções usadas para enviar ou receber dados via
* Modbus devolver esses valores de retorno.
*
**********************************************************************/
int modbus_request(unsigned char slave, unsigned char *data)
{
  int response_length;
  unsigned int crc_calc = 0;
  unsigned int crc_received = 0;
  unsigned char recv_crc_hi;
  unsigned char recv_crc_lo;
  response_length = receive_request(data);
  if (response_length > 0)
  {
    crc_calc = crc(data, 0, response_length - 2);
    recv_crc_hi = (unsigned)data[response_length - 2];
    recv_crc_lo = (unsigned)data[response_length - 1];
    crc_received = data[response_length - 2];
    crc_received = (unsigned)crc_received << 8;
    crc_received =
        crc_received | (unsigned)data[response_length - 1];
    /*********** verificar CRC da resposta ************/
    if (crc_calc != crc_received)
    {
      return NO_REPLY;
    }
    /* verificar a ID do escravo */
    if (slave != data[SLAVE])
    {
      return NO_REPLY;
    }
  }
  return (response_length);
}
/*********************************************************************
*
* validate_request(request_data_array, request_length, available_regs)
*
* Função para verificar se o pedido pode ser processado pelo escravo.
*
* Retorna: 0 se OK
* Um código de exceção negativa em caso de erro
*
**********************************************************************/
int validate_request(unsigned char *data, unsigned char length,
                     unsigned int regs_size)
{
  int i, fcnt = 0;
  unsigned int regs_num = 0;
  unsigned int start_addr = 0;
  unsigned char max_regs_num;
  /* verificar o código de função */
  for (i = 0; i < sizeof(fsupported); i++)
  {
    if (fsupported[i] == data[FUNC])
    {
      fcnt = 1;
      break;
    }
  }
  if (0 == fcnt)
    return EXC_FUNC_CODE;

  /* Para as funções de leitura / escrita de registros, este é o intervalo. */
  regs_num = ((int)data[REGS_H] << 8) + (int)data[REGS_L];
  /* verifica a quantidade de registros */
  if (FC_READ_REGS == data[FUNC])
    max_regs_num = MAX_READ_REGS;
  if ((regs_num < 1) || (regs_num > max_regs_num))
    return EXC_REGS_QUANT;
  /* verificará a quantidade de registros, endereço inicial é 0 */
  start_addr = ((int)data[START_H] << 8) + (int)data[START_L];
  if ((start_addr + regs_num) > regs_size)
    return EXC_ADDR_RANGE;
  return 0; /* OK, sem exceção */
}
/************************************************************************
*
* write_regs(first_register, data_array, registers_array)
*
* escreve nos registradores do escravo os dados em consulta,
* A partir de start_addr.
*
* Retorna: o número de registros escritos
************************************************************************/
int write_regs(unsigned int start_addr, unsigned char *query, int *regs)
{
  int temp;
  unsigned int i;
  for (i = 0; i < query[REGS_L]; i++)
  {
    /* mudar reg hi_byte para temp */
    temp = (int)query[(BYTE_CNT + 1) + i * 2] << 8;
    /* OR com lo_byte */
    temp = temp | (int)query[(BYTE_CNT + 2) + i * 2];
    regs[start_addr + i] = temp;
  }
  return i;
}
/************************************************************************
*
* read_holding_registers(slave_id, first_register, number_of_registers,
* registers_array)
*
* lê os registros do escravo e envia para o mestre Modbus
*
*************************************************************************/
int read_holding_registers(unsigned char slave, unsigned int start_addr, unsigned char reg_count, int *regs)
{
  unsigned char function = 0x03; /* Função 03: Read Holding Registers */
  int packet_size = 3;
  int status;
  unsigned int i;
  unsigned char packet[MAX_MESSAGE_LENGTH];

  leitura_sensores(); /* Inseri esta funcao aqui para nao realizar a leitura no loop*/
  
  build_read_packet(slave, function, reg_count, packet);
  for (i = start_addr; i < (start_addr + (unsigned int)reg_count);
       i++)
  {
    packet[packet_size] = regs[i] >> 8;
    packet_size++;
    packet[packet_size] = regs[i] & 0x00FF;
    packet_size++;
  }
  status = send_reply(packet, packet_size);
  return (status);
}
void configure_mb_slave(long baud, char parity, char txenpin)
{
  Serial.begin(baud);
  switch (parity)
  {
  case 'e': // 8E1
    UCSR0C |= ((1 << UPM01) | (1 << UCSZ01) | (1 << UCSZ00));
    // UCSR0C &= ~((1<<UPM00) | (1<<UCSZ02) | (1<<USBS0));
    break;
  case 'o': // 8O1
    UCSR0C |= ((1 << UPM01) | (1 << UPM00) | (1 << UCSZ01) | (1 << UCSZ00));
    // UCSR0C &= ~((1<<UCSZ02) | (1<<USBS0));
    break;
  case 'n': // 8N1
    UCSR0C |= ((1 << UCSZ01) | (1 << UCSZ00));
    // UCSR0C &= ~((1<<UPM01) | (1<<UPM00) | (1<<UCSZ02) | (1<<USBS0));
    break;
  default:
    break;
  }
  if (txenpin > 1)
  {                    // pino 0 & pino 1 são reservados para RX/TX
    Txenpin = txenpin; /* definir variável global */
    pinMode(Txenpin, OUTPUT);
    digitalWrite(Txenpin, LOW);
  }
  return;
}
/*
* update_mb_slave(slave_id, holding_regs_array, number_of_regs)
*
* verifica se há qualquer pedido válido do mestre modbus. Se houver,
* executa a ação solicitada
*/
unsigned long Nowdt = 0;
unsigned int lastBytesReceived;
const unsigned long T35 = 5;
int update_mb_slave(unsigned char slave, int *regs,
                    unsigned int regs_size)
{
  unsigned char query[MAX_MESSAGE_LENGTH];
  unsigned char errpacket[EXCEPTION_SIZE + CHECKSUM_SIZE];
  unsigned int start_addr;
  int exception;
  int length = Serial.available();
  unsigned long now = millis();
  if (length == 0)
  {
    lastBytesReceived = 0;
    return 0;
  }
  if (lastBytesReceived != length)
  {
    lastBytesReceived = length;
    Nowdt = now + T35;
    return 0;
  }
  if (now < Nowdt)
    return 0;
  lastBytesReceived = 0;
  length = modbus_request(slave, query);
  if (length < 1)
    return length;
  exception = validate_request(query, length, regs_size);
  if (exception)
  {
    build_error_packet(slave, query[FUNC], exception,
                       errpacket);
    send_reply(errpacket, EXCEPTION_SIZE);
    return (exception);
  }
  start_addr = ((int)query[START_H] << 8) +
               (int)query[START_L];
  switch (query[FUNC])
  {
  case FC_READ_REGS:
    return read_holding_registers(slave,
                                  start_addr,
                                  query[REGS_L],
                                  regs);
    break;
  }
}
