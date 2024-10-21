INSTRUCTIONS FOR SERIAL PORT PROTOCOL
=====================================

This folder contains the base code of the serial port protocol.

Project Structure
-----------------

- bin/: Compiled binaries.
- src/: Source code for the implementation of the link-layer and application layer protocols. Students should edit these files to implement the project.
- include/: Header files of the link-layer and application layer protocols. These files must not be changed.
- cable/: Virtual cable program to help test the serial port. This file must not be changed.
- main.c: Main file. This file must not be changed.
- Makefile: Makefile to build the project and run the application.
- penguin.gif: Example file to be sent through the serial port.

Instructions to Run the Project
-------------------------------

1. Edit the source code in the src/ directory.
2. Compile the application and the virtual cable program using the provided Makefile.
3. Run the virtual cable program (either by running the executable manually or using the Makefile target):
	$ sudo ./bin/cable_app
	$ sudo make run_cable

4. Test the protocol without cable disconnections and noise
	4.1 Run the receiver (either by running the executable manually or using the Makefile target):
		$ ./bin/main /dev/ttyS11 9600 rx penguin-received.gif
		$ make run_tx

	4.2 Run the transmitter (either by running the executable manually or using the Makefile target):
		$ ./bin/main /dev/ttyS10 9600 tx penguin.gif
		$ make run_rx

	4.3 Check if the file received matches the file sent, using the diff Linux command or using the Makefile target:
		$ diff -s penguin.gif penguin-received.gif
		$ make check_files

5. Test the protocol with cable disconnections and noise
	5.1. Run receiver and transmitter again
	5.2. Quickly move to the cable program console and press 0 for unplugging the cable, 2 to add noise, and 1 to normal
	5.3. Check if the file received matches the file sent, even with cable disconnections or with noise



# Apontamentos


#### features
- framing
	- packaging - dados vindos da upper layer são packed em frames
		- frames:
			- header
			- body
			- trailer
		- dados da application vão para a secção do body
		- são designados information frames
	- sincronização de frames - delimitation
		- inicio e fim dos frames são unicamente identificados para que os dados de receção possam estar sincronizados
			- abordagem principal é usar uma flag no início e no final dos frames
			- é preciso assegurar que o seu valor não aparece dentro do frame por acaso
				- mecanismo de stuffing
			- o tamanho dos frames devem estar implicitamente determinados
				- contar o numero de bytes entre sync flags
			- ou indicados explicitamente no campo do header
- establecimento da conexão e término
	- troca de mensagens específicas enviadas em frames de tamanho fixo
		- designados por supervision frames e tendo apenas campos de controlo sem user data
- frame numbering
	- um counter module-in no header dos frames para permitir a verificação da sequência correta de information frames e/ou a ocurrencia de duplicatas
- acknowledgement
	- todas as vezes que um frame é recebido sem erros e na sequência certa, um ackowledgement é enviado de volta para o sender
- error control - stop and wait ; go back n, selective repeat
	- utilização de time out timers para ativar a retransmissão de um não aknowledged frame
	- uso de negativo acknowledgement para pedir uma retransmissão de out-of-sequence frames ou frames com erro
	- verificacão de duplicatas que possam ocurrer devido a retransmissões
- flow control 

### Data Link Protocol
O protocolo a implementar combina características de protocolos existentes
- independente do tipo de user data a ser transferido
- transmissão organizada em frames, que podem ser de tres tipos:
	- Information
	- Supervision
	- Unnumbered
- Frames tem um header com um formato comum
	- apenas Information frames têm um campo para transportar user data 
		- um campo para transportar um packet gerado pela applicação, cujo conteudo não é processado pelo protocolo de data link
- A delimitação dos frames é feita por meio de uma sequencia especial de 8 bits - flag - e um byte stuffing que assegura que esse valor não vai aparecer a meio do frame
- Os frames são protegidos por um codigo de deteção de erros
	- Nos frames S e U ha uma protecao simples de frame visto que não transportam data
	- Nos frames I há uma dupla e independente proteção do header e do campo de dados (que permite a utilização de um header válido, mesmo se algum erro aparecer no campo de dados)
- A variante Stop and Wait é usada (?)

### Formato e tipo de frames

#### Supervision e Unnumbered Frames

F A C BCC1 F

- Flag - 0x7E
- Address Field
	- 0x03
		- comando enviado pelo transmitter ou resposta enviada pelo receiver
	- 0x01
		- comando enviado pelo receiver ou resposta enviada pelo transmitter
- Control Field que indica o tipo de supervision frame / message
	- SET (setup) - 0x03
		- enviado pelo transmissor para iniciar a conexão
	- DISC (disconnect) - 0x0B
		- indicar o encerramento da conexão
	- UA (unnumbered acknowledgement) - 0x07
		- confirmação da receção de um supervision frame válido
	- RR (receiver ready / positive ack)
		- RR0 - 0xAA
			- indicação enviada pelo receiver que está pronto para receber um information frame number 0
		- RR1 - 0xAB
			- indicação enviada pelo receiver que está pronto para receber um information frame number 1
	- REJ (reject / negative ack)
		- REJ0 - 0x54
			- indicação enviada pelo receiver que rejeita o information frame number 0
		- REJ1 -0x55
			- indicação enviada pelo receiver que rejeita o information frame number 1
- Campo de proteção para detetar a ocurrencia de erros no header (BCC1)
	- A XOR C
		- Campo que deteta a ocurrência de erros no header

#### Information Frames
F A C BCC1 D1 ... Dn BCC2 F

- Flag - 0x7E
	- igual
- Address Field
	- 0x03 e 0x01 igual tambem
- Control Field
	- same
- Information Field
- Independent Protection fields (BCC1 - header , BCC2 - data)
	- BCC1 é igual 
	- BCC2 é D1 xor D2 xor D3 ... XOR Dn

#### Packets e Frames
Na application layer
-  file para ser transmitido é fragmentado 
	- os fragmentos são encapsulados em data packets que são passados para o link layer um por um
- em adição aos data packets (que contem os fragmentos do file), o protocolo da aplicação usa pacotes de controlo

Na link layer, cada packet (data ou control) é carregado no data field de um I frame

O transmitter é a maquina que envia o file e o receiver é a máquina que receve o file
- assim apenas o transmitter transmite packets (data or control), ou seja apenas ele transmite I frames 

Ambos o Transmitter e o Receiver enviam e recebem frames (write or read frames into/from the serial line)


#### Frames - Delimitation and header

-  Control packets are also transported in I frames
- Todos os frames são delimitados por flags 0x7E
- Um frame pode ser iniciado com uma ou mais flags, que devem ser levadas em consideração pelo mecanismo de receção dos frames
- Os frames I, SET e DISC são designados Commands e o resto (UA, RR e REJ) são considerados Replies
- Frames têm um header com o seguinte formato
	- A - Address Field
	- C - Control Field
	- BCC - Block Check Character - deteção de erros baseado na geração de um octeto de modo a que haja um número par de 1s em cada posição, considerando todos os octetos protegidos pelo BCC (header ou data, conforme apropriado) e o próprio BCC (antes do stuffing)

#### Receção de frames
- Frames I, S ou U com header errado são ignorados sem qualquer ação
- Os data fields dos I frames são protegidos pelo seu próprio BCC (paridade par em cada bit dos octetos de dados e no BCC)
- Frames I recebidos sem erros detetados no header e data field são aceites para processamento
	- Se for um frame novom o data field é aceite (e passado para a application), e o frame tem que ser confirmado com RR
	- Se é um duplicado, o data field é descartado, mas o frame tem que ser tambem confirmado com RR
- Se frames sem erros de header detetados mas com outros erros detetados (pelo respetivo BCC) no data field, o data field é descartado, mas o control field pode ser usado para dar trigger a uma ação apropriada
	- Se for um frame novo, é conveniente fazer um pedido de retransmissão com REJ, que permite anticipar a ocurrencia do time-out no transmitter
	- Se é um duplicado, tem que ser confirmado com RR
- Frames I, SET e DISC são protegidos por um timer
	- No caso de um time-out, o numero maximo de tentativas de retransmissão tem que ser feito (valor configurável, por exemplo 3)

#### Transparency / Stuffing
- A transmissão entre os dois computadores é , neste trabalho, baseado numa tecnica chamada asynchronous transmission
	- Esta técnica é caracterizada pela transmissão de "caracteres" (curtas strings de bits, cujo numero pode ser configurado) delimitada por bits de start e stop
	- Alguns protocolos usam caracteres (words) de um codigo (ASCII p.e.) para delimitar e identificar os campos que constituem os frames e para suportar a execução do mecanismo de protocolo
		- Nestes protocolos, a transmissão de data de forma transparente (independentemente do código usado pelo protocolo) requer o uso de mecanismos de escape.
	- No protocolo a ser implementado não é paseado em nenhum uso de código, entao os characters transmitidos (8bits) devem ser interpretados como simples octetos (bytes). e qualquer uma das 256 possiveis combinações podem ocorrer
	- Para evitar um falso reconhecimento da flag dentro de um frame, é necessário um mecanismo que garanta transparencia
- No protocolo a ser implementado, o mecanismo usado em PPP é adotado, que usa o octeto de escape 0x7d
	- Se este octeto 0x7e aparecer dentro do frame, isto é, o padrão que corresponde à flag, o octeto é substituído pela sequência 0x7d 0x5e (octeto de escape seguido do resultado do "ou exclusivo" / xor do octeto substituído com o octeto 0x20)
	- Se o octeto 0x7d aparecer dentro do frame, ou seja, o escape octet, o octeto é substituído pela sequencia 0x7d 0x5d (octeto de escape seguido do resultado do xor do octeto substituido com o octeto 0x20)
	- Na gerção do BCC, apenas o octeto original é considerado (antes da operação do stuffing), mesmo que um octeto tenha que ser substituido pela sequencia de escape correspondente (incluindo o BCC)
	- A verificação do BCC é realizada em relação aos octetos originais, ou seja, após a operação inversa (destuffing) ter sido realizada, caso a substituição de qualquer um dos octetos especiais pela sequência de escape correspondente tenha ocorrido

![[Pasted image 20241019150404.png]]

Exemplode uma tipica frame sequence

#### Retransmissões
- Acknowledgement / Controlo de Erro
	- Stop-and-Wait:
		- um método de controlo de erro em que o transmissor aguarda uma confirmação do receiver antes de enviar o próximo frame. Isto garante que o transmissor saiba que os dados foram recebidos corretamente
-  Timer
	- Ativo depois de um frame I, SET, DISC
		- O timer é ativo assim que um frame I (Information), SET (Set) ou DISC (Disconnect) é enviado, aguardando uma confirmação
	- Desativo após um acknowledgement
		- O timer é desativado assim que a confirmação (ack) é recebida corretamente do receiver
	- Se expirar - timeout - forçar a retransmissão
- Retransmissão de frames I
	- Após um timeout, devido à perda do frame I ou do seu ack
		- Número máximo de tentativas de retransmissão
	- Depois de recever um acknowledgement negativo (REJ)
- Frame Protection
	- Geração e verificação de campos de proteção BCC

#### Interface Protocolo-Aplicação 

![[Pasted image 20241019151802.png]]


Exemplos de data structures

Protocol
```
struct LinkLayer 
{
	char serialPort [50]; // device /dev/ttySx, x = 0, 1
	LinkLayerRole role; // transmitter | receiver
	int baudRate;  // speed of the transmission
 	int nRetransmissions; // number of retries in case of failure
	int timeout; // timer value 1 s
}
```

### Open

```
int llopen(LinkLayer connectionParameters)
	arguments
	- connectionParameters: estrutura LinkLayer com parametros de conexão

return
- identificação de data link
- valor negativo em caso de erro

```

Step 1:
a application layer do receiver invoca a funcao llopen

![[Pasted image 20241019152515.png]]

Step 2:
a application layer do transmitter invoca llopen que corre na link layer, trocando supervision frames

![[Pasted image 20241019152802.png]]

### Send / Receive
```
int llwrite(const unsigned char *buf, int bufSize)
	arguments
	- buf: array de chars a serem transmitidos
	- bufSize: tamanho do array de chars

	retorna
	- numero de chars escritos
	- valor negativo em caso de erro


```


```
int llread(unsigned char *packet)
	arguments
	- packet: array de chars lidos

	return
	- tamanho do array - numero de caracteres lidos
	- valor negativo em caso de erro

```

- A application layer Tx forma um packet (data or control) e invoca llwrite 
- A application layer RX invoca llread; 
- As funções llwrite e llread são responsáveis pela troca de frames I e S
- Quando os frames são recebidos corretamente, ambas as funções devolvem o controlo à application layer

	![[Pasted image 20241019153804.png]]


### Close

```
int llclose(int showStatistics)
	arguments
	-  ShowStatistics: dá print a Communications statistics (exemplo: numero de frames, numero de retransmissões, numero de timeouts, etc...)

	return
	- valor positivo caso sucesso
	- valor positivo se houver erro

```

As application layers Tx e Rx invocam llclose para encerrar a conexão
Durante o encerramento, a link layer troca frames apropriados para finalizar a comunicação corretamente:
![[Pasted image 20241019154706.png]]

Transmissor:
	A aplicação chama close
	O protocolo de link envia um quadro disc para o receiver
Recetor
	O recetor recebe o quadro DISC e responde com outro quadro dis
	O transmissor então responde com UA (Unnumbered Acknowledgment) indicando que a desconexão foi confirmada
Após esta troca, a aplicação do receiver também invoca o close para concluir o processo de encerramento

### Test application 
o objetivo é desenvolver um protocolo de aplicação para transferencia de arquivos utilizando o serviço confiável oferecido pelo data link protocol

A aplicação deve suportar dois tipos de pacotes enviados pelo transmitter
	- Pacotes de Controlo: Para sinalizar o início e o fim da transferência de arquivos
	- Pacotes de Dados: Contêm fragmentos do arquivo a ser transmitido

O pacote de controlo que sinaliza o inicio da transmissão start deve ter um campo com o tamanho do arquivo e opcionalmente um campo com o nome do arquivo (entre outros campos possiveis)
O que sinaliza o fim da transmissão END deve repetir as informações contidas no START

Os data packets devem conter um campo (dois octetos) que indica o tamanho do campo de dados D1..DN para permitir verificaçẽs adicionais de integridade dos dados
Depende do tamanho maximo que pode ser estabeleicdo para o campo de Informacao dos I frames