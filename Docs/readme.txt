Elenco dei comandi ADS1256 gestiti dal firmware V1.00
==========================================================================================================
Command: WAKEUP (0xE0)
Description: Wakeup from STDBY
Parameters: NONE
Response: NONE
 
Command: SELFCAL (0xE1)
Description: Offset and Gain Self-Calibration
Parameters: NONE
Response: NONE

Command: SELFOCAL (0xE2)
Description: Offset Self-Calibration
Parameters: NONE
Response: NONE

Command: SELFGCAL (0xE3)
Description: Gain Self-Calibration
Parameters: NONE
Response: NONE

Command: SYSOCAL (0xE4)
Description: System Offset Calibration
Parameters: NONE
Response: NONE

Command: SYSGCAL (0xE5)
Description: System Gain Calibration
Parameters: NONE
Response: NONE

Command: TRIGGER (0xE6)
Description: Start a the conversions. After averaging n conversions the data output (8-byte double in IEEE 754 format) is queued in txDataQueue
Parameters: NONE
Response: B1 B1 B3 B4 B5 B6 B7 B8

Command: STANDBY (0xE7)
Description: Begin Standby Mode
Parameters: NONE
Response: NONE

Command: RESET (0xE8)
Description: Reset to Power-Up Values
Parameters: NONE
Response: NONE

Command: ACALON (0xE9)
Description: Enable Auto-Calibration. When Auto-Calibration is enabled, self-calibration begins at the completion of the commands that changes the PGA, DRATE or the Analog Input Buffer.
Parameters: NONE
Response: NONE

Command: ACALOFF (0xEA)
Description: Disable Auto-Calibration.
Parameters: NONE
Response: NONE

Command: BUFEN (0xEB)
Description: Analog Input Buffer Enable
Parameters: NONE
Response: NONE

Command: BUFDIS (0xEC)
Description: Analog Input Buffer Disable
Parameters: NONE
Response: NONE

Command: MUX B1 (0xED)
Description: Setup Input Multiplexer
Parameters: B1 - MUX configuration register (see datasheet p. 31)
Response: NONE

Command: PGA (0xEE)
Description: Setup Programmable Gain Amplifier
Parameters: B1 - Gain value code (0x00=1, 0x01=2, 0x02=4, 0x03=8, 0x04=16, 0x05=32, 0x06=64, 0x07=64)
Response: NONE

Command: DRATE B1  (0xEF)
Description: Set A/D Data Rate
Parameters: B1 - Data rate code (0x03=2.5SPS, 0x13=5SPS, 0x23=10SPS, 0x33=15SPS, 0x43=25SPS, 0x53=30SPS, 0x63=50SPS, 0x72=60SPS, 0x82=100SPS, 0x92=500SPS, 0xA1=1kSPS, 0xB0=2kSPS, 0xC0=3.75kSPS, 0xD0=7.5kSPS, 0xE0=15kSPS, 0xF0=30kSPS)
Response: NONE

Command: OFCW B1 B2 B3 (0xF0)
Description: Set Offset Calibration Bytes
Parameters: B1=byte 0, B2=byte1, B3=byte2
Response: NONE

Command: OFCR  (0xF1)
Description: Read Offset Calibration Bytes
Parameters: NONE
Response: B1 B2 B3

Command: FSCW B1 B2 B3 (0xF2)
Description: Set Full−scale Calibration Bytes
Parameters: B1=byte 0, B2=byte1, B3=byte2
Response: NONE

Command: FSCR (0xF3)
Description: Read Full-Scale Calibration Bytes
Parameters: NONE
Response: B1 B2 B3

Command: AVERAGE B1  (0xF4)
Description: Set the number of samples to average
Parameters: B1 - number of samples to average
Response: NONE

Command: READREGS (0xF5)
Description: Reads Registers
Parameters: NONE
Response: STATUS, MUX, ADCON, DRATE, IO, AVERAGE 

Command: READCAL (0xF6)
Description: Reads Registers
Parameters: NONE
Response: OFC0, OFC1, OFC2, FSC0, FSC1, FSC2
==========================================================================================================


Task attivi
==========================================================================================================
Task:        UART_RX_Task
Description: Attende il rilascio del semaforo UART_RX_Done_Sem che indica che l'UART ha ricevuto un nuovo messaggio. Rilasciato il semaforo verifica
             la validità del messaggio (STARTBYTE, CHECKSUM). 
             
             Se il messaggio è valido:     accoda ACK in txDataQueue e accoda il messaggio in CommandQueue.
             Se il messaggio non è valido: accoda NAK in txDataQueue.
             
             Alla fine riavvia la ricezione via UART in DMA.  
             
             I messaggi inviati a CommandQueue saranno elaborati dal task CommandHandler.
             I messaggi inviati a txDataQueue  saranno elaborati dal task UART_TX_TASK.

Task:        UART_TX_Task
Description: Attende e estrae i messaggi dalla queue txDataQueue e li trasmette via UART. 

             Se il messaggio è ACK (0x06) o NAK (0x15): viene trasmesso il rispettivo byte. 
             Altrimenti:                                viene trasmesso STARTBYTE STATUSCODE COMMAND [B1, B2, ...] CHECKSUM, dove:
												        STARTBYTE = 0xAA
												        STATUSCODE = SUCCESS (0x00) oppure ERRCODE
												        COMMAND = il codice del comando
												        B1, B2, ... = se presenti sono il payload del comando
												        CHECKSUM = da calcolare sulla base dei bytes precedenti   

Task:        Command_Handler
Description: 

Task:        SPI_RX_Task
Description: 
=================================================================================================================

Strutture struct
=================================================================================================================

typedef struct {
    uint8_t command_id;               // 1 byte: Il comando
    uint8_t statuscode;				  // 1 byte: 0x00 se SUCCESS, altrimenti un codice di errore
    uint8_t nbytes;					  // 1 byte: Numero di bytes del payload
    uint8_t payload[MAX_PARAM_BYTES]; // Buffer per MAX_PARAM_BYTES parametri
} CommsFrame_t;

Queues
=================================================================================================================

Coda txDataQueue
================

La coda contiene messaggi di tipo CommsFrame_t prodotti dai task che devono inviare dati all'UART.

I messaggi di questa coda vengono consumati dal consumer task UART_TX_Task.

Qualsiasi task può accodare messaggi nella coda txDataQueue. 

Nella coda txDataQueue i campi della struttura CommsFrame_t hanno questo significato:

Se il pacchetto viene usato per inviare ACK o NAK:

	command_id = ACK/NAK
	statuscode = unused
	nbytes 	   = unused
	payload    = unused
 
I campi unused saranno ignorati dal consumer.

Se il pacchetto viene usato per inviare una risposta a un comando:

	command_id = ID del comando
	statuscode = SUCCESS o codice di errore
	nbytes 	   = zero o numero di bytes del payload
	payload    = bytes del payload, se presenti

task UART_TX_Task
=================
	
Il task UART_TX_Task gestisce in esclusiva la periferica di trasmissione via UART.
 
Attende ed estrae i pacchetti CommsFrame_t dalla queue txDataQueue e li invia all'UART secondo questo schema:

                 Se il pacchetto è un ACK/NAK viene inviato il solo byte corrispondente:
                 [ACK/NAK]
                 
                 Se il pacchetto è completo si invia la seguente sequenza:
                 [STARTBYTE] [STATUSCODE] [COMMAND] [NBYTES] [B1, B2, ...] [CHECKSUM]
                 
			             - STARTBYTE   = 0xAA
			             - STATUSCODE  = 0x00 (SUCCESS) oppure ERRORCODE
			             - COMMAND     = il codice del comando eseguito
			             - NBYTES      = numero di bytes del payload (può essere zero)
			             - B1, B2, ... = eventuale payload
			             - CHECKSUM    = somma (modulo 256) dei bytes precedenti
                   
==================================================================================================================

Coda CommandQueue
=================

La coda contiene messaggi di tipo CommsFrame_t prodotti dal task StartUART_RX_Task (task di ascolto dell'UART).

I messaggi di questa coda vengono consumati dal consumer task Command_Handler.

Nella coda CommandQueue i campi della struttura CommsFrame_t hanno questo significato:

	command_id = ID del comando
	statuscode = unused
	nbytes 	   = zero o numero di bytes del payload (parametri del comando)
	payload    = bytes del payload, se presenti
	


Messaggi nella queue CommandQueue
=================================

STARTBYTE COMMAND [B1, B2, ...] CHECKSUM

Se il messaggio è stato ricevuto senza errori di checksum viene immediatamente inviato ACK (0x06) nella coda txDataQueue. 
Il messaggio viene poi accodato in CommandQueue.
Se il messaggio è stato ricevuto corrotto viene immediatamente inviato NAK (0x15) nella coda txDataQueue. 
Il messaggio viene ignorato e il PC eventualmente ritrasmette.
Il comando viene processato dal ADS1256 e viene inviato al PC:

STARTBYTE STATUSCODE COMMAND [B1, B2, ...] CHECKSUM

STATUSCODE può essere SUCCESS (0x00) oppure un ERRORCODE e viene inviato anche per i comandi che restituiscono NONE.


Esempi di comandi da inviare via UART:
==================================================================================================================================
OFCW 0x41 0xE2 0x8F -> 0xAA 0xF0 0x41 0xE2 0x8F 0x4C   Response -> 0x06 0xAA 0x00 0xF0 0x9A
MUX 0x01            -> 0xAA 0xED 0x01 0x98             Response -> 0x06 0xAA 0x00 0xED 0x97
MUX 0x08            -> 0xAA 0xED 0x08 0x9F             Response -> 0x06 0xAA 0x00 0xED 0x97
SELFCAL             -> 0xAA 0xE1 0x8B                  Response -> 0x06 0xAA 0x00 0xE1 0x8B
WAKEUP              -> 0xAA 0xE0 0x8A                  Response -> 0x06 0xAA 0x00 0xE0 0x8A
READREGS            -> 0xAA 0xF5 0x9F                  Response -> 0x06 0xAA 0x00 0xF5 STATUS, MUX, ADCON, DRATE, IO, AVERAGE, CHECKSUM
ACALON              -> 0xAA 0xE9 0x93                  Response -> 0x06 0xAA 0x00 0xE9 0x93
ACALOFF             -> 0xAA 0xEA 0x94                  Response -> 0x06 0xAA 0x00 0xEA 0x94
BUFEN               -> 0xAA 0xEB 0x95                  Response -> 0x06 0xAA 0x00 0xEB 0x95
BUFDIS              -> 0xAA 0xEC 0x96                  Response -> 0x06 0xAA 0x00 0xEC 0x96
READCAL             -> 0xAA 0xF6 0xA0                  Response -> 0x06 0xAA 0x00 0xF6 OFC0, OFC1, OFC2, FSC0, FSC1, FSC2, CHECKSUM
OFCR                -> 0xAA 0xF1 0x9B                  Response -> 0x06 0xAA 0x00 0xF1 OFC0, OFC1, OFC2, CHECKSUM 
FSCR                -> 0xAA 0xF3 0x9D                  Response -> 0x06 0xAA 0x00 0xF3 FSC0, FSC1, FSC2, CHECKSUM
PGA 0x00            -> 0xAA 0xEE 0x00 0x98             Response -> 0x06 0xAA 0x00 0xEE 0x98
PGA 0x06            -> 0xAA 0xEE 0x06 0x9E             Response -> 0x06 0xAA 0x00 0xEE 0x9E
RESET               -> 0xAA 0xE8 0x92                  Response -> 0x06 0xAA 0x00 0xE8 0x92
AVERAGE 0x05        -> 0xAA 0xF4 0x05 0xA3             Response -> 0x06 0xAA 0x00 0xF4 0x9E       
AVERAGE 0x0A        -> 0xAA 0xF4 0x0A 0xA8             Response -> 0x06 0xAA 0x00 0xF4 0x9E
DRATE 0x03          -> 0xAA 0xEF 0x03 0x9C             Response -> 0x06 0xAA 0x00 0xEF 0x99
DRATE 0x23          -> 0xAA 0xEF 0x23 0xBC             Response -> 0x06 0xAA 0x00 0xEF 0x99
TRIGGER             -> 0xAA 0xE6 0x90                  Response -> 0x06 0xAA 0x00 0xE6 0x90
DUMMY               -> 0xAA 0xF7 0xA1                  Response -> 0x06 0xAA 0x00 0xF7 0xA1     
Unknown command     -> 0xAA 0xFF 0xA9                  Response -> 0x06 0xAA 0x01 0xF7 0xA9
Corrupted checksum  -> 0xAA 0xF0 0x41 0xE2 0x8F 0x4D   Response -> 0x15
Invalid startbit    -> 0xA0 0xE0 0x8A                  Response -> 0x15
==================================================================================================================================
byte1: ACK/NAK
byte2: STARTBYTE
byte3: SUCCESS/ERRORCODE
byte4: COMMAND
byte5... CHECKSUM o PAYLOAD bytes + CHECKSUM


