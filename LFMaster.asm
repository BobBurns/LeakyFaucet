;program to test for a leaky faucet using nordic nRF8001 and a liquid flow meter Adafruit 828
;
;compile with gavrasm ble_tests1.asm
;flash with avrdude -c avrisp -p m168 -P /dev/tty.usbmodem1411 -b 19200 -U flash:w:leaky_broadcast.hex
;
; I/O pins:
;	** use PORTB except RDYN on PORTD **
;	** SCK on pin 5 configured as output **
;	** MISO on pin 4 configured as input with pullup **
;	** MOSI on pin 3 configured as output **
;	** REQN on pin 2 configured as output **
;	** RDYN	on INT0(PD2) configured as input with pullup **
;	** RESET on pin 6 configured as output **
;---- defines ----
.device atmega168
.equ HAL_PORT = 0x05		;PORTB
.equ HAL_DDR = 0x04		;DDRB
.equ HAL_PIN = 0x03		;PINB
.equ HAL_RESET = 6
.equ HAL_SCK = 5
.equ HAL_MISO = 4
.equ HAL_MOSI = 3
.equ HAL_REQN = 2
;
.equ MSG_FLG = 0
.equ RD_FLG = 1
.equ L_FLG = 2			;global leak flag
.equ C_FLG = 3			;counter overflow flag
;
;setup messages
.equ SETUP_M = 14
;
;UBBR value for USART from f_cpu 1000000
.equ UBBRvalue = 12
;RDYN on Interrupt pin PD2 // INT0
.equ INT_PORT = 0x0b
.equ INT_PIN = 0x09
.equ HAL_RDYN = 2		;PD2
;*** some opcodes for nRF8001 ***
.equ C_TEST = 0x01
.equ C_ECHO = 0x02
.equ C_OpenAdvPipe = 0x1b
.equ C_SetLocalData = 0x0d
.equ C_Broadcast = 0x1c
.equ DS_EVT = 0x81		;device started event
.equ C_RSP_EVT = 0x84		;command response event
.equ ECHO_EVT = 0x82		;echo event
;*** delay constant
.equ dlp_init = 50000
;---- registers
.def temp = r16
.def count = r17
.def temp2 = r18
.def pulse = r19
.def flags = r20		;used to keep track of if there is a message in the in buffer
.def credit = r21		;global register for pipe credit
.def byte_tx = r22
.def delayL = r24
.def delayH = r25
.def xL = r26
.def xH = r27
.def yL = r28
.def yH	= r29
.def zL = r30
.def zH = r31
.cseg
;---- Interrupt Vector ---
.org 0
	rjmp	reset
	rjmp	RDYN_L			;INT0 vector
	rjmp	flow_interrupt		;INT1 vector
RDYN_L:
;this is called when RDYN_L is low, meaning nRF8001 has something to say
	sbrc	flags,RD_FLG		;return if flag set/ message hasn't been read
	reti
	push	temp
	push	count
	push	temp2			;preserve registers
	in	temp,SREG
	push	temp			;preserve status register
	ldi	xL,low(in_buf)
	ldi	xH,high(in_buf)		;set up buffer pointer
	cbi	HAL_PORT,HAL_REQN	;lower line to acknowledge
	ldi	temp,0x00
	rcall	SPI_tradeByte		;first byte is throw-away
	ldi	temp,0x00
	rcall	SPI_tradeByte
	tst	temp			;if length == 0 store and return
	brne	continue
	cbr	flags,(1 << MSG_FLG)			;no message
	st	X,temp
	rjmp	done
continue:
	mov	count,temp		;second byte is length
	st	X+,temp	
	sbr	flags,(1 << MSG_FLG)	;set message flag
rd_lp:	ldi	temp,0x00
	rcall	SPI_tradeByte
	st	X+,temp	
	dec	count
	brne	rd_lp
done:
	sbi	HAL_PORT,HAL_REQN	;raise line to end
	pop	temp
	out	SREG,temp
	pop	temp2
	pop	count
	pop	temp
	sbr	flags,(1 << RD_FLG)	;means message is waiting to be read
	reti
flow_interrupt:
	in	temp,SREG
	push 	temp
	clc
	inc     pulse
	brcc	i_don
	sbr 	flags,(1 << C_FLG)
i_don:	pop	temp
	out	SREG,temp
	reti
reset:
;Initialize Stack
	ldi	temp,low(RAMEND)
	out	SPL,temp
	ldi	temp,high(RAMEND)
	out 	SPH,temp
;Init SPI
;** don't have to set phase or polarity. nRF8001 uses mode 0
	in	temp,SPCR
	sbr	temp,(1 << SPR0)	;divide f_cpu by 16, better for breadboards
	sbr	temp,(1 << MSTR)	;set AVR to SPI Master mode
	sbr	temp,(1 << DORD)	;use LSB bit order
	sbr	temp,(1 << SPE)		;enable SPI
	out	SPCR,temp
;
;*** set ddr and output on pin lines
;
	sbi	HAL_DDR,HAL_REQN	;output on REQN
	sbi	HAL_PORT,HAL_REQN	;configure output as high // do this first
	sbi	HAL_DDR,HAL_SCK		;output on SCK
	sbi	HAL_PORT,HAL_MISO	;set pullup on MISO // confg as input
	sbi	HAL_DDR,HAL_MOSI	;output on MOSI
	sbi	INT_PORT,HAL_RDYN	;set pullup on RDYN line
	sbi	HAL_DDR,HAL_RESET	;output on reset
	sbi	PORTD,PD3		;set pullup on sensor pin
	sbi	DDRC,PC5		;set output on LED pin
;
;*** reset nrf8001 ***
	sbi	HAL_PORT,HAL_RESET
	ldi	count,100
	rcall	m_delay
	cbi	HAL_PORT,HAL_RESET
	sbi	HAL_PORT,HAL_RESET
;*** delay a little so nRF8001 can get hold of its lines(?)
	ldi	count,30
	rcall	m_delay
;initialize USART
        ldi     temp,high(UBBRvalue)    ;baud rate param
        sts     UBRR0H,temp
        ldi     temp,low(UBBRvalue)
        sts     UBRR0L,temp
        lds     temp,UCSR0A
        ori     temp,(1 << U2X0)        ;set use 2x because %error actual baud > .5
        sts     UCSR0A,temp
;--- USART register values
        ldi     temp,(1 << TXEN0) | (1 << RXEN0) ;enable transmit and receive
        sts     UCSR0B,temp
        ldi     temp,(1 << UCSZ01) | (1 << UCSZ00) ;8 data bits, 1 stop bit
	sts	UCSR0C,temp
;Enable Interrupts
;use default change on low EICRA ISC00,ISC01 = 0
	sbi	EIMSK,INT0		;enable INT0
	lds	temp,EICRA
	sbr	temp,(1 << ISC10)	;set INT1 to interrupt on change
	sts	EICRA,temp
	cbi	EIMSK,INT1		;disable flow interrupt during bluetooth setup
;********* Main Program **********
;
start:	sei				;enable global interrupt
wt_rstm:
	sbrs	flags,MSG_FLG
	rjmp	wt_rstm			;spin until we get incomming
	cli				;disable interrupt while reading
	cbr	flags,(1 << MSG_FLG) | (1 << RD_FLG)	;got message
	rcall	prt_m			;print whole message first
	ldi	xL,low(in_buf)
	ldi	xH,high(in_buf)		;load x reg with pointer to message
	ld	count,X+
	ld	temp,X+
	cpi	temp,DS_EVT
	breq	dse
;do something here to tell you we didn't get reset message
	ldi	zH,high(err_str << 1)
	ldi	zL,low(err_str << 1)
	rcall	print_s
	rjmp	start
dse:	ldi	zH,high(succ_str << 1)
	ldi	zL,low(succ_str << 1)
	rcall 	print_s			;success!
	adiw	xH:xL,2			;get available buffer number
	ld	credit,X
;next, send test command	
; lower reqn line to ask to send data
	cbi	HAL_PORT,HAL_REQN
wt_req:	sbic	INT_PIN,HAL_RDYN	;wait for nRF8001 to acknowledge request
	rjmp	wt_req
	ldi	temp,0x02		;length byte
	rcall	SPI_tradeByte
	ldi	temp,0x01		;test command
	rcall 	SPI_tradeByte
	ldi	temp,0x02		;DTM over ACI
	rcall	SPI_tradeByte
	sbi	HAL_PORT,HAL_REQN	;terminate comm
	ldi	count,30
	rcall	m_delay			;delay a little
	sei				;wait for response
wt2:	sbrs	flags,MSG_FLG
	rjmp	wt2
	cli
	cbr	flags,(1 << MSG_FLG) | (1 << RD_FLG)	;got message
	rcall 	prt_m
	ldi	xL,low(in_buf)
	ldi	xH,high(in_buf)		;load x reg with pointer to message
	ld	count,X+
	ld	temp,X+
	cpi	temp,DS_EVT		;should return with device started event
	breq	dse2
	cpi	temp,C_RSP_EVT		;this returns on error
	breq	cre
;if there is another event opcode start over
	ldi	zH,high(err_str << 1)
	ldi	zL,low(err_str << 1)
	rcall	print_s
	rjmp	start
cre:	ldi	zL,low(cre_str << 1)
	ldi	zH,high(cre_str << 1)
	rcall 	print_s
	adiw	xH:xL,2
	ld	byte_tx,X
	rcall	transmit		;print out status byte
in_lp:	nop
	rjmp	in_lp			;spin
dse2:	ldi	zL,low(mode_str << 1)
	ldi	zH,high(mode_str << 1)
	rcall	print_s
	ld	byte_tx,X+
	mov	temp2,byte_tx
	rcall	transmit		;print device mode
	ldi	byte_tx,0x20
	rcall	transmit	
	cpi	temp2,0x01		;check if we're in test mode
	breq	next
	rjmp	in_lp			;spin if no
next:	ldi	zL,low(buff_str << 1)
	ldi	zH,high(buff_str << 1)
	rcall	print_s
	ld	byte_tx,X+
	ld	byte_tx,X
	rcall 	transmit
	ldi	byte_tx,0x20
	rcall	transmit
;next send echo command
; lower reqn line to ask to send data
	cbi	HAL_PORT,HAL_REQN
req_lp:	sbic	INT_PIN,HAL_RDYN	;wait for nRF8001 to acknowledge request
	rjmp	req_lp
	ldi	temp,0x0d		;length byte
	rcall	SPI_tradeByte
	ldi	temp,0x02		;echo command
	rcall 	SPI_tradeByte
	ldi	count,0x0c
	ldi	zL,low(hello << 1)
	ldi	zH,high(hello << 1)
sd_lp:	lpm	temp,Z+
	rcall	SPI_tradeByte
	dec	count
	brne	sd_lp
	sbi	HAL_PORT,HAL_REQN	;terminate comm
	ldi	count,30
	rcall	m_delay			;delay a little
	sei				;wait for response
wt3:	sbrs	flags,MSG_FLG
	rjmp	wt3
	cli
	cbr	flags,(1 << MSG_FLG) | (1 << RD_FLG)	;got message
	rcall 	prt_m
	ldi	xL,low(in_buf)
	ldi	xH,high(in_buf)		;load x reg with pointer to message
	ld	count,X+
	ld	temp,X+
	cpi	temp,ECHO_EVT		;should return with echo event
	breq	echo_e
	cpi	temp,C_RSP_EVT		;this returns on error
	breq	go_cre
	ldi	zL,low(err_str << 1)
	ldi	zH,high(err_str << 1)
	rcall	print_s
	rjmp	in_lp
go_cre:	rjmp	cre			
echo_e:	ldi	zL,low(succ_str << 1)
	ldi	zH,high(succ_str << 1)
	rcall	print_s
;*** this is where we initiate setup ***
; lower reqn line to ask to send data
	cbi	HAL_PORT,HAL_REQN
wt4:	sbic	INT_PIN,HAL_RDYN	;wait for nRF8001 to acknowledge request
	rjmp	wt4
	ldi	temp,0x02		;length byte
	rcall	SPI_tradeByte
	ldi	temp,0x01		;test command
	rcall 	SPI_tradeByte
	ldi	temp,0xff		;exit test
	rcall	SPI_tradeByte
	sbi	HAL_PORT,HAL_REQN	;terminate comm
	ldi	count,30
	rcall	m_delay			;delay a little
	sei				;wait for response
wt5:	sbrs	flags,MSG_FLG
	rjmp	wt5
	cli
	cbr	flags,(1 << MSG_FLG) | (1 << RD_FLG)	;got message
	rcall 	prt_m
	ldi	xL,low(in_buf)
	ldi	xH,high(in_buf)		;load x reg with pointer to message
	ld	count,X+
	ld	temp,X+
	cpi	temp,DS_EVT		;should return with device started event
	breq	dse3
	cpi	temp,C_RSP_EVT		;this returns on error
	breq	go_cre2
	ldi	zL,low(err_str << 1)
	ldi	zH,high(err_str << 1)
	rcall	print_s
	rjmp	in_lp
go_cre2:
	rjmp	cre			
dse3:
	ldi	zL,low(mode_str << 1)
	ldi	zH,high(mode_str << 1)
	rcall	print_s
	ld	byte_tx,X+
	mov	temp2,byte_tx
	rcall	transmit		;print device mode
	ldi	byte_tx,0x20
	rcall	transmit	
	cpi	temp2,0x02		;check if we're in setup mode
	breq	next2
	rjmp	in_lp			;spin if no
;**** START SETUP ****
.def count2 = r22
next2:	ldi	zL,low(setup_str << 1)
	ldi	zH,high(setup_str << 1)
	rcall	print_s
	ldi	count,SETUP_M
	ldi	zL,low(s_data << 1)
	ldi	zH,high(s_data << 1)
s_lp:
; lower reqn line to ask to send data
	cbi	HAL_PORT,HAL_REQN
wt6:	sbic	INT_PIN,HAL_RDYN	;wait for nRF8001 to acknowledge request
	rjmp	wt6
	lpm	temp,Z+			;length byte
	mov	count2,temp		;use for inner loop
	rcall	SPI_tradeByte
sin_lp:	lpm	temp,Z+
	rcall	SPI_tradeByte		;second byte is test command
	dec	count2
	brne	sin_lp
	sbi	HAL_PORT,HAL_REQN	;terminate comm
	sei				;wait for response
wt7:	sbrs	flags,MSG_FLG
	rjmp	wt7
	cli
	cbr	flags,(1 << MSG_FLG) | (1 << RD_FLG)	;got message
	rcall 	prt_m
	ldi	xL,low(in_buf)
	ldi	xH,high(in_buf)		;load x reg with pointer to message
	ld	temp,X+			;first byte length
	ld	temp,X+
	cpi	temp,C_RSP_EVT		;should return with command response event
	breq	cre2
	rjmp	in_lp
cre2:	ld	byte_tx,X+		;opcode of response
	rcall 	transmit
	ld	byte_tx,X+
	rcall	transmit		;status code
	cpi	byte_tx,0x01		;transaction continue
	breq	cnt2
	cpi	byte_tx,0x02		;transaction complete
	breq	tr_cmp
	ldi	zL,low(err_str << 1)
	ldi	zH,high(err_str << 1)
	rcall	print_s
	rjmp	in_lp
cnt2:	dec	count
	brne	s_lp
;finished without transaction complete
	ldi	zL,low(err_str << 1)
	ldi	zH,high(err_str << 1)
	rcall	print_s
	rcall 	prt_m
	rjmp	in_lp
;next should receive device started event
tr_cmp:
	sei
wt8:	sbrs	flags,MSG_FLG
	rjmp	wt8
	cli
	cbr	flags,(1 << MSG_FLG) | (1 << RD_FLG)	;got message
	rcall	prt_m
	ldi	xL,low(in_buf)
	ldi	xH,high(in_buf)
	ld	count,X+
	ld	temp,X+
	cpi	temp,DS_EVT
	breq	dse4
;didn't get device started event
	ldi	zH,high(err_str << 1)
	ldi	zL,low(err_str << 1)
	rcall	print_s
	rjmp	in_lp
dse4:	ld	temp,X+
	cpi	temp,0x03		;standby mode
	breq	success
	ldi	zH,high(err_str << 1)
	ldi	zL,low(err_str << 1)
	rcall	print_s
	rjmp	in_lp
success:
	ldi	zH,high(succ_str << 1)
	ldi	zL,low(succ_str << 1)
	rcall 	print_s			;success!
	ld	temp,X+			;cause of restart
	ld	credit,X		;available buffers
	ldi	zL,low(buff_str << 1)
	ldi	zH,high(buff_str << 1)
	rcall	print_s
	mov	r24,credit		;register for t_htoa
	rcall	t_htoa
;next, send OpenAdvPipe command	
;eventually put leak logic here and broadcast when ready
;
leak:
	sbi	EIMSK,INT1		;enable leak interrupt
	cbi	EIMSK,INT0		;disable bluetooth interrupt
;---- init timer1 16 bit ----
	ldi	temp,0x3d		;set compare to 3d0a
	sts	OCR1AH,temp		;makes timer 1 sec loop
	ldi	temp,0x0a
	sts	OCR1AL,temp
	lds	temp,TCCR1B
	sbr	temp,(1 << WGM12)
	sts	TCCR1B,temp		;set WG mode to 4 CTC
	lds	temp,TCCR1B
	sbr	temp,(1 << CS11) | (1 << CS10)	;start timer
	sts	TCCR1B,temp	
	ldi	count,0x00		;leak counter at 0
;---- event loop ----
l_st:	cbr	flags,(1 << L_FLG)
	ldi	pulse,0x00		;clear counter
	cbr	flags,(1 << C_FLG)	;clear carry flag
	sbi	TIFR1,OCF1A		;set overflow bit to clear it
	ldi	temp,0x00
	sts	TCNT1H,temp
	sts	TCNT1L,temp		;reset timer to 0
	sei				;start counting pulses
t_lp:
	sbic	TIFR1,OCF1A
	rjmp	t_don
	rjmp	t_lp
;print and compare flow
.equ thresh = 0x1b			;sets leak zone between 1 - 11 pulses/sec
t_don:	cli
	mov	byte_tx,pulse
	rcall	transmit
	ldi	byte_tx,0x20
	rcall	transmit
	sbrc	flags,C_FLG	;if flag is set we're over 0xff
	rjmp	carry
	clc			;make sure carry is clear for cpi
	cpi	pulse,thresh
	brsh	no_lk		;branch if yH >= thresh
	tst	pulse
	brne	ledon		;more than 0 less than thresh
no_lk:	
	ldi	count,0x00	;reset leak counter
	cbi	PORTC,PC5
	ldi	byte_tx,0x4e	
	rcall	transmit
	rjmp	l_st
ledon:
	inc	count
	cpi	count,0x0a
	brlo	not_yet
 	sbi	PORTC,PC5	;here is where we tell bluetooth to broadcast
	ldi	byte_tx,0x4c
	rcall	transmit	;print L
	rjmp	b_start
not_yet:
	rjmp	l_st	
carry:  ldi	byte_tx,0xff
	rcall	transmit

;------- Broadcast routine ------
; lower reqn line to ask to send data
b_start:
	lds	temp,TCCR1B
	cbr	temp,(1 << CS11) | (1 << CS10)	;stop timer while broadcasting
	sts	TCCR1B,temp
	cbi	EIMSK,INT1		;disable leak interrupt
	sbi	EIMSK,INT0		;enable  bluetooth interrupt
	cbi	HAL_PORT,HAL_REQN
wt9:	sbic	INT_PIN,HAL_RDYN	;wait for nRF8001 to acknowledge request
	rjmp	wt9
	ldi	temp,0x09		;length byte
	rcall	SPI_tradeByte
	ldi	temp,C_OpenAdvPipe
	rcall	SPI_tradeByte
	ldi	temp,0x02		;service pipe bitmap 
	rcall	SPI_tradeByte		;0200 0000 0000 0000
	ldi	count,7
pp_lp:	ldi	temp,0x00
	rcall	SPI_tradeByte
	dec	count
	brne	pp_lp
	sbi	HAL_PORT,HAL_REQN	;terminate comm
	sei				;wait for response
wt10:	sbrs	flags,MSG_FLG
	rjmp	wt10
	cli
	cbr	flags,(1 << MSG_FLG) | (1 << RD_FLG)	;got message
	rcall 	prt_m
	ldi	xL,low(in_buf)
	ldi	xH,high(in_buf)
	ld	temp,X+			;length byte
	ld	temp,X+			;command, should be 0x81
	ld	temp,X+			;reponse command, should be 0x1b
	cpi	temp,0x1b
	breq	cont
	rjmp	nope
cont:	ld	temp,X+			;status, should be 0x00 success
	breq	cont2
	rjmp	nope
cont2:	cbi	HAL_PORT,HAL_REQN	;set local data
wt11:	sbic	INT_PIN,HAL_RDYN
	rjmp	wt11
	ldi	temp,0x03
	rcall	SPI_tradeByte		;length byte
	ldi	temp,C_SetLocalData
	rcall	SPI_tradeByte
	ldi	temp,0x01		;service pipe number
	rcall	SPI_tradeByte
	mov	temp,pulse		;local data = pulse count
	rcall	SPI_tradeByte
;edit: shortened data to one byte nrfgo setting?
	sbi	HAL_PORT,HAL_REQN	;terminate
	sei				;wait for response
wt12:	sbrs	flags,MSG_FLG
	rjmp	wt12
	cli
	cbr	flags,(1 << MSG_FLG) | (1 << RD_FLG)	;got message
	rcall	prt_m
	ldi	xL,low(in_buf)
	ldi	xH,high(in_buf)
	ld	temp,X+
	ld	temp,X+
	ld	temp,X+			;response command, should be 0x0d
	breq	cont3
	rjmp	nope
cont3:	ld	temp,X+			;status, should be 0x00 success
	cpi	temp,0x00
	breq	brdcst
nope:	ldi	zL,low(err_str << 1)
	ldi	zH,high(err_str << 1)
	rcall	print_s
in_lp2: nop
	rjmp	in_lp2
;*** finally the broadcast command ***
brdcst:
	cbi	HAL_PORT,HAL_REQN	;ask to send data
wt13:	sbic	INT_PIN,HAL_RDYN
	rjmp	wt13
	ldi	temp,0x05		;length byte first
	rcall	SPI_tradeByte
	ldi	temp,C_Broadcast
	rcall	SPI_tradeByte
	ldi	temp,0xb4		;timeout LSB
	rcall	SPI_tradeByte
	ldi	temp,0x00		;timeout = 180 sec
	rcall	SPI_tradeByte
	ldi	temp,0x00		;adv interval LSB
	rcall	SPI_tradeByte
	ldi	temp,0x02		;adv interval MSB
	rcall	SPI_tradeByte
	sbi	HAL_PORT,HAL_REQN
	sei				;terminate comm and wait
wt14:	sbrs	flags,MSG_FLG
	rjmp	wt14
	cli
	cbr	flags,(1 << MSG_FLG) | (1 << RD_FLG)
	rcall	prt_m
	ldi	xL,low(in_buf)
	ldi	xH,high(in_buf)
	ld	temp,X+			;first byte length
	ld	temp,X+			;second byte command
	ld	temp,X+			;third byte response cmd, should be C_Broadcast
	cpi	temp,C_Broadcast
	breq	cont4
	rjmp	nope
cont4:	ld	temp,X+			;status should be 0x00
	cpi	temp,0x00
	breq	woot
	rjmp	nope
woot:	ldi	zL,low(brd_str << 1)
	ldi	zH,high(brd_str << 1)
	rcall	print_s
;we should get a DisconnectedEvent (0x86) after broadcast has timed out
timeout:
	sei
wt15:	sbrs	flags,MSG_FLG
	rjmp	wt15
	cli
	cbr	flags,(1 << MSG_FLG) | (1 << RD_FLG)	
	rcall	prt_m
	rjmp	leak
;****** Subroutines ******
;print out message
prt_m:
	push	count
	ldi	xL,low(in_buf)
	ldi	xH,high(in_buf)		;load x reg with pointer to message
	ld	count,X+
	mov	byte_tx,count
	rcall	transmit
p_lp:	ld	byte_tx,X+
	rcall	transmit
	dec	count
	brne	p_lp
	pop	count
	ret
;---- function to transmit byte ----
;transmit byte from r19 over usart
transmit:
        lds     temp,UCSR0A
        sbrs    temp,UDRE0
        rjmp    transmit
        sts     UDR0,byte_tx
        ret
;print string function
;takes Z loaded with pointer to string with first byte being length
;uses r16,r17,r19
print_s:
        lpm     count,Z+
for1:   lpm     byte_tx,Z+
wait:   lds     temp,UCSR0A
        sbrs    temp,UDRE0      ;wait for Tx buffer to be empty
        rjmp    wait            ;not ready
        sts     UDR0,byte_tx;
        dec     count
        brne    for1
        ret
;
;-------- delay subroutines ------------
;takes count in reg r17 * 200 milliseconds
delay:
	ldi     delayH,high(dlp_init)   
        ldi     delayL,low(dlp_init)
dlp:    sbiw    delayH:delayL,1
	nop				;makes 4 clock cycles
        brne    dlp 
        dec     count
        brne    delay
        ret 
;millisecond delay *** takes count (r17) as msecond value
m_delay:
	ldi	temp,0xfa
md_lp:	nop
	nop
	dec	temp
	brne	md_lp			;4 clock cycles times 250 = 1000us (1000000 f_cpu)
	dec	count
	brne	m_delay
	ret
;----- SPI routines -----
;uses temp r16 as tx byte and returns with rx byte in temp
SPI_tradeByte:
	out	SPDR,temp
lp1:	in	temp2,SPSR
	sbrs	temp2,SPIF
	rjmp 	lp1
	in	temp,SPDR
	ret
;send ascii representation of one byte over serial
;byte to convert in r24
;uses r24 as low nibble to send and r25 for high nibble
t_htoa:
        lds     temp,UCSR0A
        sbrs	temp,UDRE0
        rjmp	t_htoa
	ldi	temp,0x30	;ascii offset
	mov	r25,r24
	lsr	r25
	lsr	r25
	lsr	r25
	lsr	r25
	cpi	r25,0x0A
	brlt	no_e
	ldi	temp,0x37	;extended hex
no_e:	add	r25,temp
        sts     UDR0,r25
t_2:	lds	temp,UCSR0A
	sbrs	temp,UDRE0
	rjmp	t_2
	ldi	temp,0x30	;ascii offset
	andi	r24,0x0f	;low nibble
	cpi	r24,0x0a
	brlt	no_e2
	ldi	temp,0x37
no_e2:	add	r24,temp	;add 0x30 to get ascii representation
	sts	UDR0,r24
	ret
err_str: .db 36,0x0d,0x0a,"*** not what we were expecting ***",0x0d,0x0a,0x00
succ_str: .db 20,0x0d,0x0a,"*** Success! ***",0x0d,0x0a,0x00
cre_str: .db 24,"command response event: ",0x00
mode_str: .db 5,"mode:"
buff_str: .db 8,"buffers:",0x00
hello: .db "hello world!"
brd_str: .db 22,"*** we are live! ***",0x0d,0x0a,0x00
setup_str: .db 32,"Setup mode! starting set up...",0x0d,0x0a,0x00
;***** setup data generated from nrfgo studio ******
s_data: .db 0x07,0x06,0x00,0x00,0x03,0x02,0x41,0xfe
;
.db 0x1f,0x06,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.db 0x01,0x00,0x01,0x01,0x01,0x00,0x00,0x06,0x00,0x03
.db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
;
.db 0x1f,0x06,0x10,0x1c,0x00,0x00,0x00,0x00,0x00,0x00
.db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x90,0x00,0xff
;
.db 0x1f,0x06,0x10,0x38,0xff,0xff,0x02,0x58,0x0a,0x05
.db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.db 0x10,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
;
.db 0x05,0x06,0x10,0x54,0x00,0x00
;
.db 0x1f,0x06,0x20,0x00,0x04,0x04,0x02,0x02,0x00,0x01
.db 0x28,0x00,0x01,0x00,0x18,0x04,0x04,0x05,0x05,0x00
.db 0x02,0x28,0x03,0x01,0x02,0x03,0x00,0x00,0x2a,0x04,0x04,0x14
;
.db 0x1f,0x06,0x20,0x1c,0x03,0x00,0x03,0x2a,0x00,0x01
.db 0x42,0x4f,0x42,0x64,0x69,0x63,0x73,0x65,0x6d,0x69
.db 0x2e,0x63,0x6f,0x6d,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x04
;
.db 0x1f,0x06,0x20,0x38,0x05,0x05,0x00,0x04,0x28,0x03
.db 0x01,0x02,0x05,0x00,0x01,0x2a,0x06,0x04,0x03,0x02
.db 0x00,0x05,0x2a,0x01,0x01,0x34,0x12,0x04,0x04,0x05,0x05,0x00
;
.db 0x1f,0x06,0x20,0x54,0x06,0x28,0x03,0x01,0x02,0x07
.db 0x00,0x04,0x2a,0x06,0x04,0x09,0x08,0x00,0x07,0x2a
.db 0x04,0x01,0xff,0xff,0xff,0xff,0x00,0x00,0xff,0xff,0x04,0x04
;
.db 0x1f,0x06,0x20,0x70,0x02,0x02,0x00,0x08,0x28,0x00
.db 0x01,0x01,0x18,0x04,0x04,0x02,0x02,0x00,0x09,0x28
.db 0x00,0x01,0x46,0x00,0x04,0x04,0x05,0x05,0x00,0x0a,0x28,0x03
;
.db 0x14,0x06,0x20,0x8c,0x01,0x02,0x0b,0x00,0x46,0x00
.db 0x04,0x04,0x01,0x01,0x00,0x0b,0x00,0x46,0x01,0x55,0x00,0x0d
;
.db 0x06,0x40,0x00,0x00,0x46,0x01,0x00,0x01,0x04,0x00,0x0b,0x00
;
.db 0x00,0x06,0x06,0x60,0x00,0x00,0x00,0x00
;
.db 0x06,0x06,0xf0,0x00,0x03,0x81,0xc2,0x00
;
.dseg
in_buf: .byte 0x40		;should never be more than 32 * 2 bytes in buffer
