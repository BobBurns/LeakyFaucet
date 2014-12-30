;new routines for leaky_faucet
;
;flash with avrdude -c avrisp -p m168 -P /dev/tty.usbmodem1411 -b 19200 -U flash:w:newLeak.hex
;
;*** use int1 for senor interrup PD3 ****
; make sure to check registers before importing into ble program
.equ UBBRvalue = 12
.def temp = r16
.def count = r17
.def flags = r18	;global flag register to tell if counter over 0xffff
.def byte_tx = r24	;for serial transmit routine
.def xL = r26           ;x reg used as global counter
.def xH = r27
.def yL = r28
.def yH = r29
.def zL = r30
.def zH = r31
.device atmega168
;---- global variables ----
.equ L_FLG = 2		;global leak flag
.equ C_FLG = 3		;counter overflow flag
.cseg
.org 0
	jmp	reset
	jmp	blu_interupt
	jmp	flow_interrupt
blu_interupt:
	reti
flow_interrupt:
	sbr	flags,(1 << L_FLG)
	reti
;----- Main -----
.org    0x0034
reset:
;inittialize Stack
        ldi     temp,low(RAMEND)
        out     SPL,temp
	ldi     temp,high(RAMEND)
        out     SPH,temp
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
        sts     UCSR0C,temp
;---- enable interrupt ----
	sbi	EIMSK,INT1
	lds	temp,EICRA
	sbr	temp,(1 << ISC10)	;set INT1 to interrupt on change
	sts	EICRA,temp	
	sbi	PORTD,PD3		;set pullup on sensor pin
	sbi	DDRC,PC5		;output on LED pin
;---- init timer1 16 bit ----
	ldi	temp,0x3d		;set compare to 3d0a
	sts	OCR1AH,temp		;makes timer 1 sec loop
	ldi	temp,0x0a
	sts	OCR1AL,temp
	lds	temp,TCCR1B
	sbr	temp,(1 << WGM12)
	sts	TCCR1B,temp		;set WG mode to 4 CTC
;---- event loop ----
start:	cbr	flags,(1 << L_FLG)
	sei
poll:	sbrs	flags,L_FLG		;its fine if ble int gets called now
	rjmp	poll
	cli
;**** we have change on sensor ***
;**** so now we measure it
;todo: figure out how to handle interrupt on bluetooth RDY_LN
;**** start timer
	ldi	yL,0x00
	ldi	yH,0x00			;clear counter
	sbi	TIFR1,OCF1A		;set overflow bit to clear it
	ldi	temp,0x00
	sts	TCNT1H,temp
	sts	TCNT1L,temp		;reset timer to 0
	lds	temp,TCCR1B
	sbr	temp,(1 << CS11) | (1 << CS10)	;start timer
	sts	TCCR1B,temp	
	clc
t_lp:
	sbic	TIFR1,OCF1A
	rjmp	t_don
	sbis	PIND,PD3
	rjmp	t_lp		;don't increment if sensor hasn't connected (high)
	adiw	yH:yL,1
	brcc	no_z
	sbr	flags,(1 << C_FLG)
	rjmp	t_lp
no_z:	cbr	flags,(1 << C_FLG)
cont2:	rjmp	t_lp
;print and compare flow
.equ thresh = 0x80			;sets leak zone between 1 - 4fff
t_don:	mov	byte_tx,yH
	rcall	transmit
	mov	byte_tx,yL
	rcall	transmit
	ldi	byte_tx,0x20
	rcall	transmit
	sbrc	flags,C_FLG	;if flag is set we're over ffff
	rjmp	no_lk
	clc			;make sure carry is clear for cpi
	cpi	yH,thresh
	brsh	no_lk		;branch if yH >= thresh
	cpi	yH,0x00
	brne	ledon		;more than 0 less than thresh
	cpi	yL,0x00
	brne	ledon
;no reading so off
no_lk:	cbi	PORTC,PC5
	ldi	byte_tx,0x4e
	rcall	transmit
	rjmp	cleanup
ledon: 	sbi	PORTC,PC5
	ldi	byte_tx,0x4c
	rcall	transmit	;print L
cleanup:
	lds	temp,TCCR1B
	cbr	temp,(1 << CS11) | (1 << CS10)	;turn off counter
	rjmp	start	
;*** Subroutines ***
transmit:
	lds 	temp,UCSR0A
	sbrs	temp,UDRE0
	rjmp	transmit
	sts	UDR0,r24
	ret
