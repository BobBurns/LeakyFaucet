;leaky faucet program
; first pass
; send value to serial
; turn on leds if change less than 255 in one second
;
;flash with avrdude -c avrisp -p m168 -P /dev/tty.usbmodem1411 -b 19200 -U flash:w:leakyFaucet.hex
;
.equ UBBRvalue = 12
.equ UCSR0C = 0xc2      ;SRAM address of UCSR0C
.def temp = r16
.def count = r17
.def flags = r18	;global flag register to tell if counter over 0xffff
.def xL = r26           ;x reg used as global counter
.def xH = r27
.def zL = r30
.def zH = r31
.device atmega168
;---- global variables ----
.cseg
.org 0
        jmp     reset
        jmp     c_change        ;int0
c_change:
	push	temp		;preserve r16
        in      temp,SREG       ;save status register
	push	temp
	clc
	in	temp,PIND
	sbrs	temp,PD2
	rjmp	c_clear		;skip if sensor switch hasn't connected
        adiw    xH:xL,1
	brcc	c_clear
	sbr	flags,(1 << 0)	;more than 0xffff
c_clear:
	pop 	temp		;restore sreg and r16
	out     SREG,temp
	pop	temp
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
;--- enable interrupt
	sbi     EIMSK,INT0              ;enable int0
        lds     temp,EICRA
        sbr     temp,ISC00
        sts     EICRA,temp              ;set external interrupt control register to interrupt change
        sbi     PORTD,PD2               ;set pullup on button pin
        ldi     temp,0xff
        out     DDRB,temp               ;set led's active
;--- send a message
        ldi     zH, high(hello_str * 2)
        ldi     zL, low(hello_str * 2)
        rcall   print_s
;--- event loop ---
loop:
	ldi     flags,0x00		;clear over flag
        ldi     xH,0x00
        ldi     xL,0x00                 ;xReg is counter
        sei                             ;set global interupt
.equ d_cnt = 50000                      ;inline delay
        ldi     count,0x10			;two seconds
outer:  ldi     r24,low(d_cnt)
        ldi     r25,high(d_cnt)
delay:  sbiw    r25:r24,1
        nop                             ;this makes four clock cycles = 200,000 us 
        brne    delay
        dec     count
        brne    outer
        cli                             ;clear global interrupt
.equ thresh = 0x20
;compare threshhold with count / should work with just high bytes
	cpi	xL,0x00			;first see if any water is flowing
	brne	htest
	cpi	xH,0x00
	breq	led_off
htest:  cpi     xH,0x01
        brge    led_off
	cpi	xL,thresh
	brge	led_off
        ldi     temp,0xff		;led on if sensor > 0 or < threshold
        out     PORTB,temp
        rjmp    msg
led_off:
	ldi     temp,0x00
        out     PORTB,temp
msg:
        mov     r24,xH
        rcall	t_htoa
        mov     r24,xL
        rcall	t_htoa
        ldi     r24,0x0d
        rcall   transmit
        ldi     r24,0x0a
        rcall   transmit
        rjmp    loop
;       
;       
;--- Subroutines ---
;print string function
;takes Z loaded with pointer to string with first byte being length
;uses r16,r17,r24
.def byte_tx = r24
print_s:
        lpm     count,Z+
for1:   lpm     byte_tx,Z+
wait:   lds     temp,UCSR0A
        sbrs	temp,UDRE0      ;wait for Tx buffer to be empty
        rjmp    wait            ;not ready
        sts     UDR0,byte_tx;
        dec     count
        brne    for1
        ret
;send ascii representation of one byte
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
transmit:
	lds 	temp,UCSR0A
	sbrs	temp,UDRE0
	rjmp	transmit
	sts	UDR0,r24
	ret
hello_str: .db 22," ** Leaky Faucet **",0x0d,0x0a
