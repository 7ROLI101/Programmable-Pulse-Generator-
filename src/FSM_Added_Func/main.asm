;
; ppg_IV_fsm_extra.asm
;
; Created: 12/1/2019 11:33:18 PM
; Author : Aaron
;Target:ATMEGA324A
; Created: 11/3/2019 11:16:51 PM
;Description: Using the ATMEGA324A, we will be creating a system 
;that would allow the user to implement functionality from lab9
;using interrupts and the fsm table

.nolist 
.include "m324adef.inc"
.list
	`
	
.def pstatel = r24 ;low byte of present state address
.def pstateh = r25 ;high byte of present state address

.dseg
burst_count_bcd_setting:	.byte 3 ;setting in bcd
burst_count_binary_setting:	.byte 1 ;setting in binary	

pulse_width_bcd_setting:	.byte 3 ;setting in bcd
pulse_width_binary_setting:	.byte 1 ;setting in binary	

delay_time_bcd_setting:	.byte 3 ;setting in bcd
delay_time_binary_setting:	.byte 1 ;setting in binary	

continuous_flag: .byte 1 ;flag for continuous burst
normal_flag: .byte 1; flag for the normal burst

.cseg
start:
.org 0x0000
rjmp init
.org int0addr
jmp ISR0
.org int1addr
jmp ISR1


init:
;initialize the stack pointer 
ldi r16, LOW(RAMEND)
out SPL,r16
ldi r16, HIGH(RAMEND)
out SPH, r16

;setting up PORTB
ldi r16, $FF     ; set portB = output.
out DDRB, r16     ; 
sbi PORTB, 4      ; set /SS of DOG LCD = 1 (Deselected)

;setting up PORTD
ldi r16, $00;PortD is an input port
out DDRD, r16

ldi r16, $03;initialize pull-up resistors for PD0-1
out PORTD, r16

;setting up PORTA
ldi r16, $C0
out DDRA, r16

ldi r16, $3F;initialize pull-up resistors for PD0-1
out PORTD, r16



;initializing the interrupts
;configures positive edge triggering
ldi r16, (1<<ISC00)|(1<<ISC01)|(1<<ISC10)|(1<<ISC11)
sts EICRA, r16
;configures each of the interrupts
ldi r16, (1<<INT0)|(1<<INT1)
out EIMSK, r16

rcall init_spi_lcd;initialize lcd screen

rcall clr_dsp_buffs;displays a blank screen

rcall update_lcd_dog;updates the screen

;put FSM in initial state
ldi pstatel,low(clr_screen)
ldi pstateh, high(clr_screen)
sei 

main_loop: 
;polling to see if any of the flags are set
lds r16, normal_flag
cpi r16, 1 
breq normal_burst_setting
lds r16, continuous_flag
cpi r16, 1 
breq zero_burst_setting

rjmp main_loop

;if the burst is not set at 0
;gives a 1 of 998 cycles positive and
;gives a 0 of around 99-1000 cycles 
normal_burst_setting: 
ldi r16, 0
sts normal_flag, r16
;time delay
ldi YH, high(delay_time_binary_setting)
ldi YL, low(delay_time_binary_setting)
ld r15, Y
;pulse width
ldi YH, high(pulse_width_binary_setting)
ldi YL, low(pulse_width_binary_setting)
ld r13, Y
output_normal_burst_setting:
mov r16, r13
sbi PORTA, 7
rcall var_delay
cbi PORTA,7
mov r16, r15
rcall var_delay
dec r22
brne output_normal_burst_setting

polling:
;check and see if setting is reinitialized
lds r16, normal_flag
cpi r16, 3
breq normal_burst_setting

;check and see if CLR is pressed
lds r16, normal_flag
cpi r16, 2
breq main_loop

brne polling

;if the burst happens to be set at 0
zero_burst_setting:
ldi r16, 0
sts continuous_flag, r16
;time delay
ldi YH, high(delay_time_binary_setting)
ldi YL, low(delay_time_binary_setting)
ld r15, Y
;pulse width
ldi YH, high(pulse_width_binary_setting)
ldi YL, low(pulse_width_binary_setting)
ld r13, Y
;gives a 1 of 1000 cycles and a 0 of 1007 cycles 
output_zero_burst_setting:
mov r16, r13
sbi PORTA, 7
rcall var_delay
nop
nop
cbi PORTA,7
mov r16, r15
rcall var_delay
lds r16, continuous_flag
cpi r16, 2
breq main_loop

rjmp output_zero_burst_setting

.nolist
.include "lcd_dog_asm_driver_m324a.inc"
.include "subroutines.inc"
.list

;***************************************************************************
;* 
;*Title: ISR0
;* Description: checks what button on the keypad is presseed
;*
;* Target:ATMEGA324A
;* Number of words: 17 words 
;* Number of cycles: 25 cycles 
;*
;* High registers modified:
;*
;* Returns: N/A
;*
;*also calls upon the code_to_value subroutine to decode the button press
;* returns the keycode in r18
;***************************************************************************
ISR0:
push r16
in r16, SREG
push r16
in r16, PIND
andi r16, $F0
swap r16
mov r18, r16
rcall code_to_value
mov r16, r18
rcall fsm
pop r16
out SREG, r16
pop r16
reti

;***************************************************************************
;* 
;*Title: ISR1
;* Description: Checks to see if the pushbutton is activated
;*
;* Target:ATMEGA324A
;* Number of words: 22
;* Number of cycles: 24769 
;*
;* Low registers modified: N/A
;* High registers modified: N/A

;* Returns: carry flag set or cleared
;*
;***************************************************************************
ISR1:
push r16
in r16, SREG
push r16

makedebounce:
sbis PIND, 3
rjmp makedebounce
ldi r16, 100
rcall var_delay
breakdebounce:
sbic PIND, 3
rjmp breakdebounce 
ldi r16, 100
rcall var_delay

ldi r16, (1<<INT1)
out EIFR, r16
;using 16 as the pbpress
ldi r16, $10 
rcall fsm

pop r16
out SREG, r16
pop r16
reti
;***************************************************************************
;* 
;*Title: decode_button
;* Description: checks what button on the keypad is presseed
;*
;* Target:ATMEGA324A
;* Number of words: 12 words
;* Number of cycles: 31 cycles
;*
;* High registers modified: r16, r18, ZH, ZL
;*
;*also calls upon the code_to_value subroutine to decode the button press
;*returns the decoded keycode from the table into r18
;***************************************************************************
decode_button:
;check to see if a button is pressed
clc ;indicates that button is not pressed
sbis PINC, 6
ret
in r16, PIND
andi r16, $F0
swap r16
mov r18, r16
rcall code_to_value
sec ;indicates that a key is pressed
;clears the D-FF and sets it again for polling
cbi PORTC, 7
sbi PORTC, 7 
ret

;***************************************************************************
;* 
;*Title: select_line
;* Description: meant to show which line is currently active and 
;*writing inputs to the LCD 
;*
;* Target:ATMEGA324A
;* Number of words: 3 words
;* Number of cycles: 7 cycles
;*
;* High registers modified: r18, YH, YL
;*
;* Returns: N/A
;*
;***************************************************************************
select_line:
ldi r18, '#'
std Y+11, r18
ret

;***************************************************************************
;* 
;*Title: deselect_line
;* Description: meant to show which line is currently inactive 
;* Target:ATMEGA324A
;* Number of words: 3 words
;* Number of cycles: 7 cycles
;*
;* High registers modified: r18, YH, YL
;*
;* Returns: N/A
;*
;***************************************************************************
deselect_line:
ldi r18, ' '
std Y+11, r18
ret


;***************************************************************************
;* 
;*Title: code_to_value
;* Description:function that decodes the button
;*
;* Target:ATMEGA324A
;* Number of words: 15 words
;* Number of cycles: 12 cycles
;*
;* High registers modified: r16, r18, ZH, ZL
;* Parameters:r16
;*
;* Returns: N/A
;*
;***************************************************************************
code_to_value://table lookup
conversion:
ldi ZH, high(keyconvert*2)
ldi ZL, low(keyconvert*2)
ldi r16, $00
add ZL, r18
adc ZH, r16
lpm r18, Z
ret
;table of values used for all of the pushbuttons
keyconvert: .db $01, $02, $03,$0F, $04, $05, $06, $0E
			.db $07, $08, $09, $0D, $0A, $00, $0B, $0C




;***************************************************************************
;* 
;*Title: keep_values
;* Description: meant to take in the values in a line after switching in 
;* between lines
;*used to prevent lines from being written with other line's values in them
;* Target:ATMEGA324A
;* Number of words: 4 words
;* Number of cycles: 10 cycles
;*
;* High registers modified: r21,r22,r23
;*
;* Returns: Previous values of the a current line 
;*
;* meant to be used after initializing the prompt for a specific line
;* using either prompt_burst_count or prompt_pulse_width_count
;***************************************************************************
keep_values:
ld r23, Y+
ld r22, Y+
ld r21, Y+
ret



;***************************************************************************
;* 
;*Title: check_unneeded_buttons
;* Description: Checks if unneeded buttons were pressed on the keypad
;* Unneeded buttons are now the 2nd and HELP buttons
;*
;* Target:ATMEGA324A
;* Number of words: 10 words
;* Number of cycles:14 cycles
;*
;* High registers modified: r18
;*
;* Returns: N/A
;*
;***************************************************************************
check_unneeded_buttons:
cpi r18, $0B
breq equal

cpi r18, $0D
breq equal

rjmp notequal

equal:

notequal:
ret

;***************************************************************************
;* 
;*Title: convert_hex
;* Description: simple program used to convert hex
;*
;* Target:ATMEGA324A
;* Number of words: 3 words
;* Number of cycles: 6 cycles
;*
;* High registers modified: r17, r21
;* Parameters:r16
;*
;* Result is in: r21
;*
;***************************************************************************
convert_hex:
ldi r17, $30
add r21, r17
ret




;***************************************************************************
;* 
;*Title: check_internal_trigger
;* Description:checking if Q at PA3 is set
;*
;* Target:ATMEGA324A
;* Number of words: 9 words
;* Number of cycles: 24769 cycles
;*
;* High registers modified: r16
;* Returns: carry flag set or cleared
;*
;***************************************************************************
check_internal_trigger:
clc ;carry clear means that the pushbutton is not pressed
sbis PINA,3
ret;pushbutton is not pressed

sec; pushbutton has been pressed
cbi PORTA, 2
ldi r16, 250
rcall var_delay
sbi PORTA, 2 
ret


;***************************************************************************
;*
;* "BCD2bin16" - BCD to 16-Bit Binary Conversion
;*
;* This subroutine converts a 5-digit packed BCD number represented by
;* 3 bytes (fBCD2:fBCD1:fBCD0) to a 16-bit number (tbinH:tbinL).
;* MSD of the 5-digit number must be placed in the lowermost nibble of fBCD2.
;*
;* Let "abcde" denote the 5-digit number. The conversion is done by
;* computing the formula: 10(10(10(10a+b)+c)+d)+e.
;* The subroutine "mul10a"/"mul10b" does the multiply-and-add operation
;* which is repeated four times during the computation.
;*
;* Number of words	:30
;* Number of cycles	:108
;* Low registers used	:4 (copyL,copyH,mp10L/tbinL,mp10H/tbinH)
;* High registers used  :4 (fBCD0,fBCD1,fBCD2,adder)	
;*
;***************************************************************************

;***** "mul10a"/"mul10b" Subroutine Register Variables

.def	copyL	=r12		;temporary register
.def	copyH	=r13		;temporary register
.def	mp10L	=r14		;Low byte of number to be multiplied by 10
.def	mp10H	=r15		;High byte of number to be multiplied by 10
.def	adder	=r19		;value to add after multiplication	

;***** Code

mul10a:	;***** multiplies "mp10H:mp10L" with 10 and adds "adder" high 
;nibble
	swap	adder
mul10b:	;***** multiplies "mp10H:mp10L" with 10 and adds "adder" low 
;nibble
	mov	copyL,mp10L	;make copy
	mov	copyH,mp10H
	lsl	mp10L		;multiply original by 2
	rol	mp10H
	lsl	copyL		;multiply copy by 2
	rol	copyH		
	lsl	copyL		;multiply copy by 2 (4)
	rol	copyH		
	lsl	copyL		;multiply copy by 2 (8)
	rol	copyH		
	add	mp10L,copyL	;add copy to original
	adc	mp10H,copyH	
	andi	adder,0x0f	;mask away upper nibble of adder
	add	mp10L,adder	;add lower nibble of adder
	brcc	m10_1		;if carry not cleared
	inc	mp10H		;	inc high byte
m10_1:	ret	

;***** Main Routine Register Variables

.def	tbinL	=r14		;Low byte of binary result (same as mp10L)
.def	tbinH	=r15		;High byte of binary result (same as mp10H)
.def	fBCD0	=r16		;BCD value digits 1 and 0
.def	fBCD1	=r17		;BCD value digits 2 and 3
.def	fBCD2	=r18		;BCD value digit 5

;***** Code

BCD2bin16:
	andi	fBCD2,0x0f	;mask away upper nibble of fBCD2
	clr	mp10H		
	mov	mp10L,fBCD2	;mp10H:mp10L = a
	mov	adder,fBCD1
	rcall	mul10a		;mp10H:mp10L = 10a+b
	mov	adder,fBCD1
	rcall	mul10b		;mp10H:mp10L = 10(10a+b)+c
	mov	adder,fBCD0		
	rcall	mul10a		;mp10H:mp10L = 10(10(10a+b)+c)+d
	mov	adder,fBCD0
	rcall	mul10b		;mp10H:mp10L = 10(10(10(10a+b)+c)+d)+e
	ret

;************************
;NAME:      clr_dsp_buffs
;FUNCTION:  Initializes dsp_buffers 1, 2, and 3 with blanks (0x20)
;ASSUMES:   Three CONTIGUOUS 16-byte dram based buffers named
;           dsp_buff_1, dsp_buff_2, dsp_buff_3.
;RETURNS:   nothing.
;MODIFIES:  r25,r26, Z-ptr
;CALLS:     none
;CALLED BY: main application and diagnostics
;********************************************************************
clr_dsp_buffs:
     ldi R25, 48               ; load total length of both buffer.
     ldi R26, ' '              ; load blank/space into R26.
     ldi ZH, high (dsp_buff_1) ; Load ZH and ZL as a pointer to 1st
     ldi ZL, low (dsp_buff_1)  ; byte of buffer for line 1.
   
    ;set DDRAM address to 1st position of first line.
store_bytes:
     st  Z+, R26       ; store ' ' into 1st/next buffer byte and
                       ; auto inc ptr to next location.
     dec  R25          ; 
     brne store_bytes  ; cont until r25=0, all bytes written.
     ret
;**********************************************************************


;************************
;NAME:      prepare_for_BCD2bin
;FUNCTION:  prepares for the use of the BCD2bin function
;RETURNS:   
;MODIFIES: Zh, ZL, r16, r17, r18
;cycles: 18 cycles
;words: 11 words
;(MSB)(MIDDLE DIGIT)(LSB)
; r16-------> (MIDDLE DIGIT)(LSB)
; r17-------> (0)(MSB)
; r18-------> (0)(0)
;********************************************************************
prepare_for_BCD2bin:
;convert back into unpacked hex
subi r23, $30
subi r22, $30
subi r21, $30
;storing it into the setting 
st Z+, r23
st Z+, r22
st Z+, r21

;prepping for the BCD2BIN function -->43210
swap r22
;will give BCD digits 1 and 0
mov r16, r21
add r16, r22
;will give BCD digits 3 and 2
mov r17, r23
;set BCD digit 4 and 5 as 0
ldi r18, $00
ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;DELAY FUNCTIONS
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;***************************************************************************
;* 
;*Title: var_delay
;* Description: Creates a delay for the ATMEGA324(has to run @ 1Mhz CLK)
;*
;* Target:ATMEGA324A
;* Number of words:6
;* Number of cycles: 25248 cycles (when n = 32 and m = 255)
;* n = inner loop variable and m = outer loop variable
;* 3(nm+m+1)= total number of cycles 
;*
;* High registers modified: r16, r17
;* Parameters:r16
;*
;* Returns: N/A
;*
;*Delay provided should be around n*0.1ms
;*As stated before, n is the number inputted into r16
;***************************************************************************
var_delay: ;delay for ATmega324 @ 1MHz = r16 * 0.1 ms
outer_loop:
ldi r17, 32
inner_loop:
dec r17
brne inner_loop
dec r16
brne outer_loop
ret


;* 
;*Title: var_delay_2
;* Description: Creates a delay for the ATMEGA324(has to run @ 1Mhz CLK)
;* Same purpose as var_delay, except that r17 is defined as 31
;* Target:ATMEGA324A
;* Number of words:6
;* Number of cycles:
;* n = inner loop variable and m = outer loop variable
;* 3(nm+m+1)= total number of cycles 
;*
;* High registers modified: r16, r17
;* Parameters:r16
;*
;* Returns: N/A
;*
;*Delay provided should be around n*0.1ms
;*As stated before, n is the number inputted into r16
;***************************************************************************
var_delay_2: 
outer:
ldi r17, 31
inner:
dec r17
brne inner
dec r16
brne outer
ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;PROMPTS AND UPDATES FOR THE PROMPTS
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;***************************************************************************
;* 
;*Title: prompt_burst_count
;* Description: Creates n<space>=<space> on the LCD
;* also used in order to set the X pointer pointing to the start of the 
;*numbers to be displayed on the LCD
;*
;* Target:ATMEGA324A
;* Number of words: 11 words
;* Number of cycles: 6313 cycles
;* 
;*
;* High registers modified: r16, YH, YL
;* 
;* Parameters:r16
;*
;* Returns: N/A
;*
;***************************************************************************
prompt_burst_count:
ldi YH, high(dsp_buff_1)
ldi YL, low(dsp_buff_1)

ldi r16, $6E ;displays n
st Y+, r16

ldi r16, $20;displays <space>
st Y+, r16

ldi r16, $3D ;displays =
st Y+, r16

ldi r16, $20 ;displays <space>
st Y+, r16
rcall update_lcd_dog
;at the end, should display n<space>=<space> 
;on the LCD
ret

;***************************************************************************
;* 
;*Title: update_burst_count
;* Description: meant to update the LCD buffer when doing the burst prompt 
;*in real time (for the burst count line (line 1))
;*
;* Target:ATMEGA324A
;* Number of words: 5 words
;* Number of cycles: 6326 cycles 
;*
;* High registers modified: YH, YL
;* Parameters:r16
;*
;* Returns: N/A
;*
;*calls prompt burst count to reinitialize that line for the n = 
;***************************************************************************

update_burst_count:
st Y+, r23
st Y+, r22
st Y+, r21
rcall prompt_burst_count
ret


;***************************************************************************
;* 
;*Title: prompt_pulse_width_count
;* Description: Creates t<space>=<space> on the LCD
;* also used in order to set the X pointer pointing to the start of the 
;*numbers to be displayed on the LCD
;*
;* Target:ATMEGA324A
;* Number of words: 11 words
;* Number of cycles: 6313 cycles 
;* 
;*
;* High registers modified: r16, YH, YL
;* 
;* Parameters:r16
;*
;* Returns: N/A
;*
;***************************************************************************
prompt_pulse_width_count:
ldi YH, high(dsp_buff_2)
ldi YL, low(dsp_buff_2)

ldi r16, 't' ;displays t
st Y+, r16

ldi r16, $20;displays <space>
st Y+, r16

ldi r16, $3D ;displays =
st Y+, r16

ldi r16, $20 ;displays <space>
st Y+, r16
rcall update_lcd_dog
;at the end, should display t<space>=<space> 
;on the LCD
ret


;***************************************************************************
;* 
;*Title: update_pulse_width
;* Description: meant to update the LCD buffer when doing the pulse width 
;*in real time (for the pulse width line (line 2))
;*
;* Target:ATMEGA324A
;* Number of words: 5 words
;* Number of cycles: 6326 cycles
;*
;* High registers modified: YH, YL
;*
;* Returns: N/A
;*
;*calls prompt pulse width count to reinitialize that line for the t = 
;***************************************************************************

update_pulse_width:
st Y+, r23
st Y+, r22
st Y+, r21
rcall prompt_pulse_width_count
ret

;***************************************************************************
;* 
;*Title: prompt_delay_count
;* Description: Creates d<space>=<space> on the LCD
;* also used in order to set the X pointer pointing to the start of the 
;*numbers to be displayed on the LCD
;*
;* Target:ATMEGA324A
;* Number of words: 11 words
;* Number of cycles: 6313 cycles
;* 
;*
;* High registers modified: r16, YH, YL
;* 
;* Parameters:r16
;*
;* Returns: N/A
;*
;***************************************************************************
prompt_delay_count:
ldi YH, high(dsp_buff_3)
ldi YL, low(dsp_buff_3)

ldi r16, 'd' ;displays n
st Y+, r16

ldi r16, $20;displays <space>
st Y+, r16

ldi r16, $3D ;displays =
st Y+, r16

ldi r16, $20 ;displays <space>
st Y+, r16
rcall update_lcd_dog
;at the end, should display n<space>=<space> 
;on the LCD
ret

;***************************************************************************
;* 
;*Title: update_delay
;* Description: meant to update the LCD buffer when doing the pulse width 
;*in real time
;*
;* Target:ATMEGA324A
;* Number of words: 5 words
;* Number of cycles: 6326 cycles
;*
;* High registers modified: r23,r22, r21, YH, YL
;*
;* Returns: N/A
;*
;*calls prompt burst count to reinitialize that line for the t = 
;***************************************************************************

update_delay:
st Y+, r23
st Y+, r22
st Y+, r21
rcall prompt_delay_count
ret

;***************************************************************************
;* 
;* "fsm" - Simplified Table Driven Finite State Machine
;*
;* Description:
;* This table driven FSM can handle 255 or fewer input symbols.
;*
;* Author:              Ken Short
;* Version:             2.0
;* Last updated:        11/09/15
;* Target:              ATmega16
;* Number of words:
;* Number of cycles:
;* Low regs modified:   r16, r18, r20, r21, r31, and r31
;* High registers used:
;*
;* Parameters:          present state in r25:r24 prior to call
;*                      input symbol in r16 prior to call
;*
;* Notes: 
;*
;***************************************************************************

;input symbols for example finite state machine
.equ i0 = $00   ;input symbols equated to numerical values ;
.equ i1 = $01
.equ i2 = $02
.equ i3 = $03
.equ i4 = $04
.equ i5 = $05
.equ i6 = $06
.equ i7 = $07
.equ i8 = $08
.equ i9 = $09
.equ UP = $0F
.equ DOWN = $0E
.equ key2nd = $0D 
.equ CLEAR = $0A
.equ HELP = $0B
.equ ENTERED = $0C 
.equ pbpress = $10   
.equ eol = $FF  ;end of list (subtable) do not change

;state table for example finite state machine
;each row consists of input symbol, next state address, task
;subroutine address

state_table:

clr_screen: 
    .dw CLEAR,		input_burst,		task1
    .dw eol,	clr_screen,		task0

input_burst: 
	.dw ENTERED,		check_trigger,		task8
    .dw DOWN,		input_width,		task3
    .dw eol,	input_burst,		task2

input_width: 
	.dw ENTERED,		check_trigger,		task8
    .dw UP,			input_burst,		task6
	.dw DOWN,		input_delay,		task5
    .dw eol,	input_width,		task4
input_delay: 
	.dw ENTERED,		check_trigger,		task8
    .dw UP,		input_width,		task3
    .dw eol,	input_delay,		task7

check_trigger: 
	.dw pbpress,		0,		task11
    .dw eol,	check_trigger,		task10

normal_burst_int_trigger: 
	.dw pbpress,		normal_burst_int_trigger,		task9
    .dw CLEAR,		input_burst,		task1
    .dw eol,	normal_burst_int_trigger,		task10

continuous_chk_clr: 
	.dw CLEAR,		input_burst,		task1
    .dw eol,	continuous_chk_clr,		task10


fsm:
;load Z with a byte pointer to the subtable corresponding to the
;present state
    mov ZL, pstatel ;load Z pointer with pstate address * 2
    add ZL, ZL ;since Z will be used as a byte pointer with the lpm instr.
    mov ZH, pstateh
    adc ZH, ZH

;search subtable rows for input symbol match
search:
    lpm r18, Z ;get symbol from state table
    cp r18, r16 ;compare table entry with input symbol
    breq match

;check input symbol against eol
check_eol:
    cpi r18, eol ;compare low byte of table entry with eol
    breq match

nomatch:
    adiw ZL, $06 ;adjust Z to point to next row of state table
    rjmp search ;continue searching

;a match on input value to row input value has been found
;the next word in this row is the next state address
;the word following that is the task subroutine's address
match:
	;make preseent state equal to next state value in row
	;this accomplishes the stat transition
    adiw ZL, $02 ;point to low byte of state address
    lpm pstatel, Z+; ;copy next state addr. from table to preseent stat
    lpm pstateh, Z+

	;execute the subroutine that accomplihes the task associated
	;with the transition
    lpm r19, Z+ ;get subroutine address from state table
    lpm r20, Z ;and put it in Z pointer
    mov ZL, r19
    mov ZH, r20
    icall ;Z pointer is now used as a word pointer
    ret



;***************************************************************************
;* 
;* "taskn" - Stub subroutines for testing
;*
;* Description:
;* These subroutines are the tasks for the simple table driven FSM example.
;* When a program is being developed, you should start with each of these
;* subroutines consisting of just a nop and a return. You can then simulate
;* the program and verify that the transitions defined by you transition
;* table and original state diagram take place in response to input
;* sequences.
;*
;* Author:              Ken Short
;* Version:
;* Last updated:
;* Target:
;* Number of words:
;* Number of cycles:
;* Low registers used:
;* High registers used:
;*
;* Parameters:
;*
;* Notes: 
;*
;***************************************************************************

;sounds the buzzer for all inputs not CLR, and clears the screen
task0:
rcall clr_dsp_buffs;displays a blank screen
;rcall update_lcd_dog;updates the screen

;put FSM in initial state
ldi pstatel,low(clr_screen)
ldi pstateh, high(clr_screen)

;sounds the buzzer 
ldi r16, 255
sbi PORTA, 6
rcall var_delay
cbi PORTA, 6
ret

;displays all prompts, makes burst line active
task1:
;clears all flags
ldi r16, 2
sts normal_flag, r16
 
ldi r16, 2
sts continuous_flag, r16
;initializes all lines to 0 

;initial values for the burst count 
ldi r21, $30
ldi r22, $30
ldi r23, $30

;setting r21,r20 and r19 to zero in case of enter 
;pressed without inputting values
rcall prompt_burst_count
rcall update_burst_count
rcall select_line

rcall prompt_pulse_width_count
rcall update_pulse_width

rcall prompt_delay_count
rcall update_delay

ret

;inputs digits into burst buffer, sounds buzzer for keys not used
task2:
;-------------------------------------------------
;GETTING BURST COUNT 
;-------------------------------------------------
;getting the value of the burst count
;meant to run in an infinite loop getting values 
;until enter is pressed
;r23-->MSB
;r21-->LSB

input1: ;corresponding to line 1 of the LCD

mov r18, r16
;if CLR is pressed, take another input instead
cpi r18, $0A
breq endwithbuzzer

;if pushbutton is pressed, take another input
cpi r18, pbpress
breq endwithbuzzer

;if up arrow is pressed, ignore input
cpi r18, $0F
breq endwithbuzzer

rcall check_unneeded_buttons
breq endwithbuzzer

mov r23, r22
mov r22, r21

mov r21, r18

rcall convert_hex
rcall prompt_burst_count
rcall update_burst_count
rcall select_line
rcall prompt_pulse_width_count
rcall deselect_line
rcall prompt_delay_count
rcall deselect_line
rcall update_lcd_dog
ret

;sounds the buzzer for useless inputs
endwithbuzzer:
ldi r16, 255
sbi PORTA, 6
rcall var_delay
cbi PORTA, 6
ret


;width line active (makes other lines inactive)
task3:
rcall prompt_pulse_width_count
rcall keep_values
rcall prompt_pulse_width_count
rcall select_line
rcall prompt_burst_count
rcall deselect_line
rcall prompt_delay_count
rcall deselect_line
rcall update_lcd_dog
ret

;inputs digits into width buffer, sounds buzzer for keys not used
task4:
;-------------------------------------------------
;GETTING PULSE WIDTH
;-------------------------------------------------

input2: 
mov r18, r16
;if CLR is pressed, take another input instead
cpi r18, $0A
breq endwithbuzzer2

;if pushbutton is pressed, take another input
cpi r18, pbpress
breq endwithbuzzer2

rcall check_unneeded_buttons
breq endwithbuzzer2

mov r23, r22
mov r22, r21

mov r21, r18


mov r21, r18
rcall convert_hex
rcall prompt_burst_count
rcall deselect_line
rcall prompt_delay_count 
rcall deselect_line
rcall prompt_pulse_width_count
rcall update_pulse_width
rcall select_line
rcall update_lcd_dog

ret

;sounds the buzzer for useless inputs
endwithbuzzer2:
ldi r16, 255
sbi PORTA, 6
rcall var_delay
cbi PORTA, 6
ret

;delay line active
task5:
inner_loop3:
rcall prompt_delay_count
rcall keep_values
rcall prompt_delay_count
rcall select_line
rcall prompt_burst_count
rcall deselect_line
rcall prompt_pulse_width_count
rcall deselect_line
rcall update_lcd_dog
ret

;burst line active
task6:
rcall prompt_burst_count
rcall keep_values
rcall prompt_burst_count
rcall select_line
rcall prompt_pulse_width_count
rcall deselect_line
rcall prompt_delay_count
rcall deselect_line
rcall update_lcd_dog
ret

;inputs digits into delay buffer, sounds buzzer for keys not used
task7:
;-------------------------------------------------
;GETTING DELAY COUNT
;-------------------------------------------------

input3:;corresponding to line 3 of the LCD
mov r18, r16
;if CLR is pressed, take another input instead
cpi r18, $0A
breq endwithbuzzer3

;if pushbutton is pressed, take another input
cpi r18, pbpress
breq endwithbuzzer3

;if down arrow is pressed, read another input again
cpi r18, $0E
breq endwithbuzzer3

rcall check_unneeded_buttons
breq endwithbuzzer3

mov r23, r22
mov r22, r21

mov r21, r18


mov r21, r18
rcall convert_hex
rcall prompt_burst_count
rcall deselect_line
rcall prompt_pulse_width_count
rcall deselect_line
rcall prompt_delay_count
rcall update_delay
rcall select_line
rcall update_lcd_dog
ret
;sounds the buzzer for useless inputs
endwithbuzzer3:
ldi r16, 255
sbi PORTA, 6
rcall var_delay
cbi PORTA, 6
ret

;saves all settings, if enter is pressed
task8:
;this is for inputting the prompt burst count into memory
rcall prompt_burst_count
rcall deselect_line


rcall prompt_burst_count
ld r23, Y+
ld r22, Y+
ld r21, Y+

;storing burst_count_bcd_setting
ldi ZH, high(burst_count_bcd_setting)
ldi ZL, low(burst_count_bcd_setting)

;prepares for the BCD2bin
;stores teh bcd setting into SRAM and 
;gets ready to input to the BCD2bin function
rcall prepare_for_BCD2bin

rcall BCD2bin16

ldi YH, high(burst_count_binary_setting)
ldi YL, low(burst_count_binary_setting)
;has the value of binary setting stored in Y
st Y, r14

;this is for storing the pulse width
;this is for inputting the prompt burst count into memory

rcall prompt_pulse_width_count
rcall deselect_line

rcall prompt_pulse_width_count
ld r23, Y+
ld r22, Y+
ld r21, Y+

;storing burst_count_bcd_setting
ldi ZH, high(pulse_width_bcd_setting)
ldi ZL, low(pulse_width_bcd_setting)


rcall prepare_for_BCD2bin

rcall BCD2bin16

ldi YH, high(pulse_width_binary_setting)
ldi YL, low(pulse_width_binary_setting)
;has the value of binary setting stored in Y
st Y, r14

;this is for inputting the prompt delay count into memory
rcall prompt_delay_count
rcall deselect_line

rcall prompt_delay_count
ld r23, Y+
ld r22, Y+
ld r21, Y+

;storing delay_time_bcd_setting
ldi ZH, high(delay_time_bcd_setting)
ldi ZL, low(delay_time_bcd_setting)

;prepares for the BCD2bin
;stores teh bcd setting into SRAM and 
;gets ready to input to the BCD2bin function
rcall prepare_for_BCD2bin

rcall BCD2bin16

ldi YH, high(delay_time_binary_setting)
ldi YL, low(delay_time_binary_setting)
;has the value of binary setting stored in Y
st Y, r14


;taking the burst count setting 
ldi YH, high(burst_count_binary_setting)
ldi YL, low(burst_count_binary_setting)
ld r14, Y
mov r22, r14

;taking the pulse width count setting
ldi YH, high(pulse_width_binary_setting)
ldi YL, low(pulse_width_binary_setting)
ld r13, Y

;taking the delay count setting 
ldi YH, high(delay_time_binary_setting)
ldi YL, low(delay_time_binary_setting)
ret

;reinit burst setting 
task9:
ldi YH, high(burst_count_binary_setting)
ldi YL, low(burst_count_binary_setting)
ld r22, Y

;3 means that you redo the burst setting 
ldi r16, 3
sts normal_flag, r16
ret


;sounds buzzer for all inputs not CLR or pbpress
task10:
ldi r16, 255
sbi PORTA, 6
rcall var_delay
cbi PORTA, 6
ret

;check to see if the burst count is 0 or a number
task11:
lds r14, burst_count_binary_setting
mov r22, r14
cpi r22, 0
breq continuous_flag_set

normal_flag_set:
ldi r16,1
sts normal_flag, r16
ldi r16, 0
sts continuous_flag, r16
ldi pstatel, low(normal_burst_int_trigger)
ldi pstateh, high(normal_burst_int_trigger)
ret

continuous_flag_set:
ldi r16,1
sts continuous_flag, r16
ldi r16,0
sts normal_flag, r16
ldi pstatel, low(continuous_chk_clr)
ldi pstateh, high(continuous_chk_clr)
ret



