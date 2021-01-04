;
; ppg_IV_fsm.asm
;
; Created: 11/27/2019 2:28:21 PM
; Author : Aaron
; Version: 1.0
; Target: ATMEGA324A
; Description: The purpose of the program is to 
; implement Lab 10 functionality(Lab 8 function 
; with interrupts) except this time it will be
; implemented using the fsm chart


.nolist
.include "m324adef.inc"
.list 


.def pstatel = r24 ;low byte of present state address
.def pstateh = r25 ;high byte of present state address

start:
.org 0x0000
jmp init
.org INT0addr
jmp ISR0
.org INT1addr
rjmp ISR1

.dseg
burst_count_bcd_setting:	.byte 3 ;setting in bcd
burst_count_binary_setting:	.byte 1 ;setting in binary	
normal_flag:					.byte 1 ;1 is set
continuous_flag:				.byte 1 ; 1 is set 

.cseg
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

ldi r16, $03;initialize pull-up resistors for PD0/1
out PORTD, r16

;initialize the output PA7 for the pulses
;initialize output PA6 for the buzzer
ldi r16, $C0
out DDRA, r16

;put FSM in initial state
ldi pstatel, LOW(clear_screen)
ldi pstateh, HIGH(clear_screen)


ldi r16, (1<<ISC00)|(1<<ISC01)|(1<<ISC10)|(1<<ISC11)
sts EICRA, r16

ldi r16, (1<<INT0)|(1<<INT1)
out EIMSK,	r16

sei

main_loop:
ldi r21, $30
ldi r22, $30
ldi r23, $30

ldi r16, 0 
sts continuous_flag, r16
sts normal_flag, r16
checking1:
;-------------------------------------------------
;check and see if the flag for continuous was set
lds r16, continuous_flag
cpi r16, 1 
breq zero_burst_setting

;check to see if the flag for normal burst was set
lds r16, normal_flag
cpi r16, 1 
breq normal_burst_setting
;nothing was set, so go to the beginnning
brne checking1

;if the burst is not set at 0
normal_burst_setting: 
;clear the flag to show task is done
ldi r16, 0
sts normal_flag, r16
output_normal_burst_setting:
ldi r16,10
sbi PORTA, 7
rcall var_delay
cbi PORTA,7
ldi r16,10
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
;clear the flag to show task is done
ldi r16, 0
sts continuous_flag, r16
output_zero_burst_setting:
ldi r16, 10
sbi PORTA, 7
rcall var_delay
nop
nop
cbi PORTA,7
ldi r16, 10
rcall var_delay_2
;check and see if CLR is pressed
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
;* Description:activates when the pushbutton is pressed
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
;*Title: update
;* Description: meant to update the LCD buffer when doing the burst prompt 
;*in real time
;*
;* Target:ATMEGA324A
;* Number of words: 5
;* Number of cycles: 31
;*
;* High registers modified: XH, XL
;* Parameters:r16
;*
;* Returns: N/A
;*
;*calls prompt burst count to reinitialize that line for the n = 
;***************************************************************************

update:
st X+, r23
st X+, r22
st X+, r21
rcall prompt_burst_count
ret


;***************************************************************************
;* 
;*Title: check_unneeded_buttons
;* Description: Checks if unneeded buttons were pressed on the keypad
;*
;* Target:ATMEGA324A
;* Number of words: 10 
;* Number of cycles:14
;*
;* High registers modified: r21
;*
;* Returns: N/A
;*
;***************************************************************************
check_unneeded_buttons:
cpi r21, $0B
breq equal

cpi r21, $0D
breq equal

cpi r21, $0E
breq equal

cpi r21, $0F
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
;* Number of words: 3 
;* Number of cycles: 6
;*
;* High registers modified: r17, r21
;* Parameters:r16
;*
;* Returns: r19
;*
;***************************************************************************
convert_hex:
ldi r17, $30
add r21, r17
ret

;***************************************************************************
;* 
;*Title: prompt_burst_count
;* Description: Creates n<space>=<space> on the LCD
;* also used in order to set the X pointer pointing to the start of the 
;*numbers to be displayed on the LCD
;*
;* Target:ATMEGA324A
;* Number of words: 11
;* Number of cycles: 18
;* 
;*
;* High registers modified: r16, XH, XL
;* 
;* Parameters:r16
;*
;* Returns: N/A
;*
;***************************************************************************
prompt_burst_count:
ldi XH, high(dsp_buff_1)
ldi XL, low(dsp_buff_1)

ldi r16, $6E ;displays n
st X+, r16

ldi r16, $20;displays <space>
st X+, r16

ldi r16, $3D ;displays =
st X+, r16

ldi r16, $20 ;displays <space>
st X+, r16
rcall update_lcd_dog
;at the end, should display n<space>=<space> 
;on the LCD
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

mul10a:	;***** multiplies "mp10H:mp10L" with 10 and adds "adder" high nibble
	swap	adder
mul10b:	;***** multiplies "mp10H:mp10L" with 10 and adds "adder" low nibble
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
;* When n=0, then number of cycles : 3(nm+m+n+3)
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
;* When n=0, then number of cycles : 3(nm+m+n+3)
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
;FSM 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;***************************************************************************
;* 
;* "fsm" - Simplified Table Driven Finite State Machine
;*
;* Description:
;* This table driven FSM can handle 255 or fewer input symbols.
;*
;* Author:       Aaron       
;* Version:      1.0
;* Last updated: 12/1/2019
;* Target:              ATmega324a
;* Total number of cycles depends on the task being called, which is why it
;* is excluded here
;* Total number of words: 54 words
;* Low regs modified:   r16, r18, r20, r21, r31, and r31
;* High registers used:
;*
;* Parameters:          present state in r25:r24 prior to call
;*                      input symbol in r16 prior to call
;*
;* Notes: Calls upon the stub of tasks given in "taskn"
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

;state table 
;each row consists of input symbol, next state address, task
;subroutine address

state_table:

clear_screen: 
	.dw CLEAR,		input_char,		task1
    .dw eol,	clear_screen,		task0

input_char: 
	.dw ENTERED,		check_trigger_pressed,		task3
    .dw eol,	input_char,		task2
check_trigger_pressed:
	.dw pbpress, 0 , task5
	.dw eol,	check_trigger_pressed,	task4
normalburst_check_buttons:
	.dw CLEAR,		input_char,		task1 
	.dw pbpress, normalburst_check_buttons ,task6
	.dw eol, normalburst_check_buttons, task7  
continuous_check_clr:
	.dw CLEAR,		input_char,		task1
	.dw eol,		continuous_check_clr,	task7


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
;* Description: These are the tasks called by the FSM state table
;* Each task has a specific way of being defined, depending on where in the 
;* state diagram it is being used
;*
;* Author: Aaron Varghese            
;* Version: 1.0
;* Last updated: 12/1/2019
;* Target:ATMEGA324A
;*
;*
;*
;* Notes: The registers that is used for each task is different, and the 
;*number of words and cycles for each task is different, which is why the 
;*information is excluded 
;*
;***************************************************************************


;subroutine stubs for tasks to be implemented

;clears the screen, and sound buzzer for inputs that aren't clear
;add stuff to sound buzzer 
task0:
;sounds the buzzer for all other inputs pressed besides CLR
ldi r16, 255
sbi PORTA, 6
rcall var_delay
cbi PORTA, 6
;clears the screen 
rcall init_spi_lcd;initialize lcd screen
rcall clr_dsp_buffs;displays a blank screen
rcall update_lcd_dog;updates the screen


;put FSM in initial state
ldi pstatel, LOW(clear_screen)
ldi pstateh, HIGH(clear_screen)

ret

;displays the prompt, when clear is pressed
task1:
ldi r16, 2
sts normal_flag, r16
 
ldi r16, 2
sts continuous_flag, r16

;prompt for the burst count
rcall prompt_burst_count
rcall update_lcd_dog
ret

;input numbers into the screen, and sound buzzer for 
;characters that aren't digits 
task2:
;getting the value of the burst count
;-------------------------------------------------
;meant to run in an infinite loop getting values 
;until enter is pressed
;r23-->MSB
;r21-->LSB
inner_loop1:
mov r18, r16

cpi r18, ENTERED
breq end

cpi r18,UP
breq endwithbuzzer

cpi r18,DOWN
breq endwithbuzzer

cpi r18,key2nd
breq endwithbuzzer

cpi r18,HELP
breq endwithbuzzer

cpi r18,CLEAR
breq endwithbuzzer

cpi r18, pbpress
breq endwithbuzzer

mov r23, r22
mov r22, r21

mov r21, r18

rcall convert_hex


rcall prompt_burst_count
rcall update
rcall update_lcd_dog
;when the enter key is pressed
end:
ret

;turn on buzzer for all other inputs that
;can't be used 
endwithbuzzer:
ldi r16, 255
sbi PORTA, 6
rcall var_delay
cbi PORTA, 6
ret


;saves the setting (both binary and bcd setting)
task3:
;added in case enter is pressed without inputting
;numbers (000 is default)
ldi YH, high(burst_count_bcd_setting)
ldi YL, low(burst_count_bcd_setting)
rcall prompt_burst_count
rcall update
rcall update_lcd_dog
;if enter is pressed
;storing burst_count_bcd_setting
ldi ZH, high(burst_count_bcd_setting)
ldi ZL, low(burst_count_bcd_setting)
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

rcall BCD2bin16

ldi YH, high(burst_count_binary_setting)
ldi YL, low(burst_count_binary_setting)
;has the value of binary setting stored in Y
st Y, r14
ret

;sounds the buzzer for inputs other than pb press
task4:
ldi r16, 255
sbi PORTA, 6
rcall var_delay
cbi PORTA, 6
ret

;check burst count number 
task5:
lds r14, burst_count_binary_setting
mov r22, r14
cpi r22, 0
breq continuous_flag_set

normal_flag_set:
ldi r16,1
sts normal_flag, r16
ldi pstatel, low(normalburst_check_buttons)
ldi pstateh, high(normalburst_check_buttons)
ret

continuous_flag_set:
ldi r16,1
sts continuous_flag, r16
ldi pstatel, low(continuous_check_clr)
ldi pstateh, high(continuous_check_clr)
ret

;reinitialize burst setting 
task6:
ldi YH, high(burst_count_binary_setting)
ldi YL, low(burst_count_binary_setting)
;has the value of binary setting stored in Y
ld r14, Y
mov r22, r14

;3 means that you redo the burst setting 
ldi r16, 3
sts normal_flag, r16
ret

;meant to sound the buzzer for all other 
;inputs besides pbpress and CLR
task7:
ldi r16, 255
sbi PORTA, 6
rcall var_delay
cbi PORTA, 6
ret