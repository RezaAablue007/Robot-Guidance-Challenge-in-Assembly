; COE538 - Lab Project
; Rendel Abrasia - 500942743
; Reza Aablue - 500966944
; Huzaifa Ali - 500901727
; Section 14
; TA: Samaneh Yazdanipour

;*****************************************************************
;* This stationery serves as the framework for a                 *
;* user application (single file, absolute assembly application) *
;* For a more comprehensive program that                         *
;* demonstrates the more advanced functionality of this          *
;* processor, please see the demonstration applications          *
;* located in the examples subdirectory of the                   *
;* Freescale CodeWarrior for the HC12 Program directory          *
;*****************************************************************

; export symbols
            XDEF Entry, _Startup            ; export 'Entry' symbol
            ABSENTRY Entry        ; for absolute assembly: mark this as application entry point



; Include derivative-specific definitions 
		INCLUDE 'derivative.inc' 

ROMStart    EQU  $4000  ; absolute address to place my code/constant data

; variable/data section

            ORG RAMStart
;*******************************************************************
;           * Lab Project: Robot Guidance Challenge (9S32C) *
;*******************************************************************
;                       equates section
;*******************************************************************
LCD_DAT         EQU   PORTB       ; LCD data port, bits - PB7,...,PB0
LCD_CNTR        EQU   PTJ         ; LCD control port, bits - PJ7(E),PJ6(RS)   
LCD_E           EQU   $80         ; LCD E-signal pin
LCD_RS          EQU   $40         ; LCD RS-signal pin 
FWD_INT         EQU   69  ; 3 second delay (at 23Hz)
REV_INT         EQU   69  ; 3 second delay (at 23Hz)
FWD_TRN_INT     EQU   46  ; 2 second delay (at 23Hz)
REV_TRN_INT     EQU   46  ; 2 second delay (at 23Hz)

START           EQU   0
FWD             EQU   1
REV             EQU   2
ALL_STP         EQU   3
FWD_TRN         EQU   4
REV_TRN         EQU   5

INTPTHA         EQU   $8A                       ; Interupt Path A detection 
INTPTHB         EQU   $CF                       ; ""
INTPTHC         EQU   $CF                       ; ""
INTPTHD         EQU   $CF                       ; ""

INTPTHE         EQU   $68                       ; If SENSOR_LINE < INTPTHE, sensor E has obstacle, therefore robot shifts right
INTPTHF         EQU   $75                       ; If SENSOR_LINE > INTPTHF sensor F has obstacle, therefore robot shifts left


CLEAR_HOME      EQU   $01                         ; Clear the display and home the cursor
LCD_SEC_LINE    EQU   64                        ; Starting addr. of 2nd line of LCD (note decimal value!)
NULL            EQU   00                        ; The string ’null terminator’


; variable section
;*******************************************************************
                ORG $3000     ; Where our TOF counter register lives
TOF_COUNTER     dc.b 0        ; The timer, incremented at 23Hz
CRNT_STATE      dc.b 6        ; Current state register
T_FWD           ds.b 1        ; FWD time
T_REV           ds.b 1        ; REV time
T_FWD_TRN       ds.b 1        ; FWD_TURN time
T_REV_TRN       ds.b 1        ; REV_TURN time
TEN_THOUS       ds.b 1        ; 10,000 digit
THOUSANDS       ds.b 1        ; 1,000 digit
HUNDREDS        ds.b 1        ; 100 digit
TENS            ds.b 1        ; 10 digit
UNITS           ds.b 1        ; 1 digit
NO_BLANK        ds.b 1        ; Used in ’leading zero’ blanking by BCD2ASC
BCD_SPARE       RMB  10       ; Extra space for decimal point and string terminator

ATDDR0L         RMB   10
ATDCTL2         RMB   10
ATDCTL3         RMB   10
ATDCTL4         RMB   10
ATDCTL5         RMB   10
ATDDIEN:        RMB   10
ATDSTAT0:       RMB   10

SENSOR_LINE     FCB $01       ; Sensor (E-F) Storage for guider sensor readings
SENSOR_BOW      FCB $23       ; (Sensor A - Front Sensor) which is initialized to test values 
SENSOR_PORT     FCB $45       ; (Sensor B - Left) 
SENSOR_MID      FCB $67       ; (Sensor C - Middle )
SENSOR_STAR     FCB $89       ; (Sensor D - Right) 
SENSOR_NUM      RMB 1         ; The currently selected sensor
TEMP            RMB 1         ; Temporary location

TOP_LINE        RMB 20        ; Top line of display
                FCB NULL         ; terminated by null

BOT_LINE        RMB 20        ; Bottom line of display
                FCB NULL         ; terminated by null

CLEAR_LINE      FCC ' '
                FCB NULL         ; terminated by null
                

DETA            DC.B  0        ; Pattern detection for sensor A (NO PATH = 0,PATH = 1)
DETB            DC.B  0        ; Pattern detection for sensor B (NO PATH = 0,PATH = 1)
DETC            DC.B  0        ; Pattern detection for sensor C (NO PATH = 0,PATH = 1)
DETD            DC.B  0        ; Pattern detection for sensor D (NO PATH = 0,PATH = 1)
DETE            DC.B  0        ; Pattern detection for sensor E (NO PATH = 0,PATH = 1)
DETF            DC.B  0        ; Pattern detection for sensor F (NO PATH = 0,PATH = 1)
 

; code section
;*******************************************************************
                ORG   $4000     ; Where the code starts --------------------
Entry:                                                          ; |
_Startup:                                                       ; |
                CLI             ; Enable interrupts               |
                LDS   #$4000 ; Initialize the stack pointer     ; I
                BSET  DDRA,%00000011 ; STAR_DIR, PORT_DIR         N
                BSET  DDRT,%00110000 ; STAR_SPEED, PORT_SPEED     I 
                                                                ; T
                JSR   initAD ; Initialize ATD converter           I       
                                                                ; A
                JSR   initLCD ; Initialize the LCD                L
                JSR   clrLCD ; Clear LCD & home cursor            I        
                                                                ; Z
                LDX   #msg1 ; Display msg1                        A
                JSR   putsLCD ; "                                 T
                                                                ; I
                LDAA  #$C0 ; Move LCD cursor to the 2nd row       O
                JSR   cmd2LCD ;                                   N
                LDX   #msg2 ; Display msg2                        |
                JSR   putsLCD ; "                                 |
                
                LDAA  #$CF    ; ; Move LCD cursor to the end of msg2        
                JSR   cmd2LCD 
                LDX   #msg3     ; Display msg3                    |
                JSR   putsLCD  ; "       
                
                LDAA  #$8F    ; Move LCD cursor to the end of msg3       
                JSR   cmd2LCD 
                LDX   #msg4     ; Display msg4                    |
                JSR   putsLCD  ; "    
                                                                 ;|
                JSR   ENABLE_TOF ; Jump to TOF initialization   ----

;---------------------------------------------------------------------------
;                     Display Sensors
MAIN            JSR     G_LEDS_ON ; Enable the guider LEDs
                JSR     READ_SENSORS ; Read the 5 guider sensors
                JSR     G_LEDS_OFF ; Disable the guider LEDs
                JSR     DISPLAY_SENSORS ; and write them to the LCD
                LDY     #6000 ; 300 ms delay to avoid
                JSR     del_50us ; display artifacts
                JSR     UPDT_DISPL ; -----------------------------  M
                LDAA    CRNT_STATE ;                                A
                JSR     DISPATCHER ;                                I
                BRA     MAIN ; ----------------------------         N
 
; data section
;---------------------------------------------------------------------------
msg1            dc.b  "Battery volt ",0
msg2            dc.b  "State ",0 
msg3            dc.b  "Sensor ",0 
msg4            dc.b  "Bumper ",0 

tab             dc.b  "START ",0
                dc.b  "FWD ",0
                dc.b  "REV ",0
                dc.b  "ALL_STP",0
                dc.b  "FWD_TRN",0
                dc.b  "REV_TRN",0

; subroutine section
;*******************************************************************
DISPATCHER    CMPA    #START        ; If it’s the START state -----------------
              BNE     NOT_START     ;                                         |
              JSR     START_ST      ; then call START_ST routine              D
              BRA     DISP_EXIT     ; and exit                                I
                                                                            ; S
NOT_START     CMPA    #FWD          ; Else if it’s the FORWARD state          P
              BNE     NOT_FWD       ; then call the FORWARD routine         ; A
              JSR     FWD_ST                                                ; T
              JMP     DISP_EXIT     ; and exit                              
                                                                            ; C
                                                                            ; H
NOT_FWD       CMPA    #REV          ; Else if it’s the REVERSE state          E
              BNE     NOT_REV                                               ; R
              JSR     REV_ST        ; then call the REVERSE routine           |
              JMP     DISP_EXIT     ; and exit                                |
                                                                            ; |
NOT_REV       CMPA    #ALL_STP      ; Else if it’s the ALL_STOP state
                                                                            ; |
              BNE     NOT_ALL_STOP                                          ; |
              JSR     ALL_STP_ST      ; then call the ALL_STOP routine        |
              JMP     DISP_EXIT       ; and exit                              |
                                                                            ; |
NOT_ALL_STOP  CMPA    #FWD_TRN        ; Else if it’s the FORWARD_TURN state   |
              BNE     NOT_FWD_TRN                                           ; |
              JSR     FWD_TRN_ST       ; then call the FORWARD_TURN routine   |
              JMP     DISP_EXIT       ; and exit                              |
                                                                            ; |
NOT_FWD_TRN   CMPA    #REV_TRN      ; Else if it’s the REV_TRN state          | 
              BNE     NOT_REV_TRN                                           ; |
              JSR     REV_TRN_ST    ; then call REV_TRN_ST routine            | 
              BRA     DISP_EXIT     ; and exit                              ; |  
                                                                             
NOT_REV_TRN   SWI                  ; Else the CRNT_ST is not defined, so stop 
DISP_EXIT     RTS                  ; Exit from the state dispatcher ----------
;*******************************************************************
; *               Motor Speed Control          *
;*******************************************************************

STARON            LDAA    PTT
                  ORAA    #%00100000
                  STAA    PTT
                  RTS

STAROFF           LDAA    PTT
                  ANDA    #%11011111
                  STAA    PTT
                  RTS
                  
STARFWD           LDAA    PORTA
                  ANDA    #%11111101
                  STAA    PORTA
                  RTS    
                  
STARREV           LDAA    PORTA
                  ORAA    #%00000010
                  STAA    PORTA
                  RTS     
                  
PORTON            LDAA    PTT
                  ORAA    #%00010000
                  STAA    PTT
                  RTS                        

PORTOFF           LDAA    PTT
                  ANDA    #%11101111
                  STAA    PTT
                  RTS       
                  
PORTFWD           LDAA    PORTA
                  ANDA    #%11111110
                  STAA    PORTA
                  RTS

PORTREV           LDAA    PORTA
                  ORAA    #%00000001
                  STAA    PTH
                  RTS
;*******************************************************************
; * Rotation Counters + Bumper Detectors + Navigation Manager     *
;*******************************************************************

START_ST      BRCLR   PORTAD0,$04,NO_FWD ; If /FWD_BUMP  
              JSR     INIT_FWD            ; Initialize the FORWARD state
              MOVB    #FWD, CRNT_STATE    ; Go into the FORWARD state
              BRA     START_EXIT    
 
NO_FWD        NOP                  ; Else
START_EXIT    RTS                  ; return to the MAIN routine
;---------------------------------------------------------------------------

FWD_ST        BRSET   PORTAD0,$04,NO_FWD_BUMP  ; If FWD_BUMP then
              JSR     INIT_REV                 ; initialize the REVERSE routine
              MOVB    #REV, CRNT_STATE         ; set the state to REVERSE
              JMP     FWD_EXIT                 ; and return

NO_FWD_BUMP   BRSET   PORTAD0,$08,NO_REAR_BUMP  ; If REAR_BUMP, then we should stop
              JSR     INIT_ALL_STP              ; so initialize the ALL_STOP state
              MOVB    #ALL_STP_ST,CRNT_STATE    ; and change state to ALL_STOP
              JMP     FWD_EXIT                  ; and return

NO_REAR_BUMP  LDAA    TOF_COUNTER               ; If Tc>Tfwd then
              CMPA    T_FWD                     ; the robot should make a turn
              BNE     NO_FWD_TRN                ; so
              JSR     INIT_FWD_TRN              ; initialize the FORWARD_TURN state
              MOVB    #FWD_TRN, CRNT_STATE       ; and go to that state
              JMP     FWD_EXIT              

NO_FWD_INTERSECT               

NO_FWD_TRN    NOP                              ; Else
FWD_EXIT      RTS                              ; return to the MAIN routine
;---------------------------------------------------------------------------
REV_ST        LDAA    TOF_COUNTER       ; If Tc>Trev then
              CMPA    T_REV             ; the robot should make a FWD turn
              BNE     NO_REV_TRN        ; so
              JSR     INIT_REV_TRN      ; initialize the REV_TRN state
              MOVB    #REV_TRN,CRNT_STATE ; set state to REV_TRN
              BRA     REV_EXIT              ; and return

NO_REV_TRN    NOP                           ; Else
REV_EXIT      RTS                           ; return to the MAIN routine
;---------------------------------------------------------------------------
ALL_STP_ST    BRSET  PORTAD0,$04,NO_START   ; If FWD_BUMP
              BCLR   PTT,%00110000          ; initialize the START state (both motors off)
              MOVB   #START,CRNT_STATE      ; set the state to START
              BRA    ALL_STP_EXIT           ; and return

NO_START      NOP                           ; Else
ALL_STP_EXIT  RTS                           ; return to the MAIN routine
;---------------------------------------------------------------------------
FWD_TRN_ST    LDAA   TOF_COUNTER            ; If Tc>Tfwdturn then
              CMPA   T_FWD_TRN              ; the robot should go FWD
              BNE    NO_FWD_FT              ; so
              JSR    INIT_FWD               ; initialize the FWD state
              MOVB   #FWD,CRNT_STATE        ; set state to FWD
              BRA    FWD_TRN_EXIT           ; and return

NO_FWD_FT     NOP                           ; Else
FWD_TRN_EXIT  RTS                           ; return to the MAIN routine
;---------------------------------------------------------------------------
REV_TRN_ST    LDAA   TOF_COUNTER            ; If Tc>Trevturn then
              CMPA   T_REV_TRN              ; the robot should go FWD

             BNE NO_FWD_RT        ; so
             JSR INIT_FWD         ; initialize the FWD state
             MOVB #FWD,CRNT_STATE ; set state to FWD
             BRA REV_TRN_EXIT     ; and return

NO_FWD_RT    NOP                  ; Else
REV_TRN_EXIT RTS                  ; return to the MAIN routine

;---------------------------------------------------------------------------
INIT_FWD     BCLR PORTA,%00000011 ; Set FWD direction for both motors
             BCLR PTT,%00110000   ; Turn on the drive motors
             LDAA TOF_COUNTER     ; Mark the fwd time Tfwd
             ADDA #FWD_INT
             STAA T_FWD
             RTS

;---------------------------------------------------------------------------
INIT_REV     BSET PORTA,%00000011 ; Set REV direction for both motors
             BSET PTT,%00110000   ; Turn on the drive motors
             LDAA TOF_COUNTER     ; Mark the fwd time Tfwd
             ADDA #REV_INT
             STAA T_REV
             RTS
             
;---------------------------------------------------------------------------
INIT_ALL_STP BCLR PTT,%00110000   ; Turn off the drive motors
             RTS
             
;---------------------------------------------------------------------------
INIT_FWD_TRN BSET PORTA,%00000010 ; Set REV dir. for STARBOARD (right) motor
             LDAA TOF_COUNTER     ; Mark the fwd_turn time Tfwdturn
             ADDA #FWD_TRN_INT
             STAA T_FWD_TRN
             RTS
             
;---------------------------------------------------------------------------
INIT_REV_TRN BCLR PORTA,%00000010 ; Set FWD dir. for STARBOARD (right) motor
             LDAA TOF_COUNTER     ; Mark the fwd time Tfwd
             ADDA #REV_TRN_INT
             STAA T_REV_TRN
             RTS
;*******************************************************************
; *                Sensor Scanner + Guidance Routine       *
;******************************************************************* 
; Checking Sensor
CHECK_A         LDAA  SENSOR_BOW                ; If SENSOR_BOW > 
                CMPA  #INTPTHA                  ; INTPTHA
                BLO   CHECK_B                   ;
                INC   DETA                      ; Set Sensor A detection to 1

CHECK_B         LDAA  SENSOR_PORT               ; If SENSOR_PORT > 
                CMPA  #INTPTHB                  ;  INTPTHB
                BLO   CHECK_C                   ;
                INC   DETB                      ; Set Sensor B detection to 1

CHECK_C         LDAA  SENSOR_MID                ; If SENSOR_MID >
                CMPA  #INTPTHC                  ; INTPTHC
                BLO   CHECK_D                   ;
                INC   DETC                      ; Set Sensor C detection to 1
                
CHECK_D         LDAA  SENSOR_STAR               ; If SENSOR_STBD >
                CMPA  #INTPTHD                  ; INTPTHD
                BLO   CHECK_E                   ;
                INC   DETD                      ; Set Sensor D detection to 1

CHECK_E         LDAA  SENSOR_LINE               ; If SENSOR_LINE <
                CMPA  #INTPTHE                  ; INTPTHE
                BHI   CHECK_F                   ;
                INC   DETE                      ; Set Sensor E detection to 1
                
CHECK_F         LDAA  SENSOR_LINE               ; If SENSOR_LINE >
                CMPA  #INTPTHF                  ; INTPTHF
                BLO   UPDATED                   ;
                INC   DETF                      ; Set Sensor F detection to 1

UPDATED         RTS
;---------------------------------------------------------------------------
G_LEDS_ON       BSET  PORTA,%00100000           ; Set bit 5
                RTS
;---------------------------------------------------------------------------
G_LEDS_OFF      BCLR  PORTA,%00100000           ; Clear bit 5
                RTS
;---------------------------------------------------------------------------
; Read Sensor
READ_SENSORS  CLR     SENSOR_NUM      ; Select sensor number 0
              LDX     #SENSOR_LINE    ; Point at the start of the sensor array
RS_MAIN_LOOP  LDAA    SENSOR_NUM      ; Select the correct sensor input
              JSR     SELECT_SENSOR     ; on the hardware
              LDY     #400            ; 20 ms delay to allow the
              JSR     del_50us        ; sensor to stabilize
              LDAA    #%10000001      ; Start A/D conversion on AN1
              STAA    ATDCTL5
              BRCLR   ATDSTAT0,$80,*  ; Repeat until A/D signals done
              LDAA    ATDDR0L         ; A/D conversion is complete in ATDDR0L
              STAA    0,X             ; so copy it to the sensor register
              CPX     #SENSOR_STAR    ; If this is the last reading
              BEQ     RS_EXIT         ; Then exit
              INC     SENSOR_NUM      ; Else, increment the sensor number
              INX                       ; and the pointer into the sensor array
              BRA     RS_MAIN_LOOP     ; and do it again
RS_EXIT       RTS

;---------------------------------------------------------------------------
; Select Sensor

SELECT_SENSOR PSHA                      ; Save the sensor number for the moment
              LDAA    PORTA             ; Clear the sensor selection bits to zeros
              ANDA    #%11100011 
              STAA    TEMP              ; and save it into TEMP
              PULA                    ; Get the sensor number
              ASLA                      ; Shift the selection number left, twice
              ASLA 
              ANDA    #%00011100           ; Clear irrelevant bit positions
              ORAA    TEMP                   ; OR it into the sensor bit positions
              STAA    PORTA                  ; Update the hardware
              RTS

;---------------------------------------------------------------------------
; Position the Cursor
LCD_POS_CRSR  ORAA    #%10000000 ; Set the high bit of the control word
              JSR     cmd2LCD       ; and set the cursor address
              RTS

;---------------------------------------------------------------------------
; utility subroutines

initLCD      BSET    DDRS,%11110000          ; configure pins PS7,PS6,PS5,PS4 for output
             BSET    DDRE,%10010000          ; configure pins PE7,PE4 for output
             LDY     #2000                   ; wait for LCD to be ready
             JSR     del_50us                ; -"-
             LDAA    #$28                    ; set 4-bit data, 2-line display
             JSR     cmd2LCD                 ; -"-
             LDAA    #$0C                    ; display on, cursor off, blinking off
             JSR     cmd2LCD                 ; -"-
             LDAA    #$06                    ; move cursor right after entering a character
             JSR     cmd2LCD                 ; -"-
             RTS

;*******************************************************************
clrLCD       LDAA    #$01                  ; clear cursor and return to home position
             JSR     cmd2LCD               ; -"-
             LDY     #40                   ; wait until "clear cursor" command is complete
             JSR     del_50us              ; -"-
             RTS
;*******************************************************************
del_50us     PSHX                     ; (2 E-clk) Protect the X register
eloop        LDX #300                 ; (2 E-clk) Initialize the inner loop counter
iloop        NOP                      ; (1 E-clk) No operation
             DBNE X,iloop             ; (3 E-clk) If the inner cntr not 0, loop again
             DBNE Y,eloop             ; (3 E-clk) If the outer cntr not 0, loop again
             PULX                     ; (3 E-clk) Restore the X register
             RTS                      ; (5 E-clk) Else return
         
;*******************************************************************
cmd2LCD:     BCLR    LCD_CNTR,LCD_RS       ; select the LCD Instruction Register (IR)
             JSR     dataMov               ; send data to IR
             RTS

;*******************************************************************
putsLCD      LDAA    1,X+                  ; get one character from the string
             BEQ     donePS                ; reach NULL character?
             JSR     putcLCD
             BRA     putsLCD
donePS       RTS
;*******************************************************************
putcLCD      BSET    LCD_CNTR,LCD_RS       ; select the LCD Data register (DR)
             JSR     dataMov               ; send data to DR
             RTS

;*******************************************************************
dataMov      BSET    LCD_CNTR,LCD_E        ; pull the LCD E-sigal high
             STAA    LCD_DAT               ; send the upper 4 bits of data to LCD
             BCLR    LCD_CNTR,LCD_E        ; pull the LCD E-signal low to complete the write oper.
             LSLA                          ; match the lower 4 bits with the LCD data pins
             LSLA                          ; -"-
             LSLA                          ; -"-
             LSLA                          ; -"-
             BSET    LCD_CNTR,LCD_E        ; pull the LCD E signal high
             STAA    LCD_DAT               ; send the lower 4 bits of data to LCD
             BCLR    LCD_CNTR,LCD_E        ; pull the LCD E-signal low to complete the write oper.
             LDY     #1                    ; adding this delay will complete the internal
             JSR     del_50us              ; operation for most instructions
             RTS
;*******************************************************************
initAD       MOVB    #$C0,ATDCTL2          ;power up AD, select fast flag clear
             JSR     del_50us              ;wait for 50 us
             MOVB    #$00,ATDCTL3          ;8 conversions in a sequence
             MOVB    #$85,ATDCTL4          ;res=8, conv-clks=2, prescal=12
             BSET    ATDDIEN,$0C           ;configure pins AN03,AN02 as digital inputs
             RTS

;*******************************************************************
int2BCD
                XGDX                  ;Save the binary number into .X
                LDAA #0               ;Clear the BCD_BUFFER
                STAA TEN_THOUS
                STAA THOUSANDS
                STAA HUNDREDS
                STAA TENS
                STAA UNITS
                STAA BCD_SPARE
                STAA BCD_SPARE+1
            

                CPX #0                ;Check for a zero input
*               BEQ CON_EXIT          ;and if so, exit

                XGDX                  ;Not zero, get the binary number back to .D as dividend
                LDX #10               ;Setup 10 (Decimal!) as the divisor
                IDIV                  ;Divide: Quotient is now in .X, remainder in .D
                STAB UNITS            ;Store remainder
                CPX #0                ;If quotient is zero,
*               BEQ CON_EXIT          ;then exit

                XGDX                  ;else swap first quotient back into .D
                LDX #10               ;and setup for another divide by 10
                IDIV
                STAB TENS
                CPX #0
*               BEQ CON_EXIT

                XGDX                  ;Swap quotient back into .D
                LDX #10               ;and setup for another divide by 10
                IDIV
                STAB HUNDREDS
                CPX #0
*               BEQ CON_EXIT

                XGDX                  ;Swap quotient back into .D
                LDX #10               ;and setup for another divide by 10
                IDIV
                STAB THOUSANDS
                CPX #0
*               BEQ CON_EXIT

                XGDX                  ;Swap quotient back into .D
                LDX #10               ;and setup for another divide by 10
                IDIV
                STAB TEN_THOUS

*               CON_EXIT RTS          ;We're done the conversion
                      
             
;*******************************************************************
BCD2ASC
             LDAA #0                     ;Initialize the blanking flag
             STAA NO_BLANK
*
C_TTHOU 
             LDAA TEN_THOUS              ;Check the 'ten_thousands' digit
             ORAA NO_BLANK
             BNE NOT_BLANK1
*
ISBLANK1 
             LDAA #' '                   ; It's blank
             STAA TEN_THOUS              ;so store a space
             BRA C_THOU                  ;and check the 'thousands' digit
*
NOT_BLANK1 
             LDAA TEN_THOUS              ;Get the 'ten_thousands' digit
             ORAA #$30                   ;Convert to ascii
             STAA TEN_THOUS
             LDAA #$1                    ;Signal that we have seen a 'non-blank' digit
             STAA NO_BLANK
*
C_THOU 
                LDAA THOUSANDS              ;Check the thousands digit for blankness
                ORAA NO_BLANK               ;If it's blank and 'no-blank' is still zero
                BNE NOT_BLANK2
*
ISBLANK2 
                LDAA #' '                   ;Thousands digit is blank
                STAA THOUSANDS              ;so store a space
                BRA C_HUNS                  ;and check the hundreds digit
*
NOT_BLANK2 
                LDAA THOUSANDS              ;(similar to 'ten_thousands' case)
                ORAA #$30
                STAA THOUSANDS
                LDAA #$1
                STAA NO_BLANK
*
C_HUNS
                LDAA HUNDREDS               ;Check the hundreds digit for blankness
                ORAA NO_BLANK               ;If it's blank and 'no-blank' is still zero
                BNE NOT_BLANK3
*
ISBLANK3 
                LDAA #' '                   ;Hundreds digit is blank
                STAA HUNDREDS               ;so store a space
                BRA C_TENS                  ;and check the tens digit
*
NOT_BLANK3 
                LDAA HUNDREDS               ;(similar to 'ten_thousands' case)
                ORAA #$30
                STAA HUNDREDS
                LDAA #$1
                STAA NO_BLANK
*
C_TENS 
                LDAA TENS                   ;Check the tens digit for blankness
                ORAA NO_BLANK               ;If it's blank and 'no-blank' is still zero
                BNE NOT_BLANK4
*
ISBLANK4
                LDAA #' '                   ;Tens digit is blank
                STAA TENS                   ;so store a space
                BRA C_UNITS                 ;and check the units digit
*
NOT_BLANK4 
                LDAA TENS                   ;(similar to 'ten_thousands' case)
                ORAA #$30
                STAA TENS
*
C_UNITS 
                LDAA UNITS                  ;No blank check necessary, convert to ascii.
                ORAA #$30
                STAA UNITS
*
                RTS                         ;We're done
;*******************************************************************
HEX_TABLE       FCC   '0123456789ABCDEF' ; Table for converting values
BIN2ASC         PSHA                        ; Save a copy of the input number on the stack
                TAB                     ; and copy it into ACCB
                ANDB   #%00001111         ; Strip off the upper nibble of ACCB
                CLRA                  ; D now contains 000n where n is the LSnibble
                ADDD    #HEX_TABLE ; Set up for indexed load
                XGDX
                LDAA    0,X ; Get the LSnibble character
                PULB ; Retrieve the input number into ACCB
                PSHA ; and push the LSnibble character in its place
                RORB ; Move the upper nibble of the input number
                RORB ; into the lower nibble position.
                RORB
                RORB
                ANDB    #%00001111 ; Strip off the upper nibble
                CLRA ; D now contains 000n where n is the MSnibble
                ADDD    #HEX_TABLE ; Set up for indexed load
                XGDX
                LDAA    0,X ; Get the MSnibble character into ACCA
                PULB ; Retrieve the LSnibble character into ACCB
                RTS

;*******************************************************************
;*                     Display Sensors                 *
;*******************************************************************
 
DP_FRONT_SENSOR EQU   TOP_LINE + 3
DP_PORT_SENSOR  EQU   BOT_LINE + 0
DP_MID_SENSOR   EQU   BOT_LINE + 3
DP_STBD_SENSOR  EQU   BOT_LINE + 6
DP_LINE_SENSOR  EQU   BOT_LINE + 9

DISPLAY_SENSORS LDAA  SENSOR_BOW ; Get the FRONT sensor value
                JSR   BIN2ASC ; Convert to ascii string in D
                LDX   #DP_FRONT_SENSOR ; Point to the LCD buffer position
                STD   0,X ; and write the 2 ascii digits there
                LDAA  SENSOR_PORT ; Repeat for the PORT value
                JSR   BIN2ASC
                LDX   #DP_PORT_SENSOR
                STD   0,X
                LDAA  SENSOR_MID ; Repeat for the MID value
                JSR   BIN2ASC
                LDX   #DP_MID_SENSOR
                STD   0,X
                LDAA  SENSOR_STAR ; Repeat for the STARBOARD value
                JSR   BIN2ASC
                LDX   #DP_STBD_SENSOR
                STD   0,X
                LDAA  SENSOR_LINE ; Repeat for the LINE value
                JSR   BIN2ASC
                LDX   #DP_LINE_SENSOR
                STD   0,X
                LDAA  #CLEAR_HOME ; Clear the display and home the cursor
                JSR   cmd2LCD ; "
                LDY   #40 ; Wait 2 ms until "clear display" command is complete
                JSR   del_50us
                LDX   #TOP_LINE ; Now copy the buffer top line to the LCD
                JSR   putsLCD
                LDAA  #LCD_SEC_LINE ; Position the LCD cursor on the second line
                JSR   LCD_POS_CRSR
                LDX   #BOT_LINE ; Copy the buffer bottom line to the LCD
                JSR   putsLCD
                RTS


;*******************************************************************
;*         Update Display (Battery Voltage + Current State)        *
;*******************************************************************

UPDT_DISPL   MOVB #$90,ATDCTL5 ; R-just., uns., sing. conv., mult., ch=0, start
            BRCLR ATDSTAT0,$80,* ; Wait until the conver. seq. is complete
  
             LDAA ATDDR0L ; Load the ch0 result - battery volt - into A
                           ; Display the battery voltage
           
;-------------------------
  
             LDAA #$C6 ; Move LCD cursor to the 2nd row, end of msg2
             JSR cmd2LCD ;
        
            LDAB CRNT_STATE ; Display current state
            LSLB ; "
            LSLB ; "
            LSLB ; "
            LDX #tab ; "
            ABX ; "
           JSR putsLCD ; "
        
            RTS


;*******************************************************************
ENABLE_TOF      LDAA    #%10000000
                STAA    TSCR1       ; Enable TCNT
                STAA    TFLG2       ; Clear TOF
                LDAA    #%10000100  ; Enable TOI and select prescale factor equal to 16
                STAA    TSCR2
                RTS
;*******************************************************************
TOF_ISR         INC     TOF_COUNTER
                LDAA    #%10000000   ; Clear
                STAA    TFLG2        ; TOF
                RTI      

;*******************************************************************

;**************************************************************
;*                 Interrupt Vectors                          *
;**************************************************************
            ORG   $FFFE
            DC.W  Entry           ; Reset Vector