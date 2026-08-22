
	list	p=16F73
	#include <p16F73.inc>
;#include <p16F873a.inc>
	__CONFIG _HS_OSC & _WDT_OFF & _PWRTE_ON & _CP_OFF & _BODEN_ON

;---------------ユーザー定義定数-----------------------------------------------
WRITE		equ	h'a0'
READ		equ	h'a1'

tx_ch1		equ	1		;portc
tx_ch2		equ	0
SCL		equ	3
SDA		equ	4
rx_ch1		equ	7
rx_ch2		equ	6
rx_ch3		equ	2
rx_ch4		equ	5

cs0		equ	0		;porta
cs1		equ	1
cs2		equ	2
cs3		equ	3
cs4		equ	4
cs5		equ	5

_mod_reset	equ	0		;flag
_mod_sensadj	equ	1
_mod_send	equ	2
_mod_check	equ	3
_mod_setting	equ	4
_mod_cont_const	equ	5
_mod_save	equ	6
set_ch		equ	7

_s0_ON		equ	0		;flag_2
_s1_ON		equ	1
_s2_ON		equ	2
_reset_ON	equ	3
_send_ON	equ	4
_sensadj_ON	equ	5
_clear_ON	equ	6
_enter_ON	equ	7

ten_1		equ	0		;flag_3
ten_2		equ	1
len_1		equ	2
len_2		equ	3
ten_set		equ	4
error_set	equ	5
stop_set	equ	6
const_set	equ	7

time_1_1	equ	0		;flag_4
time_1_2	equ	1
time_1_3	equ	2
time_1_4	equ	3
time_2		equ	4
time_3		equ	5
time_4		equ	6

edge_1		equ	0		;flag_5
edge_2		equ	1
edge_3		equ	2
edge_4		equ	3
dp_ON		equ	4
dpb_short	equ	5
disp_ten	equ	6
disp_len	equ	7

update_1	equ	0		;flag_6
update_2	equ	1
update_3	equ	2
update_4	equ	3
;---------------ユーザー定義メモリ---------------------------------------------
		cblock	h'20'
		wtemp			;割り込み処理時待避用
		statustemp
		pclathtemp
		flag
		flag_2
		flag_3
		flag_4
		flag_5
		flag_6
		scntr			;LED表示列カウンタ
		cscntr
		cntr
		cntr_1
		cntr34_1		;糸長データ読み込み用カウンタ(CH1_0)
		cntr34_2		;糸長データ読み込み用カウンタ(CH2_0)
		cntr34_3		;糸長データ読み込み用カウンタ(CH1_1)
		cntr34_4		;糸長データ読み込み用カウンタ(CH2_1)
		cntr34			;300bpsデータ送信用カウンタ(3.4ms)
		cntr20			;LED表示用カウンタ(2ms)
		cntr50			;EEPROM書き込み待ちカウンタ(5ms)
		cntr500			;50msカウンタ
		blink_cntr		;デシマルブリンク用カウンタ
		blink_cntr_2
		shift_cntr		;LED文字シフトカウンタ
		txbit_cntr		;送信ビットカウンタ
		txbyte_cntr		;送信バイトカウンタ
		bitcnt			;I2C用ビットカウンタ
		rxbit_cntr_1		;受信ビットカウンタ(CH1_0)
		rxbit_cntr_2		;受信ビットカウンタ(CH2_0)
		rxbit_cntr_3		;受信ビットカウンタ(CH1_1)
		rxbit_cntr_4		;受信ビットカウンタ(CH2_1)
		rxbyte_cntr_1		;受信バイトカウンタ(CH1_0)
		rxbyte_cntr_2		;受信バイトカウンタ(CH2_0)
		rxbyte_cntr_3		;受信バイトカウンタ(CH1_1)
		rxbyte_cntr_4		;受信バイトカウンタ(CH2_1)
		savebyte_cntr		;保存バイトカウンタ
		_tmp_1
		_tmp_2
		_tmp_3
		fsrtemp_1
		fsrtemp_2
		_prm_1
		rreg_1			;送信シフトレジスタ(CH1)
		rreg_2			;送信シフトレジスタ(CH2)
		blink_reg		;文字ブリンク時待避レジスタ
		wraddrl			;EEPROMライト用アドレス
		rdaddrl			;EEPROMリード用アドレス
		eedt			;EEPROMデータ
		i2cbyte			;EEPROM受信データ
		windlen1_3		;巻き取り長４バイト目(CH1_0)
		windlen2_3		;巻き取り長４バイト目(CH2_0)
		windlen3_3		;巻き取り長４バイト目(CH1_1)
		windlen4_3		;巻き取り長４バイト目(CH2_1)
		setreg_0		;数値入力時表示文字待避レジスタ
		setreg_1
		setreg_2
		setreg_3
		setreg_4
		setreg_5

		cs0_s0			;表示文字レジスタ
		cs0_s1
		cs0_s2
		cs0_s3
		cs0_s4
		cs0_s5
		cs1_s0
		cs1_s1
		cs1_s2
		cs1_s3
		cs1_s4
		cs1_s5
		cs2_s0
		cs2_s1
		cs2_s2
		cs2_s3
		cs2_s4
		cs2_s5
		cs3_s0
		cs3_s1
		cs3_s2
		cs3_s3
		cs3_s4
		cs3_s5
		cs4_s0
		cs4_s1
		cs4_s2
		cs4_s3
		cs4_s4
		cs4_s5

		endc

		cblock	h'a3'

		lench1_0		;設定糸長(CH1_0)
		lench1_1
		lench1_2
		tens_ch1		;設定張力(CH1_0)
		checksum_1_1		;チェックサム(CH1_0)

		lench3_0		;設定糸長(CH1_1)
		lench3_1
		lench3_2
		tens_ch3		;設定張力(CH1_1)
		checksum_1_3		;チェックサム(CH1_1)

		lench2_0		;設定糸長(CH2_0)
		lench2_1
		lench2_2
		tens_ch2		;設定張力(CH2_0)
		checksum_1_2		;チェックサム(CH2_0)

		lench4_0		;設定糸長(CH2_1)
		lench4_1
		lench4_2
		tens_ch4		;設定張力(CH2_1)
		checksum_1_4		;チェックサム(CH2_1)

		stoptime_ch1
		rotate_ch1
		errortens_ch1
		errortime_ch1
		checksum_2_1		;チェックサム(CH1_0)

		stoptime_ch3
		rotate_ch3
		errortens_ch3
		errortime_ch3
		checksum_2_3		;チェックサム(CH1_1)

		stoptime_ch2
		rotate_ch2
		errortens_ch2
		errortime_ch2
		checksum_2_2		;チェックサム(CH2_0)

		stoptime_ch4
		rotate_ch4
		errortens_ch4
		errortime_ch4
		checksum_2_4		;チェックサム(CH2_1)

		const_kp_ch1
		const_ti_ch1
		dam_1_1
		dam_1_2
		dam_1_3
		const_kp_ch3
		const_ti_ch3
		dam_2_1
		dam_2_2
		dam_2_3
		const_kp_ch2
		const_ti_ch2
		dam_3_1
		dam_3_2
		dam_3_3
		const_kp_ch4
		const_ti_ch4

		windlen1_0		;巻き取り糸長CH1(1～3バイト)
		windlen1_1
		windlen1_2
		realtens1_0
		realtens1_1
		windlen3_0		;巻き取り糸長CH3(1～3バイト)
		windlen3_1
		windlen3_2
		realtens3_0
		realtens3_1
		windlen2_0		;巻き取り糸長CH2(1～3バイト)
		windlen2_1
		windlen2_2
		realtens2_0
		realtens2_1
		windlen4_0		;巻き取り糸長CH4(1～3バイト)
		windlen4_1
		windlen4_2
		realtens4_0
		realtens4_1
                len1_temp   
                len1_1_temp
                len1_2_temp
                length_change  
                len2_1_temp
                len2_2_temp   
                len2_temp         
                fresh_time_cnt 
		endc

;**************************************************************************
		org	0h
		goto	start

;**************************************************************************
		org	4h

Push
		movwf	wtemp			; コンテキスト保存
		swapf	STATUS,w
		clrf	STATUS
		movwf	statustemp
		movf	PCLATH,w
		movwf	pclathtemp
		clrf	PCLATH

		bcf	INTCON,T0IF
		movlw	d'13'
		movwf	TMR0

		decfsz	cntr34_1,f		;CH1 110bpsデータ受信用Counter
		goto	$+4
		movlw	d'17'
		movwf	cntr34_1
		bsf	flag_4,time_1_1

		decfsz	cntr34_2,f		;CH2 110bpsデータ受信用Counter
		goto	$+4
		movlw	d'17'
		movwf	cntr34_2
		bsf	flag_4,time_1_2

		decfsz	cntr34_3,f		;CH1 110bpsデータ受信用Counter
		goto	$+4
		movlw	d'17'
		movwf	cntr34_3
		bsf	flag_4,time_1_3

		decfsz	cntr34_4,f		;CH1 110bpsデータ受信用Counter
		goto	$+4
		movlw	d'17'
		movwf	cntr34_4
		bsf	flag_4,time_1_4

		decfsz	cntr34,f		;300bpsデータ送信用Counter
		goto	$+4
		movlw	d'17'
		movwf	cntr34
		bsf	flag_4,time_2

		decfsz	cntr20,f		;7セグ表示用Counter
		goto	$+4
		movlw	d'10'
		movwf	cntr20
		bsf	flag_4,time_3
			
		decfsz	cntr50,f		;EEPROM 書き込み時間遅延Counter
		goto	$+4
		movlw	d'25'
		movwf	cntr50
		bsf	flag_4,time_4

Pop

		clrf	STATUS			; コンテキスト復帰
		movf	pclathtemp,w
		movwf	PCLATH
		swapf   statustemp,w
		movwf	STATUS
		swapf   wtemp,f
		swapf   wtemp,w

		retfie

;**************************************************************************
;ポートB出力変換
Cnv_scntr
		movf	scntr,w
		addwf	PCL,f
		retlw	b'11111011'	;s0
		retlw	b'11110111'	;s1
		retlw	b'11101111'	;s2
		retlw	b'11011111'	;s3
		retlw	b'10111111'	;s4
		retlw	b'01111111'	;s5

;-----------------------------------------------
;７セグ表示データ変換
Cnv_7SEG
		addwf	PCL,f
		retlw	b'10000001'	;0
		retlw	b'11110011'	;1
		retlw	b'01001001'	;2
		retlw	b'01100001'	;3
		retlw	b'00110011'	;4
		retlw	b'00100101'	;5
		retlw	b'00000101'	;6
		retlw	b'10110001'	;7
		retlw	b'00000001'	;8
		retlw	b'00100001'	;9
		retlw	b'11111111'	;0FF
		retlw	b'10001101'	;C
		retlw	b'00010111'	;h
		retlw	b'00001101'	;E

;-----------------------------------------------
;７セグデータセット
Set_7SEG
		call	spbout
		clrf	PORTA


;        MOVLW   0X1F ;hnk
;        MOVWF  PORTA
        movlw 0x0FF
        movwf   PORTB
        bsf	PORTA,cs0 ;nk
        bCf	PORTA,cs0 

        movlw 0x0FF
        movwf   PORTB
        bsf	PORTA,cs1 
        bCf	PORTA,cs1

        movlw 0x0FF
        movwf   PORTB
        bsf	PORTA,cs2 
        bCf	PORTA,cs2


        movlw 0x0FF
        movwf   PORTB
        bsf	PORTA,cs4 
        bCf	PORTA,cs4

        movlw 0x0FF
        movwf   PORTB
        bsf	PORTA,cs3
        bCf	PORTA,cs3

        ;nk






		call	Cnv_scntr
		movwf	PORTB
		bsf	PORTA,cs5

		movlw	d'5'
		movwf	cscntr
		movf	scntr,w
		addlw	cs0_s0	
		movwf	FSR
		movf	scntr,w
		addwf	PCL,f			;dp表示のための場合分け
		goto	Set_7SEG_loop_1		;s0
		goto	Set_7SEG_loop_2		;s1
		goto	Set_7SEG_loop_1		;s2
		goto	Set_7SEG_loop_1		;s3
		goto	Set_7SEG_loop_3		;s4
		goto	Set_7SEG_loop_1		;s5

Set_7SEG_loop_1
		movf	INDF,w
		call	Cnv_7SEG
		movwf	PORTB
		movf	cscntr,w
		sublw	d'5'
		btfss	STATUS,Z
		goto	set_7seg_loop_1_2
set_7seg_loop_1_1
		btfsc	flag_5,dp_ON
		bcf	PORTB,0			;dp表示
		bsf	PORTA,cs0
		goto	set_7seg_loop_1_3
set_7seg_loop_1_2
		bcf	STATUS,C
		rlf	PORTA,f
set_7seg_loop_1_3
		movlw	d'6'
		addwf	FSR,f
		decfsz	cscntr,f
		goto	Set_7SEG_loop_1
		goto	Set_7SEG_end

Set_7SEG_loop_2
		movf	INDF,w
		call	Cnv_7SEG
		movwf	PORTB
		movf	cscntr,w
		addwf	PCL,f
		goto	Set_7SEG_end
		goto	set_7seg_loop_2_1	;cs4
		goto	set_7seg_loop_2_1	;cs3
		goto	set_7seg_loop_2_2	;cs2
		goto	set_7seg_loop_2_2	;cs1
		goto	set_7seg_loop_2_3	;cs0

set_7seg_loop_2_1
		btfsc	flag_5,disp_ten
		nop;bcf	PORTB,0			;dp表示 noe
		goto	set_7seg_loop_2_4

set_7seg_loop_2_2
		btfsc	flag_5,disp_ten
		goto	set_7seg_loop_2_2_1
		btfsc	flag_3,error_set
		goto	set_7seg_loop_2_2_1
		goto	set_7seg_loop_2_4
set_7seg_loop_2_2_1
		nop;bcf	PORTB,0			;dp表示 now
		goto	set_7seg_loop_2_4

set_7seg_loop_2_3
		movf	flag_3,w
		andlw	b'01110000'
		btfsc	STATUS,Z
		goto	set_7seg_loop_2_3_1
		bsf	PORTA,cs0
		goto	set_7seg_loop_2_5
set_7seg_loop_2_3_1
		btfsc	flag_5,dp_ON
		nop;bcf	PORTB,0			;dp表示 now
		bsf	PORTA,cs0
		goto	set_7seg_loop_2_5

set_7seg_loop_2_4
		bcf	STATUS,C
		rlf	PORTA,f
set_7seg_loop_2_5
		movlw	d'6'
		addwf	FSR,f
		decfsz	cscntr,f
		goto	Set_7SEG_loop_2
		goto	Set_7SEG_end

Set_7SEG_loop_3
		movf	INDF,w
		call	Cnv_7SEG
		movwf	PORTB
		movf	cscntr,w
		sublw	d'5'
		btfss	STATUS,Z
		goto	set_7seg_loop_3_3
set_7seg_loop_3_1
		movf	flag_3,w
		andlw	b'01110000'
		btfsc	STATUS,Z
		goto	set_7seg_loop_3_2
		bsf	PORTA,cs0
		goto	set_7seg_loop_3_4
set_7seg_loop_3_2
		btfsc	flag_5,dp_ON
		nop;bcf	PORTB,0			;dp表示  now
		bsf	PORTA,cs0
		goto	set_7seg_loop_3_4
set_7seg_loop_3_3
		bcf	STATUS,C
		rlf	PORTA,f
set_7seg_loop_3_4
		movlw	d'6'
		addwf	FSR,f
		decfsz	cscntr,f
		goto	Set_7SEG_loop_3
		goto	Set_7SEG_end

Set_7SEG_end
		call	spbin
  
		return

;-----------------------------------------------
;７セグ表示とスイッチのチェック
Disp_7SEG
		bcf	flag_4,time_3

		call	Set_7SEG		;７セグデータセット

		btfsc	flag_5,dpb_short	;デシマルブリンク中はSwitch無視
		goto	Disp_7SEGend
		btfsc	flag,_mod_save		;データ保存中はSwitch無視
		goto	Disp_7SEGend


;---------------
;スイッチのチェック
Switch_Check
		movf	scntr,w
		addwf	PCL,f
		goto	Check_s0
		goto	Check_s1
		goto	Check_s2
		goto	Disp_7SEGend
		goto	Disp_7SEGend
		goto	Disp_7SEGend

Check_s0					;S0の場合
		btfss	PORTB,0
		goto	$+5
		bsf	PCLATH,3
		call	Push_ten_set_1
		bcf	PCLATH,3
		goto	Disp_7SEGend
		btfss	PORTB,1
		goto	$+5
		bsf	PCLATH,3
		call	Push_len_set_1
		bcf	PCLATH,3
		goto	Disp_7SEGend
		btfss	PORTB,2
		goto	$+5
		bsf	PCLATH,3
		call	Push_ten_set_2
		bcf	PCLATH,3
		goto	Disp_7SEGend
		btfss	PORTB,3
		goto	$+5
		bsf	PCLATH,3
		call	Push_len_set_2
		bcf	PCLATH,3
		goto	Disp_7SEGend
		btfss	PORTB,4
		goto	$+5
		bsf	PCLATH,3
		call	Push_check
		bcf	PCLATH,3
		goto	Disp_7SEGend
		bcf	flag_2,_s0_ON
		goto	Disp_7SEGend
Check_s1					;s1の場合
		btfss	PORTB,0
		goto	$+5
		bsf	PCLATH,3
		call	Push_reset
		bcf	PCLATH,3
		goto	Disp_7SEGend
		btfss	PORTB,1
		goto	$+5
		bsf	PCLATH,3
		call	Push_send
		bcf	PCLATH,3
		goto	Disp_7SEGend
		btfss	PORTB,3
		goto	$+5
		bsf	PCLATH,3
		call	Push_sensadj
		bcf	PCLATH,3
		goto	Disp_7SEGend
		btfss	PORTB,4
		goto	$+5
		bsf	PCLATH,3
		call	Push_enter
		bcf	PCLATH,3
		goto	Disp_7SEGend
		btfss	PORTB,5
		goto	$+5
		bsf	PCLATH,3
		call	Push_num_3
		bcf	PCLATH,3
		goto	Disp_7SEGend
		btfss	PORTB,6
		goto	$+5
		bsf	PCLATH,3
		call	Push_num_6
		bcf	PCLATH,3
		goto	Disp_7SEGend
		btfss	PORTB,7
		goto	$+5
		bsf	PCLATH,3
		call	Push_num_9
		bcf	PCLATH,3
		goto	Disp_7SEGend
		bcf	flag_2,_s1_ON
		movlw	b'01000111'
		andwf	flag_2,f
		goto	Disp_7SEGend

Check_s2					;s2の場合
		btfss	PORTB,0
		goto	$+5
		bsf	PCLATH,3
		call	Push_num_0
		bcf	PCLATH,3
		goto	Disp_7SEGend
		btfss	PORTB,1
		goto	$+5
		bsf	PCLATH,3
		call	Push_num_1
		bcf	PCLATH,3
		goto	Disp_7SEGend
		btfss	PORTB,2
		goto	$+5
		bsf	PCLATH,3
		call	Push_num_4
		bcf	PCLATH,3
		goto	Disp_7SEGend
		btfss	PORTB,3
		goto	$+5
		bsf	PCLATH,3
		call	Push_num_7
		bcf	PCLATH,3
		goto	Disp_7SEGend
		btfss	PORTB,4
		goto	$+5
		bsf	PCLATH,3
		call	Push_clear
		bcf	PCLATH,3
		goto	Disp_7SEGend
		btfss	PORTB,5
		goto	$+5
		bsf	PCLATH,3
		call	Push_num_2
		bcf	PCLATH,3
		goto	Disp_7SEGend
		btfss	PORTB,6
		goto	$+5
		bsf	PCLATH,3
		call	Push_num_5
		bcf	PCLATH,3
		goto	Disp_7SEGend
		btfss	PORTB,7
		goto	$+5
		bsf	PCLATH,3
		call	Push_num_8
		bcf	PCLATH,3
		goto	Disp_7SEGend
		bcf	flag_2,_s2_ON
		movlw	b'10111111'
		andwf	flag_2,f
		goto	Disp_7SEGend

Disp_7SEGend
		incf	scntr,f
		movf	scntr,w
		sublw	d'6'
		btfsc	STATUS,Z
		clrf	scntr

		movf	flag_3,w
		andlw	b'00001111'
		btfsc	STATUS,Z
		goto	$+3
		btfss	flag,_mod_save
		call	Blink

		btfsc	flag_5,dpb_short
		call	Dp_blink

		return

;-----------------------------------------------
;110bpsデータの読み込み(ch1)
Data_110bps_1
		bcf	flag_4,time_1_1

		btfss	flag_5,edge_1
		goto	Data_110bps_1end

data_read_1_1
		btfss	PORTC,rx_ch1
		goto	$+3
		bsf	STATUS,C
		goto	$+2
		bcf	STATUS,C
		rrf	windlen1_3,f
		incf	rxbit_cntr_1,f
		movf	rxbit_cntr_1,w
		sublw	d'8'
		btfss	STATUS,Z
		goto	Data_110bps_1end
		bcf	STATUS,C
		rrf	windlen1_3,f
		movf	windlen1_3,w
		sublw	d'127'
		btfss	STATUS,Z
		goto	data_read_1_2
		clrf	rxbit_cntr_1
		clrf	rxbyte_cntr_1
		bcf	flag_5,edge_1
		goto	Data_110bps_1end
data_read_1_2
		clrf	rxbit_cntr_1
		movlw	windlen1_0
		addwf	rxbyte_cntr_1,w
		movwf	FSR
		movf	windlen1_3,w
		movwf	INDF
		incf	rxbyte_cntr_1,f
		movf	rxbyte_cntr_1,w
		sublw	d'5'
		btfss	STATUS,Z
		goto	$+5
		bsf	flag_6,update_1
		call	Update_len
		bcf	flag_6,update_1
		clrf	rxbyte_cntr_1
		bcf	flag_5,edge_1
	
Data_110bps_1end
 

len1_disp
             
		bcf	STATUS,RP0         
                movf    flag_3,w           
                andlw   0x0f               
                btfss   STATUS,Z           
                goto    no_about_len1_disp 
            


		bsf	STATUS,RP0
;windlen1_2
                movf    windlen1_2,w  

                xorwf   len1_2_temp,w ; 
                movwf   length_change   
                xorwf   len1_2_temp,f       
 
                movf    length_change,w
                btfss   STATUS,Z     
                goto    length1_change_do

;windlen2_1
                movf    windlen1_1,w  

                xorwf   len1_1_temp,w ; 
                movwf   length_change    
                xorwf   len1_1_temp,f         
 
                movf    length_change,w
                btfss   STATUS,Z      
                goto    length1_change_do

;windlen2_0
                movf    windlen1_0,w  

                xorwf   len1_temp,w 
                movwf   length_change    
                xorwf   len1_temp,f        
 
                movf    length_change,w
                btfsc   STATUS,Z      
                goto    no_about_len1_disp


length1_change_do
		bcf	STATUS,RP0

		bcf	flag,set_ch
		bsf	flag_3,ten_set
                bsf     PCLATH,3
		call	Disp_set
                bCf     PCLATH,3
		bcf	STATUS,RP0 
no_about_len1_disp


	 	bcf	STATUS,RP0 

		return

;-----------------------------------------------
;110bpsデータの読み込み(ch2)
Data_110bps_2
		bcf	flag_4,time_1_2

		btfss	flag_5,edge_2
		goto	Data_110bps_2end

data_read_2_1
		btfss	PORTC,rx_ch2
		goto	$+3
		bsf	STATUS,C
		goto	$+2
		bcf	STATUS,C
		rrf	windlen2_3,f
		incf	rxbit_cntr_2,f
		movf	rxbit_cntr_2,w
		sublw	d'8'
		btfss	STATUS,Z
		goto	Data_110bps_2end
		bcf	STATUS,C
		rrf	windlen2_3,f
		movf	windlen2_3,w
		sublw	d'127'
		btfss	STATUS,Z
		goto	data_read_2_2
		clrf	rxbit_cntr_2
		clrf	rxbyte_cntr_2
		bcf	flag_5,edge_2
		goto	Data_110bps_2end
data_read_2_2
		clrf	rxbit_cntr_2
		movlw	windlen2_0
		addwf	rxbyte_cntr_2,w
		movwf	FSR
		movf	windlen2_3,w
		movwf	INDF
		incf	rxbyte_cntr_2,f
		movf	rxbyte_cntr_2,w
		sublw	d'5'
		btfss	STATUS,Z
		goto	$+5
		bsf	flag_6,update_2
		call	Update_len
		bcf	flag_6,update_2
		clrf	rxbyte_cntr_2
		bcf	flag_5,edge_2

Data_110bps_2end


len2_disp
             
		bcf	STATUS,RP0         
                movf    flag_3,w          
                andlw   0x0f               
                btfss   STATUS,Z           
                goto    no_about_len2_disp 
            


		bsf	STATUS,RP0
;windlen1_2
                movf    windlen2_2,w  

                xorwf   len2_2_temp,w 
                movwf   length_change   
                xorwf   len2_2_temp,f        
 
                movf    length_change,w
                btfss   STATUS,Z      
                goto    length2_change_do

;windlen2_1
                movf    windlen2_1,w  

                xorwf   len2_1_temp,w ; 
                movwf   length_change    
                xorwf   len2_1_temp,f       
 
                movf    length_change,w
                btfss   STATUS,Z      
                goto    length2_change_do

;windlen2_0

                movf    windlen2_0,w  

                xorwf   len2_temp,w 
                movwf   length_change    
                xorwf   len2_temp,f        
 
                movf    length_change,w
                btfsc   STATUS,Z      
                goto    no_about_len2_disp 

length2_change_do:

		bcf	STATUS,RP0
                bsf     flag_3,ten_set
 
		bcf	flag,set_ch
		bsf	flag_3,ten_set
                bsf     PCLATH,3
		call	Disp_set
                bCf     PCLATH,3

		bcf	STATUS,RP0 
no_about_len2_disp 
		bcf	STATUS,RP0 

		return


only_one_screen
fresh_time
               bsf	STATUS,RP0
               incf fresh_time_cnt
               ;movf fresh_time_cnt,w
               movlw  0x16
               subwf  fresh_time_cnt
               btfss  STATUS,C
               goto   no_about_one_screen
               clrf   fresh_time_cnt
              
no_about_fresh_time
               
               



            
		bcf	STATUS,RP0         
                movf    flag_3,w          
                andlw   0x0f               
                btfss   STATUS,Z           
                goto    no_about_one_screen 
             

one_screen_do
		bcf	STATUS,RP0

		bcf	flag,set_ch
		bsf	flag_3,ten_set
                bsf     PCLATH,3
		call	Disp_set
                bCf     PCLATH,3
		bcf	STATUS,RP0 
no_about_one_screen

	 	bcf	STATUS,RP0 



		return

;-----------------------------------------------
;110bpsデータの読み込み(ch3)
Data_110bps_3
		bcf	flag_4,time_1_3

		btfss	flag_5,edge_3
		goto	Data_110bps_3end

data_read_3_1
		btfss	PORTC,rx_ch3
		goto	$+3
		bsf	STATUS,C
		goto	$+2
		bcf	STATUS,C
		rrf	windlen3_3,f
		incf	rxbit_cntr_3,f
		movf	rxbit_cntr_3,w
		sublw	d'8'
		btfss	STATUS,Z
		goto	Data_110bps_3end
		bcf	STATUS,C
		rrf	windlen3_3,f
		movf	windlen3_3,w
		sublw	d'127'
		btfss	STATUS,Z
		goto	data_read_3_2
		clrf	rxbit_cntr_3
		clrf	rxbyte_cntr_3
		bcf	flag_5,edge_3
		goto	Data_110bps_3end
data_read_3_2
		clrf	rxbit_cntr_3
		movlw	windlen3_0
		addwf	rxbyte_cntr_3,w
		movwf	FSR
		movf	windlen3_3,w
		movwf	INDF
		incf	rxbyte_cntr_3,f
		movf	rxbyte_cntr_3,w
		sublw	d'5'
		btfss	STATUS,Z
		goto	$+5
		bsf	flag_6,update_3
		call	Update_len
		bcf	flag_6,update_3
		clrf	rxbyte_cntr_3
		bcf	flag_5,edge_3

Data_110bps_3end
		return

;-----------------------------------------------
;110bpsデータの読み込み(ch4)
Data_110bps_4
		bcf	flag_4,time_1_4

		btfss	flag_5,edge_4
		goto	Data_110bps_4end

data_read_4_1
		btfss	PORTC,rx_ch4
		goto	$+3
		bsf	STATUS,C
		goto	$+2
		bcf	STATUS,C
		rrf	windlen4_3,f
		incf	rxbit_cntr_4,f
		movf	rxbit_cntr_4,w
		sublw	d'8'
		btfss	STATUS,Z
		goto	Data_110bps_4end
		bcf	STATUS,C
		rrf	windlen4_3,f
		movf	windlen4_3,w
		sublw	d'127'
		btfss	STATUS,Z
		goto	data_read_4_2
		clrf	rxbit_cntr_4
		clrf	rxbyte_cntr_4
		bcf	flag_5,edge_4
		goto	Data_110bps_4end
data_read_4_2
		clrf	rxbit_cntr_4
		movlw	windlen4_0
		addwf	rxbyte_cntr_4,w
		movwf	FSR
		movf	windlen4_3,w
		movwf	INDF
		incf	rxbyte_cntr_4,f
		movf	rxbyte_cntr_4,w
		sublw	d'5'
		btfss	STATUS,Z
		goto	$+5
		bsf	flag_6,update_4
		call	Update_len
		bcf	flag_6,update_4
		clrf	rxbyte_cntr_4
		bcf	flag_5,edge_4

Data_110bps_4end
		return

;-----------------------------------------------
;300bps命令の送信
Send_300bps
		bcf	flag_4,time_2

		btfsc	flag,_mod_reset
		goto	send_reset
		btfsc	flag,_mod_send
		goto	send_send
		btfsc	flag,_mod_sensadj
		goto	send_sensadj
		btfsc	flag,_mod_check
		goto	send_sensadj
		btfsc	flag,_mod_setting
		goto	send_reset
		btfsc	flag,_mod_cont_const
		goto	send_cont_const
		goto	Send_300bpsend
;---------------
;RESET or SETTING　送信
send_reset
		bsf	flag_5,dp_ON
		movf	txbit_cntr,w
		btfss	STATUS,Z
		goto	send_reset_1
		bsf	PORTC,tx_ch1
		bsf	PORTC,tx_ch2
		goto	send_reset_4
send_reset_1					;0ビット目か？
		movf	txbit_cntr,w
		sublw	d'1'
		btfss	STATUS,Z
		goto	send_reset_3
send_reset_2
		movf	txbyte_cntr,w		;７バイト目以上か？
		sublw	d'5'
		btfss	STATUS,C
		goto	send_reset_2_3
send_reset_2_1					;１～６バイトデータセット
		movlw	lench1_0
		btfss	flag,_mod_reset
		movlw	stoptime_ch1
		addwf	txbyte_cntr,w
		movwf	FSR
		movf	INDF,w
		movwf	rreg_1
		movlw	d'10'
		addwf	FSR,f
		movf	INDF,w
		movwf	rreg_2
                movf   txbyte_cntr,w  
                sublw  0x03 ;      
                btfss  STATUS,Z      
                goto   send_reset_2_2
                bcf    STATUS,C  
                rlf    rreg_1    
                bcf    STATUS,C  
                rlf    rreg_2     
                movf   txbyte_cntr,w  
send_reset_2_2					;６バイト目データセット
		movf	txbyte_cntr,w
		sublw	d'5'
		btfss	STATUS,Z
		goto	send_reset_3
		movlw	b'11110001'
		btfss	flag,_mod_reset
		movlw	b'11110101'
		movwf	rreg_1
		movwf	rreg_2
		goto	send_reset_3
send_reset_2_3					;７～１１バイトデータセット
		movlw	lench1_0-6
		btfss	flag,_mod_reset
		movlw	stoptime_ch1-1
		addwf	txbyte_cntr,w
		movwf	FSR
		movf	INDF,w
		movwf	rreg_1
		movlw	d'10'
		addwf	FSR,f
		movf	INDF,w
		movwf	rreg_2
                movf   txbyte_cntr,w  
                sublw  0x09 ;     
                btfss  STATUS,Z      
                goto   send_reset_2_4;
                bcf    STATUS,C   
                rlf    rreg_1     
                bcf    STATUS,C  
                rlf    rreg_2     
 


send_reset_2_4					;１２バイト目データセット
		movf	txbyte_cntr,w
		sublw	d'11'
		btfss	STATUS,Z
		goto	send_reset_3
		movlw	b'11111001'
		btfss	flag,_mod_reset
		movlw	b'11111101'
		movwf	rreg_1
		movwf	rreg_2
send_reset_3					;９ビット目以上か？
		movf	txbit_cntr,w
		sublw	d'8'
		btfsc	STATUS,C
		goto	send_reset_3_1
		movf	txbit_cntr,w
		sublw	d'9'
		btfsc	STATUS,Z
		goto	send_reset_3_0
		goto	send_reset_3_0_2
send_reset_3_0					;9ビット目
		movf	txbyte_cntr,w
		sublw	d'5'
		btfsc	STATUS,Z
		goto	send_reset_3_0_2
		movf	txbyte_cntr,w
		sublw	d'11'
		btfsc	STATUS,Z
		goto	send_reset_3_0_2
send_reset_3_0_1
		bsf	PORTC,tx_ch1		;ビットクリア
		bsf	PORTC,tx_ch2
		goto	send_reset_4
send_reset_3_0_2
		bcf	PORTC,tx_ch1		;ビットセット
		bcf	PORTC,tx_ch2
		goto	send_reset_4
				
send_reset_3_1					;CH1データ送信
		rrf	rreg_1,f
		btfsc	STATUS,C
		goto	$+3
		bsf	PORTC,tx_ch1
		goto	send_reset_3_2
		bcf	PORTC,tx_ch1
send_reset_3_2					;CH2データ送信
		rrf	rreg_2,f
		btfsc	STATUS,C
		goto	$+3
		bsf	PORTC,tx_ch2
		goto	send_reset_4
		bcf	PORTC,tx_ch2
send_reset_4					;カウンターチェック
		incf	txbit_cntr,f
		movf	txbit_cntr,w
		sublw	d'13'
		btfss	STATUS,Z
		goto	Send_300bpsend
		clrf	txbit_cntr
		incf	txbyte_cntr,f
		movf	txbyte_cntr,w
		sublw	d'12'
		btfss	STATUS,Z
		goto	Send_300bpsend
		clrf	txbyte_cntr
		movlw	b'11000000'
		andwf	flag,f
		bcf	flag_5,dp_ON
		bsf	flag_5,dpb_short
		goto	Send_300bpsend

;---------------
;SEND送信
send_send
		movf	txbit_cntr,w
		btfss	STATUS,Z
		goto	send_send_1
		bsf	PORTC,tx_ch1
		bsf	PORTC,tx_ch2
		goto	send_send_4
send_send_1
		movf	txbit_cntr,w
		sublw	d'1'
		btfss	STATUS,Z
		goto	send_send_3
send_send_2
		btfsc	txbyte_cntr,0
		goto	send_send_2_3
send_send_2_1
		btfsc	txbyte_cntr,1
		goto	send_send_2_2
		bsf	STATUS,RP0
		movf	tens_ch1,w
		bcf	STATUS,RP0
		movwf	rreg_1
                bcf     STATUS,C   
                rlf     rreg_1   
		bsf	STATUS,RP0
		movf	tens_ch2,w
		bcf	STATUS,RP0
		movwf	rreg_2
                bcf    STATUS,C   
                rlf     rreg_2   
		goto	send_send_3
send_send_2_2
		bsf	STATUS,RP0
		movf	tens_ch3,w
		bcf	STATUS,RP0
 		movwf	rreg_1
                bcf    STATUS,C  
                rlf     rreg_1   
		bsf	STATUS,RP0
		movf	tens_ch4,w
		bcf	STATUS,RP0
		movwf	rreg_2
                bcf    STATUS,C  
                rlf     rreg_2   
		goto	send_send_3
send_send_2_3
		btfsc	txbyte_cntr,1
		goto	send_send_2_4
		movlw	b'11110011'
		movwf	rreg_1
		movwf	rreg_2
		goto	send_send_3
send_send_2_4
		movlw	b'11111011'
		movwf	rreg_1
		movwf	rreg_2
send_send_3
		movf	txbit_cntr,w
		sublw	d'8'
		btfsc	STATUS,C
		goto	send_send_3_1
		movf	txbit_cntr,w
		sublw	d'9'
		btfsc	STATUS,Z
		goto	send_send_3_0
		goto	send_send_3_0_2
send_send_3_0
		btfsc	txbyte_cntr,0
		goto	send_send_3_0_2
send_send_3_0_1
		bsf	PORTC,tx_ch1
		bsf	PORTC,tx_ch2
		goto	send_send_4
send_send_3_0_2
		bcf	PORTC,tx_ch1
		bcf	PORTC,tx_ch2
		goto	send_send_4
send_send_3_1
		rrf	rreg_1,f
		btfsc	STATUS,C
		goto	$+3
		bsf	PORTC,tx_ch1
		goto	send_send_3_2
		bcf	PORTC,tx_ch1
send_send_3_2
		rrf	rreg_2,f
		btfsc	STATUS,C
		goto	$+3
		bsf	PORTC,tx_ch2
		goto	send_send_4
		bcf	PORTC,tx_ch2
send_send_4
		incf	txbit_cntr,f
		movf	txbit_cntr,w
		sublw	d'13'
		btfss	STATUS,Z
		goto	Send_300bpsend
		clrf	txbit_cntr
		incf	txbyte_cntr,f
		movf	txbyte_cntr,w
		sublw	d'4'
		btfss	STATUS,Z
		goto	Send_300bpsend
		clrf	txbyte_cntr
		bcf	flag,_mod_send
		bsf	flag_5,dpb_short
		goto	Send_300bpsend

;---------------
;SENS.ADJ or BREAK送信
send_sensadj
		bsf	flag_5,dp_ON
		movf	txbit_cntr,w
		btfss	STATUS,Z
		goto	send_sensadj_1
		bsf	PORTC,tx_ch1
		bsf	PORTC,tx_ch2
		goto	send_sensadj_4
send_sensadj_1
		movf	txbit_cntr,w
		sublw	d'1'
		btfss	STATUS,Z
		goto	send_sensadj_3	
send_sensadj_2
		movlw	b'11110010'
		btfss	flag,_mod_sensadj
		movlw	b'11110100'
		movwf	rreg_1
		movwf	rreg_2
send_sensadj_3
		movf	txbit_cntr,w
		sublw	d'8'
		btfsc	STATUS,C
		goto	send_sensadj_3_1
		bcf	PORTC,tx_ch1
		bcf	PORTC,tx_ch2
		goto	send_sensadj_4
send_sensadj_3_1
		rrf	rreg_1,f
		btfsc	STATUS,C
		goto	$+3
		bsf	PORTC,tx_ch1
		goto	send_sensadj_3_2
		bcf	PORTC,tx_ch1
send_sensadj_3_2
		rrf	rreg_2,f
		btfsc	STATUS,C
		goto	$+3
		bsf	PORTC,tx_ch2
		goto	send_sensadj_4
		bcf	PORTC,tx_ch2
send_sensadj_4
		incf	txbit_cntr,f
		movf	txbit_cntr,w
		sublw	d'11'
		btfss	STATUS,Z
		goto	Send_300bpsend
		clrf	txbit_cntr
		movlw	b'11000000'
		andwf	flag,f
		bcf	flag_5,dp_ON
		bsf	flag_5,dpb_short
		goto	Send_300bpsend

;---------------
;制御定数送信
send_cont_const
		bsf	flag_5,dp_ON
		movf	txbit_cntr,w
		btfss	STATUS,Z
		goto	send_cont_const_1
		bsf	PORTC,tx_ch1
		bsf	PORTC,tx_ch2
		goto	send_cont_const_4
send_cont_const_1
		movf	txbit_cntr,w
		sublw	d'1'
		btfss	STATUS,Z
		goto	send_cont_const_3
send_cont_const_2
		movf	txbyte_cntr,w
		sublw	d'2'
		btfss	STATUS,C
		goto	send_cont_const_2_3
send_cont_const_2_1
		movf	txbyte_cntr,w
		addlw	const_kp_ch1
		movwf	FSR
		movf	INDF,w
		movwf	rreg_1
		movlw	d'10'
		addwf	FSR,f
		movf	INDF,w
		movwf	rreg_2
send_cont_const_2_2
		movf	txbyte_cntr,w
		sublw	d'2'
		btfss	STATUS,Z
		goto	send_cont_const_3
		movlw	b'11110110'
		movwf	rreg_1
		movwf	rreg_2
		goto	send_cont_const_3
send_cont_const_2_3
		movf	txbyte_cntr,w
		addlw	const_kp_ch3-3
		movwf	FSR
		movf	INDF,w
		movwf	rreg_1
		movlw	d'10'
		addwf	FSR,f
		movf	INDF,w
		movwf	rreg_2
send_cont_const_2_4
		movf	txbyte_cntr,w
		sublw	d'5'
		btfss	STATUS,Z
		goto	send_cont_const_3
		movlw	b'11111110'
		movwf	rreg_1
		movwf	rreg_2
send_cont_const_3
		movf	txbit_cntr,w
		sublw	d'8'
		btfsc	STATUS,C
		goto	send_cont_const_3_1
		movf	txbit_cntr,w
		sublw	d'9'
		btfsc	STATUS,Z
		goto	send_cont_const_3_0
		goto	send_cont_const_3_0_2
send_cont_const_3_0
		movf	txbyte_cntr,w
		sublw	d'2'
		btfsc	STATUS,Z
		goto	send_cont_const_3_0_2
		movf	txbyte_cntr,w
		sublw	d'5'
		btfsc	STATUS,Z
		goto	send_cont_const_3_0_2
send_cont_const_3_0_1
		bsf	PORTC,tx_ch1
		bsf	PORTC,tx_ch2
		goto	send_cont_const_4
send_cont_const_3_0_2
		bcf	PORTC,tx_ch1
		bcf	PORTC,tx_ch2
		goto	send_cont_const_4
send_cont_const_3_1
		rrf	rreg_1,f
		btfsc	STATUS,C
		goto	$+3
		bsf	PORTC,tx_ch1
		goto	send_cont_const_3_2
		bcf	PORTC,tx_ch1
send_cont_const_3_2
		rrf	rreg_2,f
		btfsc	STATUS,C
		goto	$+3
		bsf	PORTC,tx_ch2
		goto	send_cont_const_4
		bcf	PORTC,tx_ch2
send_cont_const_4
		incf	txbit_cntr,f
		movf	txbit_cntr,w
		sublw	d'12'
		btfss	STATUS,Z
		goto	Send_300bpsend
		clrf	txbit_cntr
		incf	txbyte_cntr,f
		movf	txbyte_cntr,w
		sublw	d'6'
		btfss	STATUS,Z
		goto	Send_300bpsend
		clrf	txbyte_cntr
		bcf	flag,_mod_cont_const
		bcf	flag_5,dp_ON
		bsf	flag_5,dpb_short

Send_300bpsend
		return

;-----------------------------------------------
;データ保存(EEPROM)
Save_Data
		bcf	flag_4,time_4

		btfss	flag,_mod_save
		goto	Save_Dataend

		btfsc	flag_3,ten_set
		goto	save_data_1
		btfsc	flag_3,error_set
		goto	save_data_2
		btfsc	flag_3,stop_set
		goto	save_data_3
		btfsc	flag_3,const_set
		goto	save_data_4

save_data_1
		movf	flag_3,w
		andlw	b'00000011'
		btfsc	STATUS,Z
		goto	save_data_1_2
save_data_1_1
		movlw	d'3'
		movwf	wraddrl
		movlw	d'10'
		btfsc	flag,set_ch
		addwf	wraddrl,f
		movlw	d'20'	
		btfss	flag_3,ten_1
		addwf	wraddrl,f
		movlw	tens_ch1
		movwf	FSR
		movlw	d'5'
		btfsc	flag,set_ch
		addwf	FSR,f
		movlw	d'10'
		btfss	flag_3,ten_1
		addwf	FSR,f
		movf	INDF,w
		movwf	eedt
		call	ByteWrite
		goto	save_data_5
save_data_1_2
		clrf	wraddrl
		movlw	d'10'	
		btfsc	flag,set_ch
		addwf	wraddrl,f
		movlw	d'20'	
		btfss	flag_3,len_1
		addwf	wraddrl,f
		movf	savebyte_cntr,w
		addwf	wraddrl,f
		addlw	lench1_0
		movwf	FSR
		movlw	d'5'
		btfsc	flag,set_ch
		addwf	FSR,f
		movlw	d'10'
		btfss	flag_3,len_1
		addwf	FSR,f
		movf	INDF,w
		movwf	eedt
		call	ByteWrite
		incf	savebyte_cntr,f
		movf	savebyte_cntr,w
		sublw	d'3'
		btfss	STATUS,Z
		goto	Save_Dataend
		goto	save_data_5

save_data_2
		movlw	d'6'
		movwf	wraddrl
		movlw	d'10'
		btfsc	flag,set_ch
		addwf	wraddrl,f
		movf	flag_3,w
		andlw	b'00000101'
		movlw	d'20'	
		btfsc	STATUS,Z
		addwf	wraddrl,f
		movf	flag_3,w
		andlw	b'00000011'
		btfsc	STATUS,Z
		incf	wraddrl,f
		movlw	errortens_ch1
		movwf	FSR
		movlw	d'5'
		btfsc	flag,set_ch
		addwf	FSR,f
		movf	flag_3,w
		andlw	b'00000101'
		movlw	d'10'
		btfsc	STATUS,Z
		addwf	FSR,f
		movf	flag_3,w
		andlw	b'00000011'
		btfsc	STATUS,Z
		incf	FSR,f
		movf	INDF,w
		movwf	eedt
		call	ByteWrite
		goto	save_data_5

save_data_3
		movlw	d'4'
		movwf	wraddrl
		movlw	d'10'
		btfsc	flag,set_ch
		addwf	wraddrl,f
		movf	flag_3,w
		andlw	b'00000101'
		movlw	d'20'	
		btfsc	STATUS,Z
		addwf	wraddrl,f
		movf	flag_3,w
		andlw	b'00000011'
		btfsc	STATUS,Z
		incf	wraddrl,f
		movlw	stoptime_ch1
		movwf	FSR
		movlw	d'5'
		btfsc	flag,set_ch
		addwf	FSR,f
		movf	flag_3,w
		andlw	b'00000101'
		movlw	d'10'
		btfsc	STATUS,Z
		addwf	FSR,f
		movf	flag_3,w
		andlw	b'00000011'
		btfsc	STATUS,Z
		incf	FSR,f
		movf	INDF,w
		movwf	eedt
		call	ByteWrite
		goto	save_data_5

save_data_4
		movlw	d'8'
		movwf	wraddrl
		movlw	d'10'
		btfsc	flag,set_ch
		addwf	wraddrl,f
		movf	flag_3,w
		andlw	b'00000101'
		movlw	d'20'	
		btfsc	STATUS,Z
		addwf	wraddrl,f
		movf	flag_3,w
		andlw	b'00000011'
		btfsc	STATUS,Z
		incf	wraddrl,f
		movlw	const_kp_ch1
		movwf	FSR
		movlw	d'5'
		btfsc	flag,set_ch
		addwf	FSR,f
		movf	flag_3,w
		andlw	b'00000101'
		movlw	d'10'	
		btfsc	STATUS,Z
		addwf	FSR,f
		movf	flag_3,w
		andlw	b'00000011'
		btfsc	STATUS,Z
		incf	FSR,f
		movf	INDF,w
		movwf	eedt
		call	ByteWrite
		goto	save_data_5

save_data_5
		clrf	savebyte_cntr
		bcf	flag,_mod_save
		movlw	b'11110000'
		andwf	flag_3,f

Save_Dataend
		return
		
;-----------------------------------------------
;設定中の数字のブリンク
Blink
		movf	flag_3,w
		andlw	b'00001111'
		btfsc	STATUS,Z
		goto	Blinkend

		incf	blink_cntr_2,f
		movf	blink_cntr_2,w
		sublw	d'100'
		btfss	STATUS,Z
		goto	Blinkend
		clrf	blink_cntr_2

		btfss	flag_3,ten_1
		goto	$+3
		movlw	cs0_s3
		goto	blink_1
		btfss	flag_3,ten_2
		goto	$+3
		movlw	cs0_s0
		goto	blink_1
		btfss	flag_3,len_1
		goto	$+3
		movlw	cs2_s0
		goto	blink_1
		movlw	cs1_s0
blink_1
		movwf	FSR
		movf	INDF,w
		sublw	d'10'
		btfsc	STATUS,Z
		goto	$+4
		movlw	d'10'
		movwf	INDF
		goto	Blinkend
		movf	blink_reg,w
		movwf	INDF
		goto	Blinkend

Blinkend
		return

;-----------------------------------------------
;データ送信中のデシマルのブリンク
Dp_blink
		bsf	flag_5,dp_ON
		incf	blink_cntr,f
		movf	blink_cntr,w
		sublw	d'250'
		btfss	STATUS,Z
		goto	Dp_blinkend
		clrf	blink_cntr
		bcf	flag_5,dp_ON
		bcf	flag_5,dpb_short
Dp_blinkend
		return

;-----------------------------------------------
;ポートＢを出力に設定
spbout
		bsf	STATUS,RP0
		clrf	TRISB
		bcf	STATUS,RP0
		return

;-----------------------------------------------
;ポートＢを入力に設定
spbin
		bsf	STATUS,RP0
		movlw	h'FF'
		movwf	TRISB
		bcf	STATUS,RP0
		return

;-----------------------------------------------
;110bpsデータのエッジがきたか？
Edge_check
		btfsc	flag_5,edge_1
		goto	edge_check_2
edge_check_1_1
		btfsc	PORTC,rx_ch1
		goto	edge_check_1_2
		btfss	windlen1_3,7
		goto	edge_check_2
		movlw	d'9'
		movwf	cntr34_1
		bcf	flag_4,time_1_1
		bsf	flag_5,edge_1
		goto	edge_check_2
edge_check_1_2
		bsf	windlen1_3,7

edge_check_2
		btfsc	flag_5,edge_2
		goto	edge_check_3
edge_check_2_1
		btfsc	PORTC,rx_ch2
		goto	edge_check_2_2
		btfss	windlen2_3,7
		goto	edge_check_3
		movlw	d'9'
		movwf	cntr34_2
		bcf	flag_4,time_1_2
		bsf	flag_5,edge_2
		goto	edge_check_3
edge_check_2_2
		bsf	windlen2_3,7

edge_check_3
		btfsc	flag_5,edge_3
		goto	edge_check_4
edge_check_3_1
		btfsc	PORTC,rx_ch3
		goto	edge_check_3_2
		btfss	windlen3_3,7
		goto	edge_check_4
		movlw	d'9'
		movwf	cntr34_3
		bcf	flag_4,time_1_3
		bsf	flag_5,edge_3
		goto	edge_check_4
edge_check_3_2
		bsf	windlen3_3,7

edge_check_4
		btfsc	flag_5,edge_4
		goto	Edge_checkend
edge_check_4_1
		btfsc	PORTC,rx_ch4
		goto	edge_check_4_2
		btfss	windlen4_3,7
		goto	Edge_checkend
		movlw	d'9'
		movwf	cntr34_4
		bcf	flag_4,time_1_4
		bsf	flag_5,edge_4
		goto	Edge_checkend
edge_check_4_2
		bsf	windlen4_3,7

Edge_checkend
		return

;-----------------------------------------------
;巻き取り糸長の表示の更新
Update_len
		btfss	flag_5,disp_len
		goto	Update_ten

update_len_1_1
		btfss	flag_6,update_1
		goto	update_len_1_2
		movlw	windlen1_0
		movwf	fsrtemp_2
		movlw	cs2_s0
		movwf	fsrtemp_1
		goto	update_len_2
update_len_1_2
		btfss	flag_6,update_2
		goto	update_len_1_3
		movlw	windlen2_0
		movwf	fsrtemp_2
		movlw	cs1_s0
		movwf	fsrtemp_1
		goto	update_len_2
update_len_1_3
		btfss	flag_6,update_3
		goto	update_len_1_4
		movlw	windlen3_0
		movwf	fsrtemp_2
		movlw	cs4_s0
		movwf	fsrtemp_1
		goto	update_len_2
update_len_1_4
		btfss	flag_6,update_4
		goto	Update_ten_end
		movlw	windlen4_0
		movwf	fsrtemp_2
		movlw	cs3_s0
		movwf	fsrtemp_1
update_len_2
		movf	fsrtemp_2,w
		movwf	FSR
		movf	INDF,w
		bsf	PCLATH,3
		call	BINtoASC
		bcf	PCLATH,3
		incf	fsrtemp_2,f
		movf	fsrtemp_1,w
		movwf	FSR
		movf	_tmp_1,w
		movwf	INDF
		incf	FSR,f
		movf	_tmp_2,w
		movwf	INDF
		movlw	d'2'
		addwf	fsrtemp_1,f
		movf	fsrtemp_2,w
		movwf	FSR
		movf	INDF,w
		bsf	PCLATH,3
		call	BINtoASC
		bcf	PCLATH,3
		incf	fsrtemp_2,f
		movf	fsrtemp_1,w
		movwf	FSR
		movf	_tmp_1,w
		movwf	INDF
		incf	FSR,f
		movf	_tmp_2,w
		movwf	INDF
		movlw	d'2'
		addwf	fsrtemp_1,f
		movf	fsrtemp_2,w
		movwf	FSR
		movf	INDF,w
		bsf	PCLATH,3
		call	BINtoASC
		bcf	PCLATH,3
		movf	fsrtemp_1,w
		movwf	FSR
		movf	_tmp_1,w
		movwf	INDF
		incf	FSR,f
		movf	_tmp_2,w
		movwf	INDF
		movf	FSR,w
		bsf	PCLATH,3
		call	Zero_check_2
		bcf	PCLATH,3

Update_ten
		btfss	flag_5,disp_ten
		goto	Update_ten_end

update_ten_1_1
		btfss	flag_6,update_1
		goto	update_ten_1_2
		movlw	realtens1_0
		movwf	fsrtemp_2
		movlw	cs2_s0
		movwf	fsrtemp_1
		goto	update_ten_2
update_ten_1_2
		btfss	flag_6,update_2
		goto	update_ten_1_3
		movlw	realtens2_0
		movwf	fsrtemp_2
		movlw	cs1_s0
		movwf	fsrtemp_1
		goto	update_ten_2
update_ten_1_3
		btfss	flag_6,update_3
		goto	update_ten_1_4
		movlw	realtens3_0
		movwf	fsrtemp_2
		movlw	cs4_s0
		movwf	fsrtemp_1
		goto	update_ten_2
update_ten_1_4
		btfss	flag_6,update_4
		goto	Update_ten_end
		movlw	realtens4_0
		movwf	fsrtemp_2
		movlw	cs3_s0
		movwf	fsrtemp_1
update_ten_2
		movf	fsrtemp_2,w
		movwf	FSR
		movf	INDF,w
		bsf	PCLATH,3
		call	BINtoASC
		bcf	PCLATH,3
		movf	fsrtemp_1,w
		movwf	FSR
		movf	_tmp_1,w
		movwf	INDF
		incf	fsrtemp_1,f
		incf	fsrtemp_2,w
		movwf	FSR
		movf	INDF,w
		bsf	PCLATH,3
		call	BINtoASC
		bcf	PCLATH,3
		movf	fsrtemp_1,w
		movwf	FSR
		movf	_tmp_1,w
		movwf	INDF
		incf	FSR,f
		movf	_tmp_2,w
		movwf	INDF
		decf	fsrtemp_1,f
		movf	FSR,w
		bsf	PCLATH,3
		call	Zero_check_1
		bcf	PCLATH,3
Update_ten_end
		return

;--------------------------------
;I2Cファームウエアマスターモード用サブルーチン(24LC64用)
ByteRead					; バイトリード

		call	STARTcdtn		; スタートコンディション送信

		;------------------------
		movlw	WRITE			; ライト用コントロールバイト送信
		call	txBYTE

		;------------------------
		movlw	h'00'			; リード用アドレスHIGHバイト送信
		call	txBYTE

		;------------------------
		movf	rdaddrl,w		; リード用アドレスLOWバイト送信
		call	txBYTE

		;------------------------
		call	RepeatedSTARTcdtn	; 繰り返しスタートコンディション送信

		;------------------------
		movlw	READ			; リード用コントロールバイト送信
		call	txBYTE

		;------------------------
		call	rxLastBYTE		; データ受信(最後のバイト)
		movwf	eedt

		;------------------------
		call	STOPcdtn		; ストップコンディション
		return

;--------------------------------
ByteWrite
		call	STARTcdtn		; スタートコンディション

		;------------------------
		movlw	WRITE			; ライト用コントロールバイト送信
		call	txBYTE

		;------------------------
		movlw	h'00'			; ライト用アドレスHIGHバイト送信
		call	txBYTE

		;------------------------
		movf	wraddrl,w		; ライト用アドレスLOWバイト送信
		call	txBYTE

		;------------------------
		movf	eedt,w			; データ送信
		call	txBYTE

		;------------------------
		call	STOPcdtn		; ストップコンディション送信
		return

;--------------------------------
STARTcdtn					; スタートコンディション送信
		bsf	STATUS,RP0
		bcf	TRISC,SDA		; SDA=0
		goto	$+1
		bcf	TRISC,SCL		; SCL=0
		bcf	STATUS,RP0

		return
;--------------------------------
RepeatedSTARTcdtn				; 繰り返しスタートコンディション送信
		bsf	STATUS,RP0
		bsf	TRISC,SDA		; SDA=1
		goto	$+1
		bsf	TRISC,SCL		; SCL=1
		goto	$+1
		bcf	TRISC,SDA		; SDA=0
		goto	$+1
		bcf	TRISC,SCL		; SCL=0
		bcf	STATUS,RP0

		return

;--------------------------------
txBYTE						; データバイト送信

		movwf	i2cbyte

		movlw	d'8'
		movwf	bitcnt

txBYTEloop

		rlf	i2cbyte,f

		bsf	STATUS,RP0
		bsf	TRISC,SDA		; SDA=1
		btfss	STATUS,C
		bcf	TRISC,SDA		; SDA=0
		bsf	TRISC,SCL		; SCL=1
		goto	$+1
		bcf	TRISC,SCL		; SCL=0
		bcf	STATUS,RP0

		decfsz	bitcnt,f
		goto	txBYTEloop

		bsf	STATUS,RP0
		bsf	TRISC,SDA		; SDA=1
		bsf	TRISC,SCL		; SCL=1
		goto	$+1
		bcf	TRISC,SCL		; SCL=0
		bcf	STATUS,RP0

		return

;--------------------------------
rxLastBYTE					; データバイト受信

		movlw	d'8'
		movwf	bitcnt

rxLastBYTEloop
		bsf	STATUS,RP0
		bsf	TRISC,SDA		; SDA=1
		bsf	TRISC,SCL		; SCL=1
		bcf	STATUS,RP0

		bsf	STATUS,C
		btfss	PORTC,SDA
		bcf	STATUS,C
		rlf	i2cbyte,f


		bsf	STATUS,RP0
		bcf	TRISC,SCL		; SCL=0
		bcf	STATUS,RP0

		decfsz	bitcnt,f
		goto	rxLastBYTEloop

		bsf	STATUS,RP0
		bsf	TRISC,SDA		; SDA=1
		bsf	TRISC,SCL		; SCL=1
		goto	$+1
		bcf	TRISC,SCL		; SCL=0
		bcf	STATUS,RP0

		bsf	STATUS,RP0
		bsf	TRISC,SDA		; SDA=1
		bcf	STATUS,RP0

		movf	i2cbyte,w		; w=i2cbyte

		return

;--------------------------------
STOPcdtn					; ストップコンディション送信

		bsf	STATUS,RP0
		bcf	TRISC,SDA		; SDA=0
		goto	$+1
		bsf	TRISC,SCL		; SCL=1
		goto	$+1
		bsf	TRISC,SDA		; SDA=1
		bcf	STATUS,RP0

		return

;**************************************************************************
;初期化
start
						;ポートの設定
		clrf	PORTA
		clrf	PORTB
		clrf	PORTC
		clrf	INTCON
		bsf	STATUS,RP0
		movlw	b'10000000'		; RBPU=1, PS2:PS0=000
		movwf	OPTION_REG
		clrf	TRISA
		movlw	h'ff'
		movwf	TRISB
		movlw	b'11111100'
		movwf	TRISC
		movlw	b'00100000'
		movwf	INTCON
		bcf	STATUS,RP0

		movlw	b'00111011'
		movwf	SSPCON
		movlw	b'00110001'		;タイマー１：1/8
		movwf	T1CON

;初期設定	
		movlw	h'20'		;Clear Memory (Bank 0) 
		movwf	FSR
		clrf	INDF
		incf	FSR,f
		btfss	FSR,7
		goto	$-3

		movlw	h'a0'		;Clear Memory (Bank 1)
		movwf	FSR
		clrf	INDF
		incfsz	FSR,f
		goto	$-2

		clrf	rdaddrl
		movlw	d'4'		;ch1設定値の読み込み
		movwf	cntr
		clrf	cntr_1
		movlw	lench1_0
		movwf	fsrtemp_1
		movwf	FSR
init_loop_1
		call	ByteRead
		movf	eedt,w
		movwf	INDF
		incf	FSR,f
		incf	rdaddrl,f
		decfsz	cntr,f
		goto	$-6
		movlw	d'4'
		movwf	cntr
		movlw	d'16'
		addwf	FSR,f
		call	ByteRead
		movf	eedt,w
		movwf	INDF
		incf	FSR,f
		incf	rdaddrl,f
		decfsz	cntr,f
		goto	$-6
		movlw	d'2'
		movwf	cntr
		movlw	d'16'
		addwf	FSR,f
		call	ByteRead
		movf	eedt,w
		movwf	INDF
		incf	FSR,f
		incf	rdaddrl,f
		decfsz	cntr,f
		goto	$-6
		incf	cntr_1,f
		movlw	d'5'
		addwf	fsrtemp_1,f
		movf	fsrtemp_1,w
		movwf	FSR
		movlw	d'4'
		movwf	cntr
		btfss	cntr_1,2
		goto	init_loop_1
init_loop_end		
		clrf	cntr
		clrf	cntr_1

		bsf	PCLATH,3
		call	Sum_checksum_1
		call	Sum_checksum_2
		bcf	PCLATH,3

		bsf	flag_3,ten_set
		bsf	PCLATH,3
		call	Disp_set
		bcf	PCLATH,3

		movlw	d'17'		;初期値設定
		movwf	cntr34_1
		movwf	cntr34_2
		movwf	cntr34_3
		movwf	cntr34_4
		movlw	d'17'
		movwf	cntr34
		movlw	d'10'
		movwf	cntr20
		movlw	d'25'
		movwf	cntr50
		movlw	d'250'
		movwf	cntr500

		clrf	TMR0
		bcf	INTCON,T0IF
		bsf	INTCON,GIE

;you wei 73 wu wei 63
;added by hnk
                BSF     STATUS,RP0
                MOVLW  	b'00000111'   ;added by hnk
                MOVWF   ADCON1        ;added by hnk
                BCF     ADCON0,0     ;added by hnk
                BCF     STATUS,RP0

;added by hnk
;**************************************************************************
mainloop



		btfsc	flag_4,time_1_1		;3.4ms毎
		call	Data_110bps_1

		btfsc	flag_4,time_1_2		;3.4ms毎
		call	Data_110bps_2
		btfsc	flag_4,time_1_3		;3.4ms毎
		call	Data_110bps_3

		btfsc	flag_4,time_1_4		;3.4ms毎
		call	Data_110bps_4

		btfsc	flag_4,time_2		;3.4ms毎
		call	Send_300bps

		btfsc	flag_4,time_3		;2ms毎
		call	Disp_7SEG

		btfsc	flag_4,time_4		;5ms毎
		call	Save_Data

		call	Edge_check

		goto	mainloop

;**************************************************************************

		org	h'0800'

;-----------------------------------------------
;通常モードの表示
Disp_set
		movf	flag_5,w
		andlw	b'11000000'
		btfss	STATUS,Z
		goto	disp_set_1

		clrf	fsrtemp_2
		movlw	d'5'
		btfsc	flag,set_ch
		movwf	fsrtemp_2
		movlw	cs0_s0
		movwf	fsrtemp_1
		btfss	flag_3,ten_set
		goto	$+4
		movlw	tens_ch2
		addwf	fsrtemp_2,f
		goto	disp_set_2
		btfss	flag_3,error_set
		goto	$+4
		movlw	errortens_ch2
		addwf	fsrtemp_2,f
		goto	disp_set_2
		btfss	flag_3,stop_set
		goto	$+4
		movlw	stoptime_ch2
		addwf	fsrtemp_2,f
		goto	disp_set_2
		movlw	const_kp_ch2
		addwf	fsrtemp_2,f
		goto	disp_set_2
disp_set_1
		movlw	d'10'
		movwf	cs0_s0
		movwf	cs0_s1
		movwf	cs0_s2
		movwf	cs0_s3
		movwf	cs0_s4
		movwf	cs0_s5
		goto	disp_set_3
disp_set_2
		call	Disp_3digit

disp_set_3
		movlw	cs1_s0
		movwf	fsrtemp_1
		clrf	fsrtemp_2
		movlw	d'5'
		btfsc	flag,set_ch
		movwf	fsrtemp_2
		btfss	flag_5,disp_ten		;張力表示？
		goto	$+4
		movlw	realtens2_0
		addwf	fsrtemp_2,f
		goto	disp_set_5
		btfss	flag_5,disp_len		;長さ表示？
		goto	$+4
		movlw	windlen2_0
		addwf	fsrtemp_2,f
		goto	disp_set_4
		btfss	flag_3,ten_set		;張力設定？
		goto	$+4
		movlw	lench2_0
		addwf	fsrtemp_2,f
		goto	disp_set_4
		btfss	flag_3,error_set	;エラー設定？
		goto	$+4
		movlw	errortime_ch2
		addwf	fsrtemp_2,f
		goto	disp_set_5
		btfss	flag_3,stop_set		;停止判定設定？
		goto	$+4
		movlw	rotate_ch2
		addwf	fsrtemp_2,f
		goto	disp_set_5
		movlw	const_ti_ch2		;制御定数設定
		addwf	fsrtemp_2,f
		goto	disp_set_5
disp_set_4
		call	Disp_6digit
		goto	disp_set_6
disp_set_5
		call	Disp_3digit

disp_set_6
		movlw	cs3_s0
		movwf	fsrtemp_1
		btfss	flag_5,disp_len		;長さ表示？
		goto	$+4
		movlw	windlen4_0
		movwf	fsrtemp_2
		goto	disp_set_8
		btfss	flag_5,disp_ten		;張力？
		goto	$+4
		movlw	realtens4_0
		movwf	fsrtemp_2
		goto	disp_set_9
disp_set_7
		movlw	d'10'			;OFF
		movwf	cs3_s3
		movwf	cs3_s4
		movwf	cs4_s3
		movwf	cs4_s4
		movlw	d'11'			;C
		movwf	cs3_s2
		movwf	cs4_s2
		movlw	d'12'			;h
		movwf	cs3_s1
		movwf	cs4_s1
		btfsc	flag,set_ch
		goto	disp_set_7_2
disp_set_7_1
		movlw	d'1'
		movwf	cs4_s0
		movlw	d'2'
		movwf	cs3_s0
		goto	disp_set_7_3
disp_set_7_2
		movlw	d'3'
		movwf	cs4_s0
		movlw	d'4'
		movwf	cs3_s0
disp_set_7_3
		movlw	d'11'			;C
		btfsc	flag_3,ten_set
		movlw	d'10'			;OFF
		btfsc	flag_3,error_set
		movlw	d'13'			;E
		btfsc	flag_3,stop_set
		movlw	d'0'			;0
		movwf	cs3_s5
		movwf	cs4_s5

	 	btfss	flag_3,ten_set       ;k9
	 	goto	Disp_set_end	      

		movlw	cs3_s0        
		movwf	fsrtemp_1     
		clrf	fsrtemp_2     
		movlw	d'5'            
		btfsc	flag,set_ch     
		movwf	fsrtemp_2     

		movlw	windlen2_0     
		addwf	fsrtemp_2,f    


		call	Disp_6digit    



		goto	Disp_set_end
disp_set_8
		call	Disp_6digit
		goto	Disp_set_end
disp_set_9
		call	Disp_3digit

Disp_set_end
		return

;-----------------------------------------------
;３桁数字の表示
Disp_3digit
		clrf	cntr
disp_3digit_1
		btfsc	flag_5,disp_ten
		goto	disp_3digit_1_2
disp_3digit_1_1
		movf	fsrtemp_2,w
		movwf	FSR
		movf	INDF,w
		call	BINtoASC
		movf	fsrtemp_1,w
		movwf	FSR
		movf	_tmp_1,w
		movwf	INDF
		incf	FSR,f
		movf	_tmp_2,w
		movwf	INDF
		incf	FSR,f
		movf	_tmp_3,w
		movwf	INDF
		movf	FSR,w
		call	Zero_check_1
		goto	disp_3digit_1_3
disp_3digit_1_2
		movf	fsrtemp_2,w
		movwf	FSR
		movf	INDF,w
		call	BINtoASC
		movf	fsrtemp_1,w
		movwf	FSR
		movf	_tmp_1,w
		movwf	INDF
		incf	fsrtemp_1,f
		incf	fsrtemp_2,w
		movwf	FSR
		movf	INDF,w
		call	BINtoASC
		movf	fsrtemp_1,w
		movwf	FSR
		movf	_tmp_1,w
		movwf	INDF
		incf	FSR,f
		movf	_tmp_2,w
		movwf	INDF
		decf	fsrtemp_1,f
		movf	FSR,w
		call	Zero_check_1
disp_3digit_1_3
		movlw	cs1_s0
		subwf	fsrtemp_1,w
		btfsc	STATUS,C
		goto	disp_3digit_3
disp_3digit_2
		movlw	d'3'
		addwf	fsrtemp_1,f
		goto	disp_3digit_4
disp_3digit_3
		movlw	d'3'
		addwf	fsrtemp_1,w
		movwf	FSR
		movlw	d'10'
		movwf	INDF
		incf	FSR,f
		movwf	INDF
		incf	FSR,f
		movwf	INDF
		movlw	d'6'
		addwf	fsrtemp_1,f
disp_3digit_4
		movf	cntr,w
		btfss	STATUS,Z
		goto	Disp_3digit_end
		incf	cntr,f
		movlw	d'10'
		subwf	fsrtemp_2,f
		goto	disp_3digit_1

Disp_3digit_end
		return

;-----------------------------------------------
;６桁数字の表示
Disp_6digit
		clrf	cntr
		movlw	d'3'
		movwf	cntr_1
disp_6digit_1
		movf	fsrtemp_2,w
		movwf	FSR
		movf	INDF,w
		call	BINtoASC
		movf	fsrtemp_1,w
		movwf	FSR
		movf	_tmp_1,w
		movwf	INDF
		incf	FSR,f
		movf	_tmp_2,w
		movwf	INDF
		movlw	d'2'
		addwf	fsrtemp_1,f
		incf	fsrtemp_2,f
		decfsz	cntr_1,f
		goto	disp_6digit_1
disp_6digit_2
		movf	FSR,w
		call	Zero_check_2
		movf	cntr,w
		btfss	STATUS,Z
		goto	Disp_6digit_end
		incf	cntr,f
		movlw	d'13'
		subwf	fsrtemp_2,f
		movlw	d'3'
		movwf	cntr_1
		goto	disp_6digit_1

Disp_6digit_end
		return

;-----------------------------------------------
;RESETスイッチが押されたときの処理
Push_reset
		btfsc	flag_2,_reset_ON
		goto	reset_count

		movf	flag_2,w
		andlw	b'00000111'
		btfss	STATUS,Z
		goto	Push_resetend
		bsf	flag_2,_reset_ON
;		movlw	d'167'
		movlw	d'83'
		movwf	cntr
reset_count
		decfsz	cntr,f
		goto	Push_resetend
		movf	flag_5,w
		andlw	b'11000000'
		btfss	STATUS,Z
		bsf	flag,_mod_reset
		btfsc	flag_3,ten_set
		bsf	flag,_mod_reset
		btfsc	flag_3,error_set
		bsf	flag,_mod_setting
		btfsc	flag_3,stop_set
		bsf	flag,_mod_setting
		btfsc	flag_3,const_set
		bsf	flag,_mod_cont_const
		bcf	flag_2,_reset_ON
		btfsc	flag,_mod_reset
		call	Sum_checksum_1
		btfsc	flag,_mod_setting
		call	Sum_checksum_2
Push_resetend
		bsf	flag_2,_s1_ON

		return
;-----------------------------------------------
;SENDスイッチが押されたときの処理
Push_send
		btfsc	flag_2,_send_ON
		goto	send_count

		movf	flag_2,w
		andlw	b'00000111'
		btfss	STATUS,Z
		goto	Push_sendend
		bsf	flag_2,_send_ON
;		movlw	d'167'
		movlw	d'83'
		movwf	cntr
send_count
		decfsz	cntr,f
		goto	Push_sendend
		bsf	flag,_mod_send
		bcf	flag_2,_send_ON
		goto	Push_sendend

Push_sendend
		bsf	flag_2,_s1_ON

		return
;-----------------------------------------------
;SENS ADJスイッチが押されたときの処理
Push_sensadj
		btfsc	flag_2,_sensadj_ON
		goto	sensadj_count

		movf	flag_2,w
		andlw	b'00000111'
		btfss	STATUS,Z
		goto	Push_sensadjend
		bsf	flag_2,_sensadj_ON
;		movlw	d'167'
		movlw	d'83'
		movwf	cntr
sensadj_count
		decfsz	cntr,f
		goto	Push_sensadjend
		bsf	flag,_mod_sensadj
		bcf	flag_2,_sensadj_ON
		goto	Push_sensadjend

Push_sensadjend
		bsf	flag_2,_s1_ON

		return
;-----------------------------------------------
;ENTERスイッチが押されたときの処理
Push_enter
		movf	flag_2,w
		andlw	b'00000111'
		btfss	STATUS,Z
		goto	Push_enterend

		movf	flag_5,w		;設定モードか？
		andlw	b'11000000'
		btfss	STATUS,Z
		goto	Push_enterend

		movf	flag_3,w		;設定モードか？
		andlw	b'00001111'
		btfss	STATUS,Z
		goto	enter_event
		btfsc	flag,set_ch
		goto	set_channel_12
set_channel_34					;3,4ch表示
		bcf	flag,set_ch        
		bcf	flag_2,_enter_ON
		goto	disp_table
set_channel_12					;1,2ch表示
		bcf	flag,set_ch
		bcf	flag_2,_enter_ON
disp_table
		call	Disp_set		;表示切り替え
		goto	Push_enterend

enter_event
		movf	flag_3,w
		andlw	b'00000011'
		btfsc	STATUS,Z
		goto	enter_event_2
enter_event_1
		movlw	cs0_s1
		movwf	FSR
		movlw	d'3'
		btfsc	flag_3,ten_1
		addwf	FSR,f
		movf	blink_reg,w
		movwf	_tmp_1
		movf	INDF,w
		movwf	_tmp_2
		incf	FSR,f
		movf	INDF,w
		movwf	_tmp_3
		call	LEDtoDAT
enter_event_1_1
		btfss	flag_3,ten_set
		goto	$+3
		movlw	tens_ch1
		goto	enter_event_1_2
		btfss	flag_3,error_set
		goto	$+3
		movlw	errortens_ch1
		goto	enter_event_1_2
		btfss	flag_3,stop_set
		goto	$+3
		movlw	stoptime_ch1
		goto	enter_event_1_2
		movlw	const_kp_ch1
enter_event_1_2
		movwf	FSR
		movlw	d'5'
		btfsc	flag,set_ch
		addwf	FSR,f
		movlw	d'10'
		btfss	flag_3,ten_1
		addwf	FSR,f
		movf	_prm_1,w
		movwf	INDF
		call	BINtoASC
		movlw	cs0_s0
		movwf	FSR
		movlw	d'3'
		btfsc	flag_3,ten_1
		addwf	FSR,f
		movf	_tmp_1,w
		movwf	INDF
		incf	FSR,f
		movf	_tmp_2,w
		movwf	INDF
		incf	FSR,f
		movf	_tmp_3,w
		movwf	INDF
		movf	FSR,w
		call	Zero_check_1
		bsf	flag,_mod_save
		goto	Push_enterend

enter_event_2
		btfss	flag_3,ten_set
		goto	enter_event_2_0
		movlw	d'3'
		movwf	cntr
		movlw	cs1_s0
		movwf	fsrtemp_1
		movlw	d'6'
		btfsc	flag_3,len_1
		addwf	fsrtemp_1,f
		movlw	lench1_0
		movwf	fsrtemp_2
		movlw	d'5'
		btfsc	flag,set_ch
		addwf	fsrtemp_2,f
		movlw	d'10'
		btfss	flag_3,len_1
		addwf	fsrtemp_2,f
		goto	enter_event_2_3
enter_event_2_0
		movlw	cs1_s1
		movwf	FSR
		movlw	d'6'
		btfsc	flag_3,len_1
		addwf	FSR,f
		movf	blink_reg,w
		movwf	_tmp_1
		movf	INDF,w
		movwf	_tmp_2
		incf	FSR,f
		movf	INDF,w
		movwf	_tmp_3
		call	LEDtoDAT
enter_event_2_1
		btfss	flag_3,error_set
		goto	$+3
		movlw	errortime_ch1
		goto	enter_event_2_2
		btfss	flag_3,stop_set
		goto	$+3
		movlw	rotate_ch1
		goto	enter_event_2_2
		movlw	const_ti_ch1
enter_event_2_2
		movwf	FSR
		movlw	d'5'
		btfsc	flag,set_ch
		addwf	FSR,f
		movlw	d'10'
		btfss	flag_3,len_1
		addwf	FSR,f
		movf	_prm_1,w
		movwf	INDF
		call	BINtoASC
		movlw	cs1_s0
		movwf	FSR
		movlw	d'6'
		btfsc	flag_3,len_1
		addwf	FSR,f
		movf	_tmp_1,w
		movwf	INDF
		incf	FSR,f
		movf	_tmp_2,w
		movwf	INDF
		incf	FSR,f
		movf	_tmp_3,w
		movwf	INDF
		movf	FSR,w
		call	Zero_check_1
		bsf	flag,_mod_save
		goto	Push_enterend
enter_event_2_3
		movf	fsrtemp_1,w
		movwf	FSR
		movf	cntr,w
		sublw	d'3'
		btfss	STATUS,Z
		goto	$+3
		movf	blink_reg,w
		movwf	INDF
		movf	INDF,w
		movwf	_tmp_1
		incf	FSR,f
		movf	INDF,w
		movwf	_tmp_2
		clrf	_tmp_3
		call	LEDtoDAT
		movlw	d'2'
		addwf	fsrtemp_1,f
		movf	fsrtemp_2,w
		movwf	FSR
		movf	_prm_1,w
		movwf	INDF
		incf	fsrtemp_2,f
		decfsz	cntr,f
		goto	enter_event_2_3
		bsf	flag,_mod_save

Push_enterend
		bsf	flag_2,_s1_ON

		return

;-----------------------------------------------
;7セグに表示している数字を張力設定データに変換
LEDtoDAT
		clrf	_prm_1

		movf	_tmp_1,w
		sublw	d'10'
		btfsc	STATUS,Z
		clrf	_tmp_1
		movf	_tmp_2,w
		sublw	d'10'
		btfsc	STATUS,Z
		clrf	_tmp_2
		movf	_tmp_3,w
		sublw	d'10'
		btfsc	STATUS,Z
		clrf	_tmp_3
LEDtoDAT_1
		movf	_tmp_1,w
		btfsc	STATUS,Z
		goto	LEDtoDAT_2
		movf	_tmp_1,w
		addwf	_prm_1,f
LEDtoDAT_2
		movf	_tmp_2,w
		btfsc	STATUS,Z
		goto	LEDtoDAT_3
		movlw	d'10'
		addwf	_prm_1,f
		decfsz	_tmp_2,f
		goto	$-2
LEDtoDAT_3
		movf	_tmp_3,w
		btfsc	STATUS,Z
		goto	LEDtoDAT_5
		movlw	d'100'
		addwf	_prm_1,f
		btfsc	STATUS,C
		goto	LEDtoDAT_4
		decfsz	_tmp_3,f
		goto	$-4
		goto	LEDtoDAT_5

LEDtoDAT_4
		movlw	h'ff'
		movwf	_prm_1

LEDtoDAT_5
		movf	_prm_1,w		;下限判定
		btfss	STATUS,Z
		goto	LEDtoDAT_6

		movf	flag_3,w
		andlw	b'00000011'
		btfss	STATUS,Z
		goto	LEDtoDAT_5_1
		movf	flag_3,w
		andlw	b'11000000'
		btfss	STATUS,Z
		goto	LEDtoDAT_5_1
		goto	LEDtoDAT_end
LEDtoDAT_5_1
		movlw	d'1'
		movwf	_prm_1
		goto	LEDtoDAT_end
LEDtoDAT_6
		movf	flag_3,w
		andlw	b'10010000'
		btfss	STATUS,Z
		goto	LEDtoDAT_end

		btfsc	flag_3,error_set
		goto	LEDtoDAT_6_1
		goto	LEDtoDAT_6_4
LEDtoDAT_6_1
		movf	_prm_1,w		;100(10.0)以上か？
		sublw	d'100'
		btfsc	STATUS,C
		goto	LEDtoDAT_6_2
		movlw	d'100'
		movwf	_prm_1
LEDtoDAT_6_2
		movf	flag_3,w		;errortens?
		andlw	b'00000011'
		btfsc	STATUS,Z
		goto	LEDtoDAT_end
LEDtoDAT_6_3
		movlw	tens_ch1		;設定張力以下か？
		movwf	FSR
		movlw	d'5'
		btfsc	flag,set_ch
		addwf	FSR,f
		movlw	d'10'
		btfss	flag_3,ten_1
		addwf	FSR,f
		movf	_prm_1,w
		subwf	INDF,w
		btfsc	STATUS,C
		goto	LEDtoDAT_end
		movf	INDF,w
		movwf	_prm_1
		goto	LEDtoDAT_end
LEDtoDAT_6_4
		movf	flag_3,w		;stoptime?
		andlw	b'00000011'
		btfsc	STATUS,Z
		goto	LEDtoDAT_end
		movf	_prm_1,w		;100(10.0)以上か？
		sublw	d'100'
		btfsc	STATUS,C
		goto	$+3
		movlw	d'100'
		movwf	_prm_1

LEDtoDAT_end
		return

;-----------------------------------------------
;2進-10進変換(8bit to 3)
BINtoASC
		movwf	_prm_1
		clrf	_tmp_3
		clrf	_tmp_2
		clrf	_tmp_1

		movlw	d'100'
BINtoASC1	subwf	_prm_1,f
		btfss	STATUS,C
		goto	BINtoASC2
		incf	_tmp_3,f
		goto	BINtoASC1

BINtoASC2	addwf	_prm_1,f

		movlw	d'10'
BINtoASC3	subwf	_prm_1,f
		btfss	STATUS,C
		goto	BINtoASC4
		incf	_tmp_2,f
		goto	BINtoASC3

BINtoASC4	addwf	_prm_1,w
		movwf	_tmp_1

		return

;-----------------------------------------------
;７セグにゼロを表示するか？(３桁)
Zero_check_1
		movwf	FSR
		movf	INDF,w
		btfss	STATUS,Z
		goto	Zero_check_1end
		movlw	d'10'
		movwf	INDF
		decf	FSR,f

;added by hnk

		movf	INDF,w  ;hk
		btfss	STATUS,Z  ;
		goto	Zero_check_1end  
		movlw	d'10'  
		movwf	INDF  

                return


		decf	FSR,f   
		movf	INDF,w  ;
		btfss	STATUS,Z  
		goto	Zero_check_1end  
		movlw	d'10'  
		movwf	INDF  
    
                return


		movf	flag_3,w
		andlw	b'10000000'  ;CONST SET
		btfss	STATUS,Z
		goto	zero_check_1_1

		movf	flag_3,w
		andlw	b'01000000'  ;STOP SET
		btfsc	STATUS,Z
		goto	Zero_check_1end

		movlw	cs1_s0
		subwf	FSR,w
		btfss	STATUS,C
		goto	Zero_check_1end

zero_check_1_1
		movf	INDF,w
		btfss	STATUS,Z
		goto	Zero_check_1end
		movlw	d'10'
		movwf	INDF

Zero_check_1end
		return

;-----------------------------------------------
;７セグにゼロを表示するか？(６桁)
Zero_check_2
		movwf	FSR
		movf	INDF,w
		btfss	STATUS,Z
		goto	Zero_check_2end
		movlw	d'10'
		movwf	INDF
		decf	FSR,f
		movf	INDF,w
		btfss	STATUS,Z
		goto	Zero_check_2end
		movlw	d'10'
		movwf	INDF
		decf	FSR,f
		movf	INDF,w
		btfss	STATUS,Z
		goto	Zero_check_2end
		movlw	d'10'
		movwf	INDF
		decf	FSR,f
		movf	INDF,w
		btfss	STATUS,Z
		goto	Zero_check_2end
		movlw	d'10'
		movwf	INDF
		decf	FSR,f
		movf	INDF,w
		btfss	STATUS,Z
		goto	Zero_check_2end
		movlw	d'10'
		movwf	INDF

Zero_check_2end
		return

;-----------------------------------------------
;チェックサムの計算
Sum_checksum_1
Sum_checksum_1
		bsf	STATUS,RP0
		clrw
		addwf	lench1_0,w
		addwf	lench1_1,w
		addwf	lench1_2,w
		addwf	tens_ch1,w
		addwf	tens_ch1,w 
		andlw	b'01111111'
		movwf	checksum_1_1

		clrw
		addwf	lench2_0,w
		addwf	lench2_1,w
		addwf	lench2_2,w
		addwf	tens_ch2,w
		addwf	tens_ch2,w 
		andlw	b'01111111'
		movwf	checksum_1_2

		clrw
		addwf	lench3_0,w
		addwf	lench3_1,w
		addwf	lench3_2,w
		addwf	tens_ch3,w
		addwf	tens_ch3,w 
		andlw	b'01111111'
		movwf	checksum_1_3

		clrw
		addwf	lench4_0,w
		addwf	lench4_1,w
		addwf	lench4_2,w
		addwf	tens_ch4,w
		addwf	tens_ch4,w 
		andlw	b'01111111'
		movwf	checksum_1_4
		bcf	STATUS,RP0

		return

		return

;-----------------------------------------------
;チェックサムの計算
Sum_checksum_2
		bsf	STATUS,RP0
		clrw
		addwf	stoptime_ch1,w
		addwf	rotate_ch1,w
		addwf	errortens_ch1,w
		addwf	errortime_ch1,w
		andlw	b'01111111'
		movwf	checksum_2_1

		clrw
		addwf	stoptime_ch2,w
		addwf	rotate_ch2,w
		addwf	errortens_ch2,w
		addwf	errortime_ch2,w
		andlw	b'01111111'
		movwf	checksum_2_2

		clrw
		addwf	stoptime_ch3,w
		addwf	rotate_ch3,w
		addwf	errortens_ch3,w
		addwf	errortime_ch3,w
		andlw	b'01111111'
		movwf	checksum_2_3

		clrw
		addwf	stoptime_ch4,w
		addwf	rotate_ch4,w
		addwf	errortens_ch4,w
		addwf	errortime_ch4,w
		andlw	b'01111111'
		movwf	checksum_2_4
		bcf	STATUS,RP0

		return

;-----------------------------------------------
;ch1張力設定スイッチが押されたときの処理
Push_ten_set_1
		movf	flag_2,w
		andlw	b'00000111'
		btfss	STATUS,Z
		goto	Push_ten_set_1end

		movf	flag_3,w
		andlw	b'11110000'
		btfsc	STATUS,Z
		goto	Push_ten_set_1end

		call	Check_set
		btfsc	flag_3,ten_1
		goto	out_set_ten_1

		movlw	b'11110000'
		andwf	flag_3,f
		bsf	flag_3,ten_1
		movf	cs0_s3,w
		movwf	setreg_0
		movf	cs0_s4,w
		movwf	setreg_1
		movf	cs0_s5,w
		movwf	setreg_2

		clrf	shift_cntr
		clrf	cs0_s3
		clrf	blink_reg
		clrf	cs0_s4
		clrf	cs0_s5
		movlw	cs0_s5
		call	Zero_check_1
		goto	Push_ten_set_1end

out_set_ten_1
		bcf	flag_3,ten_1

Push_ten_set_1end
		bsf	flag_2,_s0_ON
	
		return
;-----------------------------------------------
;ch2張力設定スイッチが押されたときの処理
Push_ten_set_2
		movf	flag_2,w
		andlw	b'00000111'
		btfss	STATUS,Z
		goto	Push_ten_set_2end

		movf	flag_3,w
		andlw	b'11110000'
		btfsc	STATUS,Z
		goto	Push_ten_set_2end

		call	Check_set
		btfsc	flag_3,ten_2
		goto	out_set_ten_2

		movlw	b'11110000'
		andwf	flag_3,f
		bsf	flag_3,ten_2
		movf	cs0_s0,w
		movwf	setreg_0
		movf	cs0_s1,w
		movwf	setreg_1
		movf	cs0_s2,w
		movwf	setreg_2

		clrf	shift_cntr
		clrf	cs0_s0
		clrf	blink_reg
		clrf	cs0_s1
		clrf	cs0_s2
		movlw	cs0_s2
		call	Zero_check_1
		goto	Push_ten_set_2end

out_set_ten_2
		bcf	flag_3,ten_2

Push_ten_set_2end
		bsf	flag_2,_s0_ON

		return
;-----------------------------------------------
;ch1糸長設定スイッチが押されたときの処理
Push_len_set_1
		movf	flag_2,w
		andlw	b'00000111'
		btfss	STATUS,Z
		goto	Push_len_set_1end

		movf	flag_3,w
		andlw	b'11110000'
		btfsc	STATUS,Z
		goto	Push_len_set_1end

		call	Check_set
		btfsc	flag_3,len_1
		goto	out_set_len_1
in_set_len_1
		movlw	b'11110000'
		andwf	flag_3,f
		bsf	flag_3,len_1
		movf	cs2_s0,w
		movwf	setreg_0
		movf	cs2_s1,w
		movwf	setreg_1
		movf	cs2_s2,w
		movwf	setreg_2
		movf	cs2_s3,w
		movwf	setreg_3
		movf	cs2_s4,w
		movwf	setreg_4
		movf	cs2_s5,w
		movwf	setreg_5

		clrf	shift_cntr
		clrf	cs2_s0
		clrf	blink_reg

		movf	flag_3,w
		andlw	b'00010100'
		sublw	b'00010100'
		btfss	STATUS,Z
		goto	in_set_len_1_2
in_set_len_1_1
		movlw	d'10'
		movwf	cs2_s1
		movwf	cs2_s2
		movwf	cs2_s3
		movwf	cs2_s4
		movwf	cs2_s5
		goto	Push_len_set_1end
in_set_len_1_2
		movlw	d'10'
		movwf	cs2_s5
		movwf	cs2_s4
		movwf	cs2_s3
		clrf	cs2_s2
		clrf	cs2_s1
		movlw	cs2_s2
		call	Zero_check_1
		goto	Push_len_set_1end

out_set_len_1
		bcf	flag_3,len_1

Push_len_set_1end
		bsf	flag_2,_s0_ON

		return
;-----------------------------------------------
;ch2糸長設定スイッチが押されたときの処理
Push_len_set_2
		movf	flag_2,w
		andlw	b'00000111'
		btfss	STATUS,Z
		goto	Push_len_set_2end

		movf	flag_3,w
		andlw	b'11110000'
		btfsc	STATUS,Z
		goto	Push_len_set_2end

		call	Check_set
		btfsc	flag_3,len_2
		goto	out_set_len_2

		movlw	b'11110000'
		andwf	flag_3,f
		bsf	flag_3,len_2
		movf	cs1_s0,w
		movwf	setreg_0
		movf	cs1_s1,w
		movwf	setreg_1
		movf	cs1_s2,w
		movwf	setreg_2
		movf	cs1_s3,w
		movwf	setreg_3
		movf	cs1_s4,w
		movwf	setreg_4
		movf	cs1_s5,w
		movwf	setreg_5

		clrf	shift_cntr
		clrf	cs1_s0
		clrf	blink_reg

		movf	flag_3,w
		andlw	b'00011000'
		sublw	b'00011000'
		btfss	STATUS,Z
		goto	in_set_len_2_2
in_set_len_2_1
		movlw	d'10'
		movwf	cs1_s1
		movwf	cs1_s2
		movwf	cs1_s3
		movwf	cs1_s4
		movwf	cs1_s5
		goto	Push_len_set_2end
in_set_len_2_2
		movlw	d'10'
		movwf	cs1_s5
		movwf	cs1_s4
		movwf	cs1_s3
		clrf	cs1_s2
		clrf	cs1_s1
		movlw	cs1_s2
		call	Zero_check_1
		goto	Push_len_set_2end

out_set_len_2
		bcf	flag_3,len_2

Push_len_set_2end
		bsf	flag_2,_s0_ON

		return

;-----------------------------------------------
Push_check
		movf	flag_2,w
		andlw	b'00000111'
		btfss	STATUS,Z
		goto	Push_checkend

		bsf	flag,_mod_check
Push_checkend
		bsf	flag_2,_s0_ON

		return

;-----------------------------------------------
;CLEARスイッチが押されたときの処理
Push_clear
		btfsc	flag_2,_clear_ON
		goto	clear_count

		movf	flag_2,w
		andlw	b'00000111'
		btfss	STATUS,Z
		goto	Push_clearend
		
		movf	flag_3,w
		andlw	b'00001111'
		btfss	STATUS,Z
		goto	clear_event
		bsf	flag_2,_clear_ON
;		movlw	d'167'
		movlw	d'83'
		movwf	cntr
clear_count
		decfsz	cntr,f
		goto	Push_clearend
		bsf	flag,_mod_check
		bcf	flag_2,_clear_ON
		goto	Push_clearend

clear_event
		movf	shift_cntr,w
		btfsc	STATUS,Z
		goto	Push_clearend
		movf	flag_3,w
		andlw	b'00000011'
		btfsc	STATUS,Z
		goto	clear_event_3
clear_event_1
		movf	shift_cntr,w
		sublw	d'1'
		btfss	STATUS,Z
		goto	clear_event_2
		movlw	cs0_s0
		movwf	FSR
		movlw	d'3'
		btfsc	flag_3,ten_1
		addwf	FSR,f
		clrf	INDF
		clrf	blink_reg
		decf	shift_cntr,f
		goto	Push_clearend
clear_event_2
		movlw	cs0_s1			;２桁目右シフト
		movwf	FSR
		movlw	d'3'
		btfsc	flag_3,ten_1
		addwf	FSR,f
		movf	INDF,w
		decf	FSR,f
		movwf	INDF
		movwf	blink_reg

		incf	FSR,f			;３桁目右シフト
		incf	FSR,f
		movlw	d'10'
		subwf	INDF,w
		btfss	STATUS,Z
		goto	clear_event_2_3
		btfss	flag_3,const_set
		goto	clear_event_2_2
clear_event_2_1
		movlw	d'10'
		goto	clear_event_2_4
clear_event_2_2
		movlw	d'0'
		goto	clear_event_2_4
clear_event_2_3
		movf	INDF,w
clear_event_2_4
		decf	FSR,f
		movwf	INDF

		incf	FSR,f			;３桁目クリア
		movlw	d'10'
		movwf	INDF
		decf	shift_cntr,f
		goto	Push_clearend

clear_event_3
		movf	shift_cntr,w
		sublw	d'1'
		btfss	STATUS,Z
		goto	clear_event_4
		movlw	cs1_s0
		movwf	FSR
		movlw	d'6'
		btfsc	flag_3,len_1
		addwf	FSR,f
		clrf	INDF
		clrf	blink_reg
		decf	shift_cntr,f
		goto	Push_clearend
clear_event_4
		movlw	cs1_s1			;２桁目右シフト
		movwf	FSR
		movlw	d'6'
		btfsc	flag_3,len_1
		addwf	FSR,f
		movf	INDF,w
		decf	FSR,f
		movwf	INDF
		movwf	blink_reg

		incf	FSR,f			;３桁目右シフト
		incf	FSR,f

		movlw	d'10'
		subwf	INDF,w
		btfss	STATUS,Z
		goto	clear_event_4_3
		btfss	flag_3,error_set
		goto	clear_event_4_2
clear_event_4_1
		movlw	d'0'
		goto	clear_event_4_4
clear_event_4_2
		movlw	d'10'
		goto	clear_event_4_4
clear_event_4_3
		movf	INDF,w
clear_event_4_4
		decf	FSR,f
		movwf	INDF

		incf	FSR,f			;４桁目右シフト
		incf	FSR,f
		movf	INDF,w
		decf	FSR,f
		movwf	INDF
		incf	FSR,f			;５桁目右シフト
		incf	FSR,f
		movf	INDF,w
		decf	FSR,f
		movwf	INDF
		incf	FSR,f			;６桁目右シフト
		incf	FSR,f
		movf	INDF,w
		decf	FSR,f
		movwf	INDF
		incf	FSR,f			;６桁目クリア
		movlw	d'10'
		movwf	INDF
		decf	shift_cntr,f
		goto	Push_clearend

Push_clearend
		bsf	flag_2,_s2_ON

		return

;-----------------------------------------------
;０スイッチが押されたときの処理
Push_num_0
		movf	flag_2,w
		andlw	b'00000111'
		btfss	STATUS,Z
		goto	Push_num_0end

		movf	flag_3,w
		andlw	b'00001111'
		btfsc	STATUS,Z
		goto	push_num_0_1

		movf	shift_cntr,w
		btfsc	STATUS,Z
		goto	Push_num_0end
		movlw	d'0'
		movwf	_prm_1
		call	Num_event
		goto	Push_num_0end

push_num_0_1
                goto Push_num_0end  ;hnk15

		movlw	b'00001111'
		andwf	flag_3,f
		movlw	b'00111111'
		andwf	flag_5,f
		bcf	flag,set_ch
		bsf	flag_3,const_set
		call	Disp_set

Push_num_0end
		bsf	flag_2,_s2_ON		

		return
;-----------------------------------------------
;１スイッチが押されたときの処理
Push_num_1
		movf	flag_2,w
		andlw	b'00000111'
		btfss	STATUS,Z
		goto	Push_num_1end

		movf	flag_3,w
		andlw	b'00001111'
		btfsc	STATUS,Z
		goto	push_num_1_1

		movlw	d'1'
		movwf	_prm_1
		call	Num_event
		goto	Push_num_1end

push_num_1_1
                 

		movlw	b'00001111'
		andwf	flag_3,f
		movlw	b'00111111'
		andwf	flag_5,f
		bcf	flag,set_ch
		bsf	flag_3,ten_set
		call	Disp_set

Push_num_1end
		bsf	flag_2,_s2_ON		

		return
;-----------------------------------------------
;２スイッチが押されたときの処理
Push_num_2
		movf	flag_2,w
		andlw	b'00000111'
		btfss	STATUS,Z
		goto	Push_num_2end

		movf	flag_3,w
		andlw	b'00001111'
		btfsc	STATUS,Z
		goto	push_num_2_1

		movlw	d'2'
		movwf	_prm_1
		call	Num_event
		goto	Push_num_2end

push_num_2_1
                goto    Push_num_2end  

		movlw	b'00001111'
		andwf	flag_3,f
		movlw	b'00111111'
		andwf	flag_5,f
		bcf	flag,set_ch
		bsf	flag_3,error_set
		call	Disp_set

Push_num_2end
		bsf	flag_2,_s2_ON		

		return
;-----------------------------------------------
;３スイッチが押されたときの処理
Push_num_3
		movf	flag_2,w
		andlw	b'00000111'
		btfss	STATUS,Z
		goto	Push_num_3end

		movf	flag_3,w
		andlw	b'00001111'
		btfsc	STATUS,Z
		goto	push_num_3_1

		movlw	d'3'
		movwf	_prm_1
		call	Num_event
		goto	Push_num_3end

push_num_3_1

                goto   Push_num_3end  

		movlw	b'00001111'
		andwf	flag_3,f
		movlw	b'00111111'
		andwf	flag_5,f
		bcf	flag,set_ch
		bsf	flag_3,stop_set
		call	Disp_set

Push_num_3end
		bsf	flag_2,_s1_ON		

		return
;-----------------------------------------------
;４スイッチが押されたときの処理
Push_num_4
		movf	flag_2,w
		andlw	b'00000111'
		btfss	STATUS,Z
		goto	Push_num_4end

		movf	flag_3,w
		andlw	b'00001111'
		btfsc	STATUS,Z
		goto	push_num_4_1

		movlw	d'4'
		movwf	_prm_1
		call	Num_event
		goto	Push_num_4end

push_num_4_1
                goto   Push_num_4end

		movlw	b'00001111'
		andwf	flag_3,f
		movlw	b'00111111'
		andwf	flag_5,f
		bcf	flag,set_ch
		bsf	flag_5,disp_ten
		call	Disp_set

Push_num_4end
		bsf	flag_2,_s2_ON		

		return
;-----------------------------------------------
;５スイッチが押されたときの処理
Push_num_5
		movf	flag_2,w
		andlw	b'00000111'
		btfss	STATUS,Z
		goto	Push_num_5end

		movf	flag_3,w
		andlw	b'00001111'
		btfsc	STATUS,Z
		goto	push_num_5_1

		movlw	d'5'
		movwf	_prm_1
		call	Num_event
		goto	Push_num_5end

push_num_5_1
                goto   Push_num_3end
		movlw	b'00001111'
		andwf	flag_3,f
		movlw	b'00111111'
		andwf	flag_5,f
		bcf	flag,set_ch
		bsf	flag_5,disp_len
		call	Disp_set

Push_num_5end
		bsf	flag_2,_s2_ON		

		return
;-----------------------------------------------
;６スイッチが押されたときの処理
Push_num_6
		movf	flag_2,w
		andlw	b'00000111'
		btfss	STATUS,Z
		goto	Push_num_6end

		movf	flag_3,w
		andlw	b'00001111'
		btfsc	STATUS,Z
		goto	Push_num_6end

		movlw	d'6'
		movwf	_prm_1
		call	Num_event

Push_num_6end
		bsf	flag_2,_s1_ON		

		return
;-----------------------------------------------
;７スイッチが押されたときの処理
Push_num_7
		movf	flag_2,w
		andlw	b'00000111'
		btfss	STATUS,Z
		goto	Push_num_7end

		movf	flag_3,w
		andlw	b'00001111'
		btfsc	STATUS,Z
		goto	Push_num_7end

		movlw	d'7'
		movwf	_prm_1
		call	Num_event

Push_num_7end
		bsf	flag_2,_s2_ON		

		return
;-----------------------------------------------
;８スイッチが押されたときの処理
Push_num_8
		movf	flag_2,w
		andlw	b'00000111'
		btfss	STATUS,Z
		goto	Push_num_8end

		movf	flag_3,w
		andlw	b'00001111'
		btfsc	STATUS,Z
		goto	Push_num_8end

		movlw	d'8'
		movwf	_prm_1
		call	Num_event

Push_num_8end
		bsf	flag_2,_s2_ON		

		return
;-----------------------------------------------
;９スイッチが押されたときの処理
Push_num_9
		movf	flag_2,w
		andlw	b'00000111'
		btfss	STATUS,Z
		goto	Push_num_9end

		movf	flag_3,w
		andlw	b'00001111'
		btfsc	STATUS,Z
		goto	Push_num_9end

		movlw	d'9'
		movwf	_prm_1
		call	Num_event

Push_num_9end
		bsf	flag_2,_s1_ON		

		return
;-----------------------------------------------
;数字スイッチが押されたときの処理
Num_event
		movf	flag_3,w
		andlw	b'00000011'
		btfsc	STATUS,Z
		goto	num_event_3
num_event_1					;張力設定部の場合
		movf	shift_cntr,w
		sublw	d'3'
		btfsc	STATUS,Z
		goto	num_event_end
		movf	shift_cntr,w
		btfss	STATUS,Z
		goto	num_event_2
		movlw	cs0_s0
		movwf	FSR
		movlw	d'3'
		btfsc	flag_3,ten_1
		addwf	FSR,f
		movf	_prm_1,w
		movwf	INDF
		movwf	blink_reg
		incf	shift_cntr,f
		goto	num_event_end
num_event_2
		movlw	cs0_s1			;２桁目左シフト
		movwf	FSR
		movlw	d'3'
		btfsc	flag_3,ten_1
		addwf	FSR,f
		movlw	d'0'
		subwf	INDF,w
		btfss	STATUS,Z
		goto	num_event_2_2
num_event_2_1
		movlw	d'10'
		goto	num_event_2_3
num_event_2_2
		movf	INDF,w
num_event_2_3
		incf	FSR,f
		movwf	INDF

		decf	FSR,f			;１桁目左シフト
		movf	blink_reg,w
		movwf	INDF
		decf	FSR,f
		movf	_prm_1,w		;１桁目数字更新
		movwf	INDF
		movwf	blink_reg
		incf	shift_cntr,f
		goto	num_event_end

num_event_3
		btfss	flag_3,ten_set		;糸長設定か？
		goto	num_event_5		;糸長設定以外
		movf	shift_cntr,w
		sublw	d'6'
		btfsc	STATUS,Z
		goto	num_event_end
		movf	shift_cntr,w
		btfss	STATUS,Z
		goto	num_event_4
		movlw	cs1_s0
		movwf	FSR
		movlw	d'6'
		btfsc	flag_3,len_1
		addwf	FSR,f
		movf	_prm_1,w
		movwf	INDF
		movwf	blink_reg
		incf	shift_cntr,f
		goto	num_event_end
num_event_4
		movlw	cs1_s4
		movwf	FSR
		movlw	d'6'
		btfsc	flag_3,len_1
		addwf	FSR,f
		movf	INDF,w
		incf	FSR,f
		movwf	INDF
		decf	FSR,f
		decf	FSR,f
		movf	INDF,w
		incf	FSR,f
		movwf	INDF
		decf	FSR,f
		decf	FSR,f
		movf	INDF,w
		incf	FSR,f
		movwf	INDF
		decf	FSR,f
		decf	FSR,f
		movf	INDF,w
		incf	FSR,f
		movwf	INDF
		movf	blink_reg,w
		decf	FSR,f
		movwf	INDF
		movf	_prm_1,w
		decf	FSR,f
		movwf	INDF
		movwf	blink_reg
		incf	shift_cntr,f
		goto	num_event_end

num_event_5
		movf	shift_cntr,w
		sublw	d'3'
		btfsc	STATUS,Z
		goto	num_event_end
		movf	shift_cntr,w
		btfss	STATUS,Z
		goto	num_event_6
		movlw	cs1_s0
		movwf	FSR
		movlw	d'6'
		btfsc	flag_3,len_1
		addwf	FSR,f
		movf	_prm_1,w
		movwf	INDF
		movwf	blink_reg
		incf	shift_cntr,f
		goto	num_event_end
num_event_6
		movlw	cs1_s1
		movwf	FSR
		movlw	d'6'
		btfsc	flag_3,len_1
		addwf	FSR,f
		movf	INDF,w
		movlw	d'0'
		subwf	INDF,w
		btfss	STATUS,Z
		goto	num_event_6_2
num_event_6_1
		movlw	d'10'
		goto	num_event_6_3
num_event_6_2
		movf	INDF,w
num_event_6_3
		incf	FSR,f
		movwf	INDF

		movf	blink_reg,w
		decf	FSR,f
		movwf	INDF
		movf	_prm_1,w
		decf	FSR,f
		movwf	INDF
		movwf	blink_reg
		incf	shift_cntr,f
		goto	num_event_end

num_event_end
		return

;-----------------------------------------------
;セットキーが押されているか？
Check_set
		movf	flag_3,w
		andlw	b'00001111'
		btfsc	STATUS,Z
		goto	Check_setend

		movf	flag_3,f
		andlw	b'00000011'
		btfsc	STATUS,Z
		goto	Check_set_2
Check_set_1
		movlw	cs0_s0
		movwf	FSR
		movlw	d'3'
		btfsc	flag_3,ten_1
		addwf	FSR,f
		movf	setreg_0,w
		movwf	INDF
		incf	FSR,f
		movf	setreg_1,w
		movwf	INDF
		incf	FSR,f
		movf	setreg_2,w
		movwf	INDF
		goto	Check_setend
Check_set_2
		movlw	cs1_s0
		movwf	FSR
		movlw	d'6'
		btfsc	flag_3,len_1
		addwf	FSR,f
		movf	setreg_0,w
		movwf	INDF
		incf	FSR,f
		movf	setreg_1,w
		movwf	INDF
		incf	FSR,f
		movf	setreg_2,w
		movwf	INDF
		incf	FSR,f
		movf	setreg_3,w
		movwf	INDF
		incf	FSR,f
		movf	setreg_4,w
		movwf	INDF
		incf	FSR,f
		movf	setreg_5,w
		movwf	INDF

Check_setend
		return

;-----------------------------------------------
test:
;test
;COMM TEST
        BSF    PORTC,0
        BCF    PORTC,0
        BSF    PORTC,1
        BCF    PORTC,2   
        GOTO   mainloop     
;COMM TEST
;LED TEST
        CLRF    PORTA
        BSF     PORTA,0
        BSF     PORTA,1   
        BSF     PORTA,2
        BSF     PORTA,3  
        BSF     PORTA,4  		
        BSF     PORTA,5  

		MOVLW   0X22
        MOVWF   PORTB
        BSF     PORTA,2  ;DUAN

        CLRF    PORTA

        MOVLW   0x4;ｹｲﾑｶ
		bsf     PORTA,5;ｹｲﾑｶ

		bsf     PORTA,4	  ;DUAN	
		bsf     PORTA,3	  ;DUAN	
		bsf     PORTA,2	  ;DUAN	
		bsf     PORTA,1	  ;DUAN		
	    bsf     PORTA,0	  ;DUAN			
		
		MOVLW   0XFF
        MOVWF   PORTB
        BSF     PORTA,2  ;DUAN

        CLRF    PORTA
        MOVLW   0x4;ｹｲﾑｶ
		bsf     PORTA,5;ｹｲﾑｶ

		bsf     PORTA,4	  ;DUAN	
		bsf     PORTA,3	  ;DUAN	
		bsf     PORTA,2	  ;DUAN	
		bsf     PORTA,1	  ;DUAN		
	    bsf     PORTA,0	  ;DUAN				
		;GOTO    mainloop
		


		call	spbout  ;clrf	TRISB
		clrf	PORTA
		call	Cnv_scntr
		movwf	PORTB
		bsf	PORTA,cs5


		bcf	PORTA,cs5

        movlw 0x33
        movwf PORTB
		bsf	PORTA,cs5
        return
		end
