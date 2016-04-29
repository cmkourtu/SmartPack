#include <xc.h>

//#pragma config FWDTEN = OFF, JTAGEN = OFF

#pragma config POSCMOD = NONE           // Primary Oscillator Select (Primary oscillator disabled)
#pragma config OSCIOFNC = OFF           // Primary Oscillator Output Function (OSC2/CLKO/RC15 functions as CLKO (FOSC/2))
#pragma config FCKSM = CSECME           // Clock Switching and Monitor (Clock switching is enabled, Fail-Safe Clock Monitor is enabled)
#pragma config FNOSC = FRC              // Oscillator Select (Fast RC Oscillator (FRC))
#pragma config IESO = OFF               // Internal External Switch Over Mode (IESO mode (Two-Speed Start-up) disabled)

// CONFIG1
#pragma config WDTPS = PS32768          // Watchdog Timer Postscaler (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler (Prescaler ratio of 1:128)
#pragma config WINDIS = ON              // Watchdog Timer Window (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config ICS = PGx2               // Comm Channel Select (Emulator/debugger uses EMUC2/EMUD2)
#pragma config GWRP = OFF               // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF                // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF //ON

#define rfid_rts    LATAbits.LATA0
#define bt_rts      LATAbits.LATA1
#define gps_rts     LATAbits.LATA2
#define lcd_clk     LATAbits.LATA3
#define lcd_rs      LATAbits.LATA4
#define lcd_stcp    LATAbits.LATA5
#define fg_int      !PORTAbits.RA6
#define fg_pol      PORTAbits.RA7
#define btn         !PORTAbits.RA9
#define led         LATAbits.LATA10
#define fg_rts      LATAbits.LATA14
#define bt_con      PORTAbits.RA15
#define lcd_rts     LATBbits.LATB0
#define fg_clr      LATBbits.LATB1

void shiftOut(char);
void printLCDMessage(char*);
void printCharToLCD(char);
void clearLCD();
void initLCD();
void delay();
void uart2String(int, char*);
void uart2StringNoSize(char*);
void sendBTMessage(int, char*);
void debounce();
void initializeModules();
void btInit();
void rfidInit();
void rfidClearTags();
void rfidGetTags();
void rfidPollTags();
void uart1String(int, char*);
void btReadMessage();

unsigned int tim_flag = 0;
unsigned int btn_flag = 0;
unsigned int bt_tx_flag = 0;
unsigned int bt_rx_flag = 0;
unsigned int rfid_flag = 0;

int tim_counter = 0;
unsigned int btn_prevstate = 0;
int btn_counter = 0;
unsigned int btn_processed = 0;
char lcd_message[4][33] = {"Battery Charge              100%","Items in the    SmartPack:     0","BT connection:  Craig's iPhone  ","GPS location:   040.425N086.912W"};

int lcd_type = 3;
unsigned short lcd_cursor = 0;
long lcd_charge = 100;
long total_interupts = 4000; //40320
long charge = 4000;
int lcd_items = 0;
char lcd_btconnected [16] =    "Connected       ";
char lcd_btnotconnected [16] = "Not connected   ";

char bt_buffer [100];
unsigned int bt_buffer_back = 0;
unsigned int bt_buffer_front = 0;
unsigned int bt_connected = 0;

char rfid_buffer [100];
unsigned int rfid_buffer_back = 0;
unsigned int rfid_buffer_front = 0;
char rfid_tags_size [] = {0xFF, 0x00, 0x29, 0x1D, 0x26, 0x00, 0x00};
char rfid_clear_tags[] = {0xFF, 0x00, 0x2A, 0x1D, 0x25, 0x00, 0x00};
char rfid_get_tag[] = {0xFF, 0x02, 0x29, 0x00, 0x01, 0x57, 0xE8};
//char rfid_poll_tags[] = {0xFF, 0x02, 0x22, 0x01, 0xF4, 0xE7, 0x76};
char rfid_poll_tags[] = {0xFF, 0x04, 0x22, 0x00, 0x01, 0x03, 0xE8, 0x3F, 0x8E};
char rfid_tags [96][20];
int rfid_total_tags = 0;
char bt_message[20] = {0x00, 0x00, 0, 0, 0, 0, 0, 0, 0x00, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int main(void)
{
    
     RCONbits.SWDTEN = 0;

    /* When clock switch occurs switch to Prim Osc (HS, XT, EC)with PLL */
    __builtin_write_OSCCONH(0x00); /* Set OSCCONH for clock switch */
    CLKDIV = 0x0700;
    __builtin_write_OSCCONL(0x01); /* Start clock switching */
    while (OSCCONbits.COSC != 0b000);
    
    TRISA = 0xBBC4;
    TRISB = 0xFFF8;


    
    SPI1CON1 = 0x0330;
    SPI1STATbits.SPIEN = 1;
    SPI1STATbits.SPISIDL = 0;
    SPI1STATbits.SPIROV = 0;
    SPI1CON2 = 0x0000;
    
    U2BRG = 8;
    U2MODE = 0x8988;
    U2STA = 0xA400;
    INTCON1bits.NSTDIS = 0;
    IEC1bits.U2RXIE = 0x01;
    IPC7bits.U2RXIP0 = 1;
    IFS1bits.U2RXIF = 0;
    
    U1BRG = 8;
    U1MODE = 0x8988;
    U1STA = 0xA400;
    IEC0bits.U1RXIE= 0x01;
    IPC2bits.U1RXIP0 = 1;
    IFS0bits.U1RXIF = 0;
    
    T1CON = 0x8000;
    IPC0bits.T1IP0 = 1;
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 0x01;

    initializeModules();
    
    
    while(1)
    {

        if (tim_flag)
        {
            int i;
            //clearLCD();
            //lcd_rts = 0;
            //printLCDMessage("There are ");
            rfidPollTags();
            rfid_buffer_back = 0;
            rfidGetTags();
            //************************************************
            /*
            char stemp[] = "1";
            stemp[0] = bt_message[0] + '0';
            printLCDMessage(stemp);
            printLCDMessage(" tags");
            (bt_message[8] & 1) == 0 ? printLCDMessage("0") : printLCDMessage("1");
            (bt_message[8] & 2) == 0 ? printLCDMessage("0") : printLCDMessage("1");
            (bt_message[8] & 4) == 0 ? printLCDMessage("0") : printLCDMessage("1");
            (bt_message[8] & 8) == 0 ? printLCDMessage("0") : printLCDMessage("1");
            (bt_message[8] & 16) == 0 ? printLCDMessage("0") : printLCDMessage("1");
            (bt_message[8] & 32) == 0 ? printLCDMessage("0") : printLCDMessage("1");
            (bt_message[8] & 64) == 0 ? printLCDMessage("0") : printLCDMessage("1");
            (bt_message[8] & 128) == 0 ? printLCDMessage("0") : printLCDMessage("1");
            (bt_message[9] & 1) == 0 ? printLCDMessage("0") : printLCDMessage("1");
            (bt_message[9] & 2) == 0 ? printLCDMessage("0") : printLCDMessage("1");
            */
            //************************************************
            
            led = bt_con;
            
            for (i = 0; i < 16; i++) lcd_message[2][16+i] = (bt_con ? lcd_btconnected[i] : lcd_btnotconnected[i]);
            if (bt_connected != bt_con && lcd_type == 1)
            {
                bt_connected = bt_con;
                btn_flag = 1;
                lcd_type--;
            }
            //bt_tx_flag = 1; // DELETE THIS
            tim_flag = 0;
        }
        
        if (bt_rx_flag)
        {
            bt_buffer_back = 0;
            uart2StringNoSize("SUR,12345678901234567890123456789AF1\r\n");
            int i;
            for (i = 0; i < 100; i++) delay();
            btReadMessage();
            
            /*if (lcd_type == 3)
            {
                btn_flag = 1;
                lcd_type--;
            }*/
            bt_buffer_back = 0;
            bt_buffer_front = 0;
            bt_rx_flag = 0;
        }
        
        if (btn_flag)
        {
            if (++lcd_type > 3) lcd_type = 0;
            if (lcd_type == 0) 
            {
                lcd_rts = 0;
                
            }
            else if (lcd_type == 3) 
            {
                lcd_rts = 1;
                clearLCD();
                lcd_rs = 0;
                delay();
                shiftOut(0xFE);
                shiftOut(0x90);
                lcd_rs = 1;
            }
            if (lcd_type < 3) printLCDMessage(lcd_message[lcd_type]);
            btn_processed = 1;
            btn_flag = 0;
        }
        
        if (bt_tx_flag)
        {
            uart2StringNoSize("SUW,12345678901234567890123456789AFF,");//123456789A123456789A123456789A123456789A\r\n");
            sendBTMessage(20, bt_message);
            //uart2StringNoSize(bt_message);
            uart2StringNoSize("\r\n");
            bt_tx_flag = 0;
        }
        
        if (fg_int) 
        {
            charge--;
            long newPercent = (charge * 100);
            newPercent = newPercent / total_interupts;
            if (newPercent != lcd_charge) {
                lcd_charge = newPercent;
                if (lcd_charge > 100) lcd_charge = 100;
                else if (lcd_charge < 5) lcd_charge = 5;
                lcd_message[0][30] = lcd_charge % 10 + '0';
                if (lcd_charge < 10) lcd_message[0][29] = ' ';
                else if (lcd_charge < 100) lcd_message[0][29] = lcd_charge / 10 + '0';
                else {
                    lcd_message[0][28] = '1';
                    lcd_message[0][29] = '0';
                }
                if (lcd_charge == 99) lcd_message[0][28] = ' ';
                if (lcd_type == 0) {
                    lcd_type = 3;
                    btn_flag = 1;
                }
            }

            fg_clr = 0;
            int i;
            for (i = 0; i < 50; i++);
            fg_clr = 1;
        }
        
        delay();
    }
}


void initializeModules()
{
    int i;
    lcd_rts = 1;
    fg_rts = 1;
    fg_clr = 1;
    bt_rts = 1;
    for (i = 0; i < 4000; i++) delay();
    rfid_rts = 0;
    btInit();
    initLCD();
    rfidInit();
}

void shiftOut(char aChar)
{
    
    SPI1BUF = aChar;
    int a;
    for (a = 0; a < 500; a++);
    lcd_stcp = 1;
    for (a = 0; a < 20; a++);
    lcd_stcp = 0;
    delay();
    lcd_clk = 0;
    delay();
    lcd_clk = 1;  
}

void printCharToLCD(char a)
{
    shiftOut(a);
    lcd_cursor++;
    if (lcd_cursor == 16) 
    {
        lcd_rs = 0;
        delay();
        shiftOut(0xFE);
        shiftOut(0xC0);
        lcd_rs = 1;
        delay();
    } 
    else if (lcd_cursor == 32) 
    {
        clearLCD();
        lcd_rs = 0;
        delay();
        shiftOut(0xFE);
        shiftOut(0x80);
        lcd_rs = 1;
        delay();
        lcd_cursor = 0;
    }
    
}

void printLCDMessage(char* word)
{
    int i = 0;
    clearLCD();          
    while (word[i] != 0) 
    {
        shiftOut(word[i++]);
        lcd_cursor++;
        if(lcd_cursor == 16)
        {
            lcd_rs = 0;
            delay();
            shiftOut(0xFE);
            shiftOut(0xC0);
            lcd_rs = 1;
            delay();
        }
    }
}
void clearLCD()
{
    lcd_rs = 0;
    delay();
    shiftOut(0x01);
    lcd_rs = 1;
    delay();
    lcd_cursor = 0;

}

void delay() // 2 miliseconds
{
    unsigned short j;
    for (j = 0; j < 667; j++);
}
void initLCD()
{
    lcd_stcp = 0;
    lcd_clk = 1;
    lcd_rs = 0;
    shiftOut(0x0F);  
    shiftOut(0x38);
    shiftOut(0x01);
    shiftOut(0xFE);
    shiftOut(0x90);
    lcd_rs = 1;
    lcd_rts = 1;
}

void sendBTMessage(int size, char* message)
{
    int i;
    char tempmess [40];
    for (i = 0; i < size; i++)
    {
        char a = (bt_message[i] >> 4) & 0x0F;
        char b = bt_message[i] & 0x0F;
        a = (a > 9 ? a - 10 + 'A' : a + '0');
        b = (b > 9 ? b - 10 + 'A' : b + '0');
        tempmess[2*i] = a;
        tempmess[2*i+1] = b;
    }
    uart2String(size*2,tempmess);
    
}

void uart2String(int size, char* word)
{
    int i = 0;
    for (i = 0; i < size; i++)
    {
        while (U2STAbits.TRMT == 0);
        U2TXREG = word[i];        
    }
    delay();
}

void uart2StringNoSize(char* word)
{
    int i = 0;
    while (word[i++] != '\0');
    uart2String(i-1,word);
}

void uart1String(int size, char* data) {
    int i;
    for (i = 0; i < size; i++) {
        while (U1STAbits.TRMT == 0);
        U1TXREG = data[i];
    }
    delay();
}




void debounce()
{
    if (btn_prevstate == 0) 
    {
        btn_counter = 0;
        btn_processed = 0;
    }
    btn_prevstate = 1;
    btn_counter++;
    if (btn_counter > 5 && !btn_processed) btn_flag = 1;
}

void btInit()
{
    int i;
    uart2StringNoSize("SN,SMARTPACK2\r\n");
    uart2StringNoSize("r,1\r\n");
    uart2StringNoSize("SS,80000001\r\n");
    for (i = 0; i < 200; i++) delay();
    uart2StringNoSize("SR,20004000\r\n");
    for (i = 0; i < 200; i++) delay();
    uart2StringNoSize("PZ\r\n");
    for (i = 0; i < 2000; i++) delay();
    
    
    uart2StringNoSize("gs\r\n");
    for (i = 0; i < 200; i++) delay();
    uart2StringNoSize("PS,123456789012345678901234567890FF\r\n"); // Creates private service with given UUID
    for (i = 0; i < 2000; i++) delay();
    uart2StringNoSize("PC,12345678901234567890123456789AFF,32,20\r\n"); // Creates private characteristics with given UUID 
    for (i = 0; i < 1000; i++) delay();
    //uart2StringNoSize("PC,12345678901234567890123456789AF1,0E,20\r\n"); // Creates private characteristics with given UUID 
    for (i = 0; i < 1000; i++) delay();
    uart2StringNoSize("r,1\r\n");
    for (i = 0; i < 4000; i++) delay();
    uart2StringNoSize("SUW,12345678901234567890123456789AFF,18\r\n");
    //for (i = 0; i < 5000; i++) delay(); //TEST TO WRITE VALUE
    //uart2StringNoSize("SUW,12345678901234567890123456789AFF,19\r\n");  
}

void btReadMessage()
{
    int temp = 16 * (bt_buffer[0] - '0') + (bt_buffer[1] - '0');
    lcd_message[3][16] = temp / 100 + '0';
    lcd_message[3][17] = (temp % 100) / 10 + '0';
    lcd_message[3][18] = temp % 10 + '0';
    temp = 4096 * (bt_buffer[2] - '0') + 256 * (bt_buffer[3] - '0') + 16 * (bt_buffer[4] - '0') + (bt_buffer[5] - '0');
    lcd_message[3][20] = temp / 1000 + '0';
    lcd_message[3][21] = (temp % 1000) / 100 + '0';
    lcd_message[3][22] = (temp % 100) / 10 + '0';
    temp = 16 * (bt_buffer[6] - '0') + (bt_buffer[7] - '0');
    lcd_message[3][24] = temp / 100 + '0';
    lcd_message[3][25] = (temp % 100) / 10 + '0';
    lcd_message[3][26] = temp % 10 + '0';
    temp = 4096 * (bt_buffer[8] - '0') + 256 * (bt_buffer[9] - '0') + 16 * (bt_buffer[10] - '0') + (bt_buffer[11] - '0');
    lcd_message[3][28] = temp / 1000 + '0';
    lcd_message[3][29] = (temp % 1000) / 100 + '0';
    lcd_message[3][30] = (temp % 100) / 10 + '0';
    temp = bt_buffer[13] - '0';
    //temp < 3 ? lcd_message[3][23] = 'N' : lcd_message[3][23] = 'S';
    //temp % 2 == 0 ? lcd_message[3][23] = 'E' : lcd_message[3][23] = 'W';
}

void rfidInit() {
    char data[] = {0xFF, 0x00, 0x04, 0x1D, 0x0B, 0x00, 0x00};
    int i;  
    rfid_buffer_back = 1;
    uart1String(5, data);  
    while (rfid_buffer_back < 8);
    //for (i = 0; i < 1000; i++) delay();
    data[1] = 0x02;
    data[2] = 0x93;
    data[3] = 0x00;
    data[4] = 0x05;
    data[5] = 0x51;
    data[6] = 0x7D;
    uart1String(7, data);
    //for (i = 0; i < 1000; i++) delay();
    while (rfid_buffer_back < 15);
    data[1] = 0x01;
    data[2] = 0x97;
    data[3] = 0x01;
    data[4] = 0x4B;
    data[5] = 0xBC;
    uart1String(6, data);
    //for (i = 0; i < 1000; i++) delay();
    while (rfid_buffer_back < 22);
    data[1] = 0x01;
    data[2] = 0x97;
    data[3] = 0x01;
    data[4] = 0x4B;
    data[5] = 0xBC;
    uart1String(6, data);
    //for (i = 0; i < 1000; i++) delay();
    while (rfid_buffer_back < 29);
    rfid_buffer_back = 0;
    rfid_flag = 0;
    
    for (i = 0; i < 100; i++) delay();
}

void rfidClearTags() {
    int temp = rfid_buffer_back;
    uart1String(5, rfid_clear_tags);
    while (rfid_buffer_back - temp < 7);
    delay();
}

void rfidGetTags() {
    rfid_buffer_back = 0;
    uart1String(5, rfid_tags_size);
    while (rfid_buffer_back < 11);
    int number_of_tags = rfid_buffer[rfid_buffer_back - 3] - rfid_buffer[rfid_buffer_back - 5];

    bt_message[0] = 0; //(char) number_of_tags;
    lcd_message[1][31] = (number_of_tags % 10) + '0';
    if (number_of_tags > 9) lcd_message[1][30] = (number_of_tags / 10) + '0';
    if (lcd_type == 1) 
    {
        btn_flag = 1;
        lcd_type--;
    }
    char temp_message[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int i;
    int j;
    int k;
    for (i = 0; i < number_of_tags; i++) {
        int tag_index = -1;
        rfid_buffer_back = 0;
        uart1String(7, rfid_get_tag);
        while (rfid_buffer_back < 75);
        unsigned int found = 0;
        for (j = 0; j < rfid_total_tags; j++) {
            found = 1;
            for (k = 0; k < 20; k++) {
                if (rfid_tags[j][k] != rfid_buffer[k + 5]) {
                    found = 0;
                    break;
                }
            }
            if (found == 1) {
                tag_index = j;
                break;
            }
        }
        if (found == 0) {
            for (k = 0; k < 20; k++) rfid_tags[rfid_total_tags][k] = rfid_buffer[k + 5];
            tag_index = rfid_total_tags;
            rfid_total_tags++;
        }
        int pow = 1;
        for (j = 0; j < tag_index % 8; j++) pow *= 2;
        if (!(temp_message[8 + tag_index / 8] & pow)) bt_message[0]++;
        temp_message[8 + tag_index / 8] = temp_message[8 + tag_index / 8] | pow;
    }


    for (i = 8; i < 20; i++) {
        if (temp_message[i] != bt_message[i]) {
            bt_tx_flag = 1;
            bt_message[i] = temp_message[i];
        }
    }
    
    rfidClearTags();
    rfid_buffer_back = 0;

}

void rfidPollTags() {
    //int temp = rfid_buffer_back;
    rfid_buffer_back = 0;
    uart1String(9, rfid_poll_tags);
    while (rfid_buffer_back < 7);
    delay();
}

void __attribute__((interrupt, no_auto_psv)) _U2RXInterrupt(void)
{
    int character = U2RXREG & 0x00FF;
    if (character == 10) character = 45;
    if (character > 31) bt_buffer[bt_buffer_back++] = (char)character;
    bt_rx_flag = 1;
    IFS1bits.U2RXIF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void)
{
    int character = U1RXREG & 0x00FF;
    rfid_buffer[rfid_buffer_back++] = (char)character;
    rfid_flag = 1;
    IFS0bits.U1RXIF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) //16ms
{
    tim_counter++;
    if (btn) debounce();
    else btn_prevstate = 0;
    if (tim_counter > 400) 
    {
        tim_counter = 0;
        tim_flag = 1;
    }
    IFS0bits.T1IF = 0;
}




/**********************************************************************
* Â© 2013 Microchip Technology Inc.
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Inc. (Microchip) licenses this software to you
* solely for use with Microchip dsPIC digital signal controller
* products. The software is owned by Microchip and is protected under
* applicable copyright laws.  All rights reserved.
*
* SOFTWARE IS PROVIDED AS IS.  MICROCHIP EXPRESSLY DISCLAIMS ANY
* WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
* PARTICULAR PURPOSE, OR NON-INFRINGEMENT. IN NO EVENT SHALL MICROCHIP
* BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL
* DAMAGES, LOST PROFITS OR LOST DATA, HARM TO YOUR EQUIPMENT, COST OF
* PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
* BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
* ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER SIMILAR COSTS.
**********************************************************************/

