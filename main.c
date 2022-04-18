/*
 * File:   main.c
 * Author: Manuel Calzadillas Valles
 * Contact: menycalzadil@hotmail.com
 * Phone: +52 6142898877
 * 
 * Created on 26 de octubre de 2020, 20:31
 */
#include "config.h"

/*
 * PINOUT DESCRIPTION PIC16F1619 20 pin DIP
 * 1:  VDD          |-u-|   11: VSS  
 * 2:  RA5          |   |   12: RA0 ICSPDAT ESC1
 * 3:  RA4 ESC4     |   |   13: RA1 ICSPCLK ESC2
 * 4:  RA3 MCLR     |   |   14: RA2 ESC3
 * 5:  RC5 CH3 RC   |   |   15: RC0
 * 6:  RC4 CH2 RC   |   |   16: RC1
 * 7:  RC3 CH1 RC   |   |   17: RC2
 * 8:  RC6 CH4 RC   |   |   18: RB4 GYRO SDA
 * 9:  RC7          |   |   19: RB5
 * 10: RB7          |___|   20: RB6 GYRO SCLK
 */

//Cuatro esc con capacidad hasta 2000 unsigned 16 bits = 65535
unsigned esc1, esc2, esc3, esc4, ch1, ch2, ch3, ch4, TMR2H, count, TMR0H, tmrLoop;
//Un sensor con acc gyro y magnetometro 16 bits -32000 hasta 32000
signed int accx, accy, accz, gyrox, gyroy, gyroz, magx, magy, magz;
unsigned char start;
//Variables previas
struct previous{
    unsigned p1 :1;
    unsigned p2 :1;
    unsigned p3 :1;
    unsigned p4 :1;
}p;

//Variable para el voltaje
long voltaje;

void main(void) { 
    //Configuraciones iniciales
    //Oscilador interno
    OSCCONbits.IRCF = 0b1110;   //Configura el reloj interno a 8MHZ   
    OSCCONbits.SCS = 0;         //System clock select FINTOSC
    while(!OSCSTATbits.PLLR);   //Espera a que el PLL este listo para correr a 32
    //Interrupciones
    INTCONbits.GIE = 1;         //Habilita todas las interrupciones
    INTCONbits.IOCIE = 1;       //Habilita las interrupciones en los pines
    INTCONbits.PEIE = 1;
    PIE1bits.TMR2IE = 1;
    INTCONbits.TMR0IE = 1;
    IOCCP = 0b01111000;         // Habilita el interrupt on change para los pines
                                // RC3, RC4, RC5 y RC6 para leer el control remoto
    //Configuracion de timers
    //TMR0
    OPTION_REGbits.TMR0CS = 0;  //Se escoge la velocidad de 8 MHZ para tmr0
    OPTION_REGbits.PSA = 0;     //Prescaler asignado a TMR0
    OPTION_REGbits.PS = 2;      //Prescaler 1:8 1us
    //TMR1
    T1CONbits.TMR1CS = 0;
    T1CONbits.T1CKPS = 3;
    //TME3
    T3CONbits.TMR3CS = 0;
    T3CONbits.T3CKPS = 3;    
    //TMR5
    T5CONbits.TMR5CS = 0;
    T5CONbits.T5CKPS = 3;    
    //TMR2
    T2CLKCON = 0;
    T2CONbits.CKPS = 3;    
    T2CONbits.OUTPS = 0;
    T2HLTbits.PSYNC = 1;
    T2RST = 15;
    //Configurar pic como master en la comunicaci�n i2c
    SSP1CONbits.SSPEN = 1;
    SSP1CONbits.SSPM = 8;
    SSP1ADD = 19;                   //Velocidad de 400 KHZ  
    RB4PPS = 17;                    //Pin B4 SDA output
    RB6PPS = 16;                    //Pin B6 SCLK input
    SSPDATPPS = 12;                 //Pin B4 SDA input        
    SSPCLKPPS = 14;                 //Pin B6 SCLK input
    //Configuracion de puertos
    ANSELA = 0;                     //Puerto A digital
    ANSELB = 0;                     //Puerto B digital
    ANSELC = 0;                     //Puerto C digital
    TRISA = 0;                      // Todos los pines del puerto A se configuran como salidas
    TRISB = 0b01010000;             //Pin B4 y B6 como inputs
    ODCONBbits.ODB4 = 1;            // Open drain en pin B4
    ODCONBbits.ODB6 = 1;            // Open drain en pin B6
    TRISC = 0b01111000;             //Pin C3-C6 inputs para control remotos 0b0111 1000
    //Inicializaci�n de variables    
    start = 0;    
    //Ajustes iniciales de los sensores
    gyro_config();          // Se enciende el giroscopio y se configura
    LATCbits.LATC7 = 0;     // Pin C7 se apaga, es el LED indicador
    ch3 = 1000;             // Se inicializa el channel 3 para evitar errores
    while(1){
         // Se reinicia el contador del ciclo
        TMR0H = 0;
        TMR0 = 0;
        TMR0H = 0;
        // Condiciones iniciales
        if(!start){
            //Inicializaci�n de variables                           
            esc1 = esc2 = esc3 = esc4 = 1000;
            p.p1 = p.p2 = p.p3 = p.p4 = 0;    
            start = 1;
        }
        if(start == 1 && ch4 > 1900 && ch3 < 1050){
            start = 2;
        }
        // Condici�n que se debe cumplir para encender el dron
        if(start == 2 && ch4 < 1600 && ch3 < 1050){
            start = 3;
            LATCbits.LATC7 = 1;
        }      
        // Combinaci�n para apagar el dron, palanca izquierda hasta la izquierda
        // abajo y luego al centro
        if(start == 3 && ch4 < 1050 && ch3 < 1050){
            start = 0;
            LATCbits.LATC7 = 0;
        }   
        // Se toman datos del sensor
        read_sensor();  
        // Para evitar que los motores piten mientras est�n apagados
        esc1 = esc2 = esc3 = esc4 = ch3;
        tmrLoop = (TMR0H << 8) | TMR0;
        
        if(start < 3){                        
            PORTA |= 0b00010111;    // Se activa el puerto A para evitar que piten los ESC
            while((((TMR0H << 8) | TMR0) - tmrLoop) < 1000 || TMR0 < 0xE8);           
            PORTA &= 0b11101000;    // Se desactiva el puerto A
        }
        // Se arrancan motores
        else{
            if(esc1 < 1200)esc1 = 1200;     // To prevent propeller from stopping
            if(esc2 < 1200)esc2 = 1200;     // if esc value is less than 1200
            if(esc3 < 1200)esc3 = 1200;     // adjust it's value to 1200
            if(esc4 < 1200)esc4 = 1200;     // 
            PORTA |= 0b00010111;            // PORTA is set high in RA0, RA1, RA2 and RA4
            while((PORTA & 0b00010111) > 0){
                if((((TMR0H << 8) | TMR0) - tmrLoop) > esc1)PORTA &= 0b11111110;
                if((((TMR0H << 8) | TMR0) - tmrLoop) > esc2)PORTA &= 0b11111101;
                if((((TMR0H << 8) | TMR0) - tmrLoop) > esc3)PORTA &= 0b11111011;
                if((((TMR0H << 8) | TMR0) - tmrLoop) > esc4)PORTA &= 0b11101111;
            }
        }       
        while(TMR0H < 0x4E || TMR0 < 20);
    }    
    SLEEP();
}

void gyro_config(){
    i2c_write(GYRO, CTRL1, 0x0F);   //Gyro powerup
    i2c_write(GYRO, CTRL4, 0x90);   //Full Scale 500dps | Block data update
    i2c_write(ACCE, CTRL1, 0x57);   //ODR 100HZ   0b0101 0111 
    i2c_write(ACCE, CTRL4, 0x90);   //BDU 0b1001 0000
    i2c_write(MAG, 0, 0x18);        //0b0001 1000
    i2c_write(MAG, 1, 0x80);        //0b0001 1000 75 hz ODR
    i2c_write(MAG, 2, 0);           //0b0001 1000 75 hz ODR
}

void read_sensor(){
    gyrox = i2c_read(GYRO, 0xA8);   //0b1010 1000
    gyroy = i2c_read(GYRO, 0xAA);
    gyroz = i2c_read(GYRO, 0xAC);
    accx = i2c_read(ACCE, 0xA8);    //1010 1000
    accy = i2c_read(ACCE, 0xAA);    //1010 1000
    accz = i2c_read(ACCE, 0xAC);    //1010 1000
    magx = i2c_read(MAG, 0x83);
    magy = i2c_read(MAG, 0x85);
    magz = i2c_read(MAG, 0x87);
}

void i2c_write(unsigned char address, unsigned char subaddress, unsigned char data){    
    i2c_start();            
    i2c_write_byte(address);            
    if(nack())
        return; 
    i2c_write_byte(subaddress);    
    if(nack())
        return;        
    i2c_write_byte(data);    
    if(nack())
        return;    
    i2c_stop();
}

int i2c_read(unsigned char address, unsigned char subaddress){
    int low, high;
    i2c_start();
    i2c_write_byte(address);
    if(nack())
        return 0;
    
    i2c_write_byte(subaddress);
    if(nack())
        return 0;
    i2c_restart();
    i2c_write_byte(address | 1);
    if(nack())
        return 0;
    low = i2c_read_byte(0);
    high = i2c_read_byte(1);
    i2c_stop();
    if(address != MAG)
        return((high << 8) | low);
    else
        return((low << 8) | high);
}

int i2c_read_byte(unsigned char ack){
    int temp;    
    SSP1CON2bits.RCEN = 1;
    while(RCEN);
    temp = SSP1BUF;
    PIR1bits.SSP1IF = 0;
    
    SSP1CON2bits.ACKDT = ack;
    SSP1CON2bits.ACKEN = 1;
    while(SSP1CON2bits.ACKEN);
    PIR1bits.SSP1IF = 0;
    return temp;
}

int nack(){
    if(SSP1CON2bits.ACKSTAT){    //Si no contesta el sensor        
        i2c_stop();
        return 1;
    }  
    return 0;
}

void i2c_start(){ 
    SSP1CON2bits.SEN = 1;
    while((SSP1CON2bits.SEN)); 
    PIR1bits.SSP1IF = 0;
}

void i2c_restart(){
    SSPCON2bits.RSEN = 1;
    while(SSPCON2bits.RSEN);    
    PIR1bits.SSP1IF = 0;
}

void i2c_write_byte(unsigned char address){
    SSP1CON2bits.RCEN = 0;
    SSP1BUF = address;
    while(SSP1STATbits.BF);
    PIR1bits.SSP1IF = 0;
    if(SSP1CON1bits.WCOL)
        SSP1CON1bits.WCOL = 0;
}

void i2c_stop(){
    SSP1CON2bits.PEN = 1;
    while(SSP1CON2bits.PEN);
    PIR1bits.SSP1IF = 0;
}

void __interrupt() remote(){ 
    // Overflow del tmr 0 cada 255 us e incremento de TMR0H
    if(INTCONbits.T0IF)
    {
        TMR0H++;
        INTCONbits.T0IF = 0;
    }
    // Overflow del tmr 2 cada 255 us e incremento de TMR2H
    if(PIR1bits.TMR2IF){
        TMR2H++;
        PIR1bits.TMR2IF = 0;
    }  
    // Rutina para revisar si hubo alg�n cambio en los pines del puerto C
    if(INTCONbits.IOCIF){
        if(IOCCFbits.IOCCF3)
        {
            if(p.p1)
            {                
                T1CONbits.TMR1ON = 0;
                if(TMR1 < 2100)
                    ch1 = TMR1;
                IOCCN &= 0b11110111;
                IOCCP |= 0b00001000;
                p.p1 = 0;
            }
            else
            {                      
                T1CONbits.TMR1ON = 1;
                TMR1 = 0;                
                IOCCP &= 0b11110111;
                IOCCN |= 0b00001000;
                p.p1 = 1;
            }
        }
        //----------------- CHANNEL 2 ------------------  //
        if(IOCCFbits.IOCCF4)
        {
            if(p.p2)
            {
                T3CONbits.TMR3ON = 0;
                IOCCN &= 0b11101111;
                IOCCP |= 0b00010000;
                if(TMR3H < 9)
                    ch2 = ((unsigned)TMR3H<<8) | (unsigned)TMR3L;
                p.p2 = 0;
            }
            else
            {
                IOCCN |= 0b00010000;
                IOCCP &= 0b11101111;                               
                TMR3L = 0;
                TMR3H = 0;
                TMR3L = 0;
                T3CONbits.TMR3ON = 1; 
                p.p2 = 1;
            }
        }
        // ---------------- CHANNEL 3 THROTTLE ---------------
        if(IOCCFbits.IOCCF5)
        {
            if(p.p3)
            {
                T5CONbits.TMR5ON = 0;
                IOCCN &= 0b11011111;
                IOCCP |= 0b00100000;
                if(TMR5H < 9)
                    ch3 = ((unsigned)TMR5H << 8) | (unsigned)TMR5L;
                p.p3 = 0;
            }
            else
            {
                IOCCN |= 0b00100000;
                IOCCP &= 0b11011111;                
                TMR5L = 0;
                TMR5H = 0;
                T5CONbits.TMR5ON = 1;
                p.p3 = 1;
            }
        }
        // ------------- CHANNEL 4 YAW ------------------------
        if(IOCCFbits.IOCCF6)
        {
            if(p.p4)
            {
                T2CONbits.ON = 0;
                IOCCN &= 0b10111111;
                IOCCP |= 0b01000000;
                if(TMR2H < 9)
                    ch4 = (TMR2H<<8) | TMR2;
                p.p4 = 0;
            }
            else
            {
                IOCCN |= 0b01000000;
                IOCCP &= 0b10111111;                
                TMR2 = 0;
                TMR2H = 0;
                T2CONbits.ON = 1;
                p.p4 = 1;
            }
        }
        IOCCF = 0;
        INTCONbits.IOCIF = 0;         
    }         
}