// Functions implementation of config file
#include "config.h"

// Inicialización de PIC
void pic_init(void){
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
    //Configurar pic como master en la comunicación i2c
    SSP1CONbits.SSPEN = 1;          // Synchronous Serial Port enabled
    SSP1CONbits.SSPM = 8;           // Synchronous Serial Port Mode 0b1000
                                    // I2C master mode, clk = Fosc/(4*SSP1ADD + 1)
    SSP1ADD = 19;                   //Velocidad de 400 KHZ  
    RB4PPS = 17;                    //Pin B4 SDA output
    RB6PPS = 16;                    //Pin B6 SCLK input
    SSPDATPPS = 12;                 //Pin B4 SDA input        
    SSPCLKPPS = 14;                 //Pin B6 SCLK input
    //Configuracion de puertos
    ANSELA = 0;                     //Puerto A digital
    ANSELB = 0b00100000;            //Puerto B digital excepto PIN B5 para lectura de batería
    ANSELC = 0;                     //Puerto C digital
    TRISA = 0;                      // Todos los pines del puerto A se configuran como salidas
    TRISB = 0b01110000;             // Pin B4, B5 and B6 as inputs
    ODCONBbits.ODB4 = 1;            // Open drain en pin B4
    ODCONBbits.ODB6 = 1;            // Open drain en pin B6
    TRISC = 0b01111000;             //Pin C3-C6 inputs para control remotos 0b0111 1000
    
    // Analog to digital converter configuration
    ADCON0bits.ADON = 1;            // Analog to digital module enabled
    ADCON0bits.CHS = 11;            // Analog Channel 11 Selected pin B5
    ADCON1bits.ADPREF = 0;          // ADC positive voltage reference VDD selected
    ADCON1bits.ADCS = 2;            // Clock selected Fosc / 32 for 8 MHZ Fosc
    ADCON1bits.ADFM = 1;            // Right justified 3 bits of ADRESH and 8 bits of ADRESL
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
void reset_timer_loop(){
    TMR0H = 0;
    TMR0 = 0;
    TMR0H = 0;
}

void calculate_pid(void){
    // TODO
}

void balance_drone(){
    // TODO
}

void battery_compensation(){
    // Lectura del PIN B5 donde está conectada la batería
    ADCON0bits.GO = 1;          // Habilitar lectura del convertidor analogo a digital
    while(ADCON0bits.GO);       // Esperar a que termine la lectura y conversión
    PIR1bits.ADIF = 0;          // Clearing interrupt bit for analog finish iterruption
    // Assign value to variable voltage. Battery has 12.6 volts, it has a voltage
    // divider with R1 = 3 kOhms and Rout = 2 kOhms to get 5 volts in pin B5
    // 5 volts = 1024 in ADC
    // Minimum allowed voltage in battery: 9.6 V ADC = 780 or 800 to be safe
    voltage = (unsigned)((ADRESH << 8) | ADRESL);
    // Turn on LED if battery is low
    if(voltage < 810)
        LATCbits.LATC7 = 1;
    
    // Battery compensation
}
// Interrupts for reading the remote control
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
    // Rutina para revisar si hubo algún cambio en los pines del puerto C
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
