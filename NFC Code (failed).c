// PIC32MX320F128H Configuration Bit Settings
 
// 'C' source line config statements
 
// DEVCFG3
// USERID = No Setting
 
// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)
 
// DEVCFG1
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = XT             // Primary Oscillator Configuration (XT osc mode)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
 
// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (ICE EMUC2/EMUD2 pins shared with PGC2/PGD2)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)
 
 
#include <plib.h>
#include <xc.h>
#include <string.h>
#include <p32xxxx.h>
 
#define GetSystemClock()              (80000000ul)
#define XTAL                           80000000
#define GetPeripheralClock()          (GetSystemClock()/(1 << OSCCONbits.PBDIV))
#define DESIRED_BAUDRATE              (115200)      //The desired BaudRate


void PutCharacter(const char character)
{
  while (!UARTTransmitterIsReady(UART1))
    ;
 
  UARTSendDataByte(UART1, character);
 
  while (!UARTTransmissionHasCompleted(UART1))
    ;
}
char c[6000], q=0x55;
long count =0;
unsigned char wake[9]={0x00,0xff,0x03,0xfd,0xd4,0x14,0x01,0x17,0x00};
unsigned char firmware[9]={0x00,0x00,0xFF,0x02,0xFE,0xD4,0x02,0x2A,0x00};
unsigned char tag[11]={0x00,0x00,0xFF,0x04,0xFC,0xD4,0x4A,0x01,0x00,0xE1,0x00};//

void wakeup()
{
    unsigned int i;
    PutCharacter(q);
    for(i = 0; i<20000; i++);
    
    for(i = 0; i<9; i++)
    {
        PutCharacter(wake[i]);
       
    }
}

void read_tag()
{
    unsigned int i;
    for(i = 0; i<11; i++)
    {
        PutCharacter(tag[i]);
    } 
}

void firmware_version(void)
{
    unsigned char i;
    for(i=0;i<9;i++) //send command
    PutCharacter(firmware[i]);
}

void __ISR(_UART1_VECTOR, IPL2SOFT) IntUart1Handler(void)
{
  // Is this an RX interrupt?
  if (INTGetFlag(INT_SOURCE_UART_RX(UART1)))
    {
      // Echo what we just received.
      
 
      // Clear the RX interrupt Flag
      
      INTClearFlag(INT_SOURCE_UART_RX(UART1));
    }
 
}
void __ISR(_TIMER_23_VECTOR, IPL2SOFT) Timer23Handler(void) {
   
    mT23ClearIntFlag();
}

int main()
{
  mPORTESetPinsDigitalIn(BIT_0);	
  int z;

  // UART 1 Configuration
  UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY);
  UARTSetFifoMode(UART1, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
  UARTSetLineControl(UART1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
  int actual = UARTSetDataRate(UART1, GetPeripheralClock(), DESIRED_BAUDRATE);
  UARTEnable(UART1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
 
  // Configure UART1 RX Interrupt
  INTEnable(INT_SOURCE_UART_RX(UART1), INT_ENABLED);
  INTSetVectorPriority(INT_VECTOR_UART(UART1), INT_PRIORITY_LEVEL_2);
  INTSetVectorSubPriority(INT_VECTOR_UART(UART1), INT_SUB_PRIORITY_LEVEL_0);
 
//   //Clock Interrupts 
//   OpenTimer23(T23_ON | T23_PS_1_256, 0x1312d); //2HZ
//   mT23SetIntPriority(2);
//   mT23IntEnable(1);
  
   // configure for multi-vectored mode
  INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
 
  // enable interrupts
  INTEnableInterrupts();
  
    wakeup();
   
//    z=5000000;
//    while(z--);
//    c[count++] = UARTGetDataByte(UART1);
//    
//    z=5000000;
//    while(z--);   

    
    
    
  while (1){     
      
      c[count++] = UARTGetDataByte(UART1); 

     }
  return 0;
}
