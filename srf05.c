/*
AUTHOR: ARDYA DIPTA NANDAVIRI
PROGRAM TO MEASURE THE DISTANCE USING SNAR SENSOR SRF05 FOR HEXAPOD ROBOT
Institut Teknologi Bandung, Indonesia
Last updated : June 13, 2009

*/
#include <mega32.h>
#include "registermega.h"      
#include <math.h>               

// DEFINE the port which will be used by the sonar sensor
//PORT ASSIGNMENT
#define PORT_SRF	PORTA       //CHOOSE THE PORT FOR SRF SENSOR
#define DDR_SRF		DDRA
#define PIN_SRF		PINA   
                      
//DELAY ASSIGNMENT
#define DELAY_HALF_SECOND    31250
#define DELAY_BEGIN				1			//warming up sensor (based on experience, this has to be there)
#define DELAY_STEP_SIGNAL		4			//~16 micro seconds
#define DELAY_CHECK 				15			//delay to check how fast the measuring is 1 cm, 60 us
#define DELAY_BETWEEN_MEASUREMENT	20000 //255 		//each measuring has to be at least 50 mS, waiting for the signal to return after being echo-ed
//#define DELAY_GPD				1000		//4u 

//local variables  
static unsigned char state_SRF;  

void inisialisasi_ultra (void)
{         
    //set state SRF
	state_SRF = 1;
	//set port trigger
	//triger for all sensors
	DDR_SRF.6 = 1;
	PORT_SRF.6 = 0;                       
	
	//set port input
	//Output pins 
	DDR_SRF.0 = 0;
	DDR_SRF.1 = 0;
	DDR_SRF.2 = 0;
	DDR_SRF.3 = 0;
	DDR_SRF.4 = 0;
	//DDR_SRF.5 = 0;
	
   //reset timer
	TCNT1 = 0;
	//set interrupt output compare A
	TIMSK |= (1 << OCIE1A);
	OCR1A = DELAY_BEGIN;
	//start timer dengan prescaler 64, 250 kHz, 4 uS/cyclej
	TCCR1B |= (1 << CS11) | (1 << CS10);
}  

   
//program to read the value of the sensor using interrupt
interrupt [TIM1_COMPA] void timer1_compa_isr(void)
{  
        static unsigned int sensortime;
    	static unsigned int sensortime_rising[7];
    	static unsigned int sensortime_distance[7];
        static bit done_SRF0;
        static bit done_SRF1;
        static bit done_SRF2;
        static bit done_SRF3;
        static bit done_SRF4;
        static bit done_SRF5;
        
	switch (state_SRF)
	{
		case 1://trigger SRF part 1
			//give high signal
			PORT_SRF.6 = 1;			
			
			//set status
			state_SRF = 2;		
			OCR1A += DELAY_STEP_SIGNAL;
		break;
		
		case 2://trigger SRF part 2
			PORT_SRF.6 = 0;				
			
			//set status
			state_SRF = 3;		
			OCR1A += DELAY_CHECK;
		break;
		
		case 3://read
			sensortime++;                  
			
			//=CHECK SRF 0---- additional:
			//Hardcoded here, how many IF is used depends on the number of the sensors
			if (done_SRF0 == 0)
			{
        			//port echo rising, sensortime is measured
        			if (PIN_SRF.0 == 1 && sensortime_rising[0] == 0)
        			{
        			        sensortime_rising[0] = sensortime;
        			}
        			
        			//when port echo is falling,measurement is done
        			else if (PIN_SRF.0 == 0 && sensortime_rising[0] != 0)
        			{
        				sensortime_distance[0] = sensortime - sensortime_rising[0];
        				
        				done_SRF0 = 1;
        			}           
        			
        			else  
        			{}
			}
			
			
			//=CEK SRF 1   
			if (done_SRF1 == 0)
			{
        			//when port echo rising, sensortime is measured
                    if (PIN_SRF.1== 1 && sensortime_rising[1] == 0)
        			{
        			        sensortime_rising[1] = sensortime;
        			}
        			
        			//when port echo kembali falling, measurement is done
        			else if (PIN_SRF.1 == 0 && sensortime_rising[1] != 0) 
        			{
        				sensortime_distance[1] = sensortime - sensortime_rising[1];
        				
        				done_SRF1 = 1;
        			}           
        			
        			else  
        			{}
            }              
           
                        //=CEK SRF 2   
			if (done_SRF2 == 0)
			{
        			//when port echo rising, sensortime is measured
                    if (PIN_SRF.2 == 1 && sensortime_rising[2] == 0)
        			{
        			        sensortime_rising[2] = sensortime;
        			}
        			
        			//when port echo falling, measurement is done
        			else if (PIN_SRF.2 == 0 && sensortime_rising[2] != 0)
        			{
        				sensortime_distance[2] = sensortime - sensortime_rising[2];
        				
        				done_SRF2 = 1;
        			}           
        			
        			else  
        			{}
            }  
                        
                    if (done_SRF3 == 0)
			        {
        			//when port echo rising, sensortime is measured
                                if (PIN_SRF.3 == 1 && sensortime_rising[3] == 0)
        			{
        			        sensortime_rising[3] = sensortime;
        			}
        			
        			//when port echo kembali falling, measurement is done
        			else if (PIN_SRF.3 == 0 && sensortime_rising[3] != 0)
        			{
        				sensortime_distance[3] = sensortime - sensortime_rising[3];
        				
        				done_SRF3 = 1;
        			}           
        			
        			else  
        			{}
            }  
                        
            if (done_SRF4 == 0)
			{
        			//when port echo rising, sensortime is measured
                                if (PIN_SRF.4 == 1 && sensortime_rising[4] == 0)
        			{
        			        sensortime_rising[4] = sensortime;
        			}
        			
        			//when port echo falling, measurement is done
        			else if (PIN_SRF.4 == 0 && sensortime_rising[4] != 0)
        			{
        				sensortime_distance[4] = sensortime - sensortime_rising[4];
        				
        				done_SRF4 = 1;
        			}           
        			
        			else  
        			{}
            }  
                        
//             if (done_SRF5 == 0)
// 			{
//         			//when port echo rising, sensortime is measured
//                                 if (PIN_SRF.5 == 1 && sensortime_rising[5] == 0)
//         			{
//         			        sensortime_rising[5] = sensortime;
//         			}
//         			
//         			//when port echo falling, measurement is done
//         			else if (PIN_SRF.5 == 0 && sensortime_rising[5] != 0)
//         			{
//         				sensortime_distance[5] = sensortime - sensortime_rising[5];
//         				
//         				done_SRF5 = 1;
//         			}           
//         			
//         			else  
//         			{}
//             }        

         //CHECK IF ALL OF THE MEASURING ARE DONE YET? THEN GIVE DELAY
        		if (done_SRF0 && done_SRF1 && done_SRF2 && done_SRF3 && done_SRF4) // && done_SRF5)
        		{         
                        //reset variable
        		        sensortime = 0;
        		        sensortime_rising[0] = 0;   
                        sensortime_rising[1] = 0;
                        sensortime_rising[2] = 0;
                        sensortime_rising[3] = 0;       
                        sensortime_rising[4] = 0;
                        //sensortime_rising[5] = 0;
				        
        		        done_SRF0 = 0;
        		        done_SRF1 = 0;
                        done_SRF2 = 0;
                        done_SRF3 = 0;
                        done_SRF4 = 0;
                        //done_SRF5 = 0;
        		                       
        		        //set status
        		        state_SRF = 4;              
        		        OCR1A += DELAY_BETWEEN_MEASUREMENT;               
        		}
        		else
        		{
        			OCR1A += DELAY_CHECK;
        		}
		break;
		        
		case 4://SRF measuring done
		//for gathering the data in centimeters
			SRF_Distance[0] = sensortime_distance[0];
			SRF_Distance[1] = sensortime_distance[1];
			SRF_Distance[2] = sensortime_distance[2];
			SRF_Distance[3] = sensortime_distance[3];
			SRF_Distance[4] = sensortime_distance[4];
			//SRF_Distance[5] = sensortime_distance[5];
			
			//set status
			state_SRF = 1;			
			OCR1A = DELAY_BEGIN;
		break;
	}
}

 