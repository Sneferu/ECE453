//Tommy Wiesler
//Nov. 6 2017
//G Code Sender
//Takes a file of G Code cmds, and sends it line by line to registers on board 
//No Ticket Required

#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <fcntl.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "../include/ece453.h" //change to fit path

//Defined in ece453.h
/*
#define GCODE_DIRX_REG "/sys/kernel/ece453/gCode30"
#define GCODE_STEPX_REG "/sys/kernel/ece453/gCode74"
#define GCODE_DIRY_REG "/sys/kernel/ece453/gCode118"
#define GCODE_STEPY_REG "/sys/kernel/ece453/gCode1512"
#define GCODE_DIRZ_REG "/sys/kernel/ece453/gCode1916"
#define GCODE_STEPZ_REG "/sys/kernel/ece453/gCode2320"
#define GCODE_2724_REG "/sys/kernel/ece453/gCode2724"
#define GCODE_STEPDIV_REG "/sys/kernel/ece453/gCodeStepDiv"
#define GCODE_STEP_FACTOR 500
//Done with control reg#define GCODE_TRMT_REG "/sys/kernel/ece453/gCodeRdy"
*/

//Below defined in ws2812b.c might not need, or need to redefine
#define KEY0  (1 << GPIO_IN_BUTTONS_BIT_NUM)
#define GCODEDONE  IRQ_GCODE_DONE_MASK
#define SIG_TEST 44 /* we define our own signal, hard coded since SIGRTMIN is different in user and in kernel space */ 

#define PID "/sys/kernel/ece453/pid"
//Masks for step division
//bit 31 = 1 for y, 0 for x
#define MORE_X_STEPS_MASK 0x7FFFFFFF
#define MORE_Y_STEPS_MASK 0x80000000

//Defined elsewhere #define GCODE_IRQ_NUM 
bool busy;

void receiveData(int n, siginfo_t *info, void *unused)
{
	printf("data received\n\r");
	printf("%d\n\r", info->si_int);
	 if( info->si_int == KEY0)
  {
		printf("KEY0 Pressed\n\r");
    //key_pressed = 0;
    busy = false;
  }
  if( info->si_int == GCODEDONE)
  {
    busy = false;
  }

}

int set_pid(void)
{

	char buf[10];
	int fd = open(PID, O_WRONLY);
	if(fd < 0) {
		perror("open");
		return -1;
	}
	sprintf(buf, "%i", getpid());
	if (write(fd, buf, strlen(buf) + 1) < 0) {
		perror("fwrite"); 
		return -1;
	}
  close(fd);
  return 0;
}

//*****************************************************************************
//*****************************************************************************
int clear_pid(void)
{

	char buf[10];
	int fd = open(PID, O_WRONLY);
	if(fd < 0) {
		perror("open");
		return -1;
	}
	
 memset(buf,0,10);
 if (write(fd, buf, strlen(buf) + 1) < 0) {
		perror("fwrite"); 
		return -1;
	}
  close(fd);
  return 0;
}

//finds index of specified char
//returns index, or -1 if not found
int findCharIndex(char line[], char find)
{
	int charloc;
	int i;
	
	charloc = -1;
	//find location of char
	for(i = 0; i < 32; i ++)
	{
		if(line[i] == find)
		{
			charloc = i;
		}
	}
	return charloc;
}

//finds the absolute distance val specified after charIndex
double findAbsoluteDist(char line[], int charIndex)
{
	char sub[8];
	char buff[32];
	double absLoc;
	int i;
	
	int subLen = 6; //set to 7 if neg
	strcpy(buff, line);
	memset(sub, '\0', sizeof(sub));
	//each number is 6 chars long, or 7 with neg val
	if(buff[charIndex+ 1] == '-')
	{
		subLen = 7;
	}
	strncpy(sub, buff + charIndex + 1, subLen);
	
	//sub[subLen] = '\0';
	absLoc = atof(sub);
	return absLoc;
}

//overwrites part of line with sub, starting at start
//stops overwriting when line has a space or null char, or sub has a null char
void replaceSubstring(char line[], char sub[], int start)
{
	int i = 0;
	while(line[start + i] != ' ' && line[start + i] != '\0' && sub[i] != '\0')
	{
		line[start + i] = sub[i];
		i ++;
	}
}

//calcs the ratio of X:Y (or Y:X) steps
//input 
//output bits 30-0 are the ratio
//if bit 31 is a 0, X has more steps
//if bit 31 is a 1, Y has more steps
//if all is 0, no difference(45 deg angle) or only X or Y steps
int step_div(double xDist, double yDist)
{
	double ratio;
	int step_div;
	if(fabs(xDist) > fabs(yDist))
	{
		ratio = fabs(xDist)/fabs(yDist);
		//printf("ratio:%f\n\r", ratio);
		step_div =  (int) ratio;
		//printf("ratioInt:%d\n\r", step_div);
		step_div = step_div & MORE_X_STEPS_MASK;
		//printf("stepDiv:%d\n\r", step_div);
	}
	else
	{
		ratio = fabs(yDist)/fabs(xDist);
		//printf("ratio:%f\n\r", ratio);
		step_div = (int) ratio;
		//printf("ratioInt:%d\n\r", step_div);
		step_div = step_div | MORE_Y_STEPS_MASK;
		//printf("stepDiv:%d\n\r", step_div);
	}	
	return step_div;
	
}


//reads a string char by char, and writes 0x00 to 
//every char at and after the new line
void nullifyString(char* str, int size)
{
	int i;
	bool nlFound = false;
	
	for(i = 0; i < size; i ++)
	{
		if(nlFound)
		{
			str[i] = 0x00;
		}
		else if(str[i] == 0x0A || str[i] == 0x0D)
		{
			nlFound = true;
			str[i] = 0x00;
		}
	}
}

int main(int argc, char* argv[])
{
	struct sigaction done_sig;
	
	FILE *fd;
	char line [32];
	int eof;
	int val; //value to make that gets loaded to regs
	int j;
	int xIndex;
	int yIndex;
	int zIndex;
	double prevX;
	double prevY;
	double prevZ;
	double absDistX;
	double absDistY;
	double absDistZ;
	double incX;
	double incY;
	double incZ;
	double dblXInc;
	double dblYInc;
	double dblZInc;
	int step_ratio;
	done_sig.sa_sigaction = receiveData;
	done_sig.sa_flags = SA_SIGINFO; //might need to set differently
	sigaction(SIG_TEST, &done_sig, NULL);//might need to set differently
	// Configure the IP module 
  set_pid();
	if(argc != 2)
	{
		//error
		exit(1);
	}
	
	fd = fopen(argv[1], "r");
	if(fd == NULL)
	{
		//error
		exit(1);
	}
	
	
	//read file line by line and write to reg
	prevX = 0;
	prevY = 0;
	prevZ = 0;
	eof = 0;
	printf("Initial: GPIO_OUT_REG:%d\n\r", ece453_reg_read(GPIO_OUT_REG));
	while(!eof)
	{
		
		//get line from file
		if(!fgets(line, 32, fd))
		{
			eof = 1;
			break;
		}
		
		//write null chars to unused
		nullifyString(line, 32);
		
		//find X, Y
		xIndex = findCharIndex(line, 'X');
		yIndex = findCharIndex(line, 'Y');
		zIndex = findCharIndex(line, 'Z');
		
		//replace absolute distances with incremental ones
		if(xIndex >= 0)
		{
			absDistX = findAbsoluteDist(line, xIndex);
			dblXInc = absDistX - prevX;
			prevX = absDistX;
			
			
		}
		if(yIndex >= 0)
		{
			absDistY = findAbsoluteDist(line, yIndex);
			dblYInc = absDistY - prevY;
			prevY = absDistY;
			
		}
		if(zIndex >= 0)
		{
			absDistZ = findAbsoluteDist(line, zIndex);
			dblZInc = absDistZ - prevZ;
			prevZ = absDistZ;
			
		}
		if(yIndex >= 0 && xIndex >= 0)
		{
			step_ratio = step_div(dblXInc, dblYInc);
		}
		else
		{
			step_ratio = 0;
		}
		

		//fill registers, each one 4bytes wide

		ece453_reg_write(GCODE_STEPX_REG, (int)(dblXInc*GCODE_STEP_FACTOR));
		ece453_reg_write(GCODE_DIRX_REG, dblXInc>0?1:0);
		ece453_reg_write(GCODE_STEPY_REG, (int)(dblYInc*GCODE_STEP_FACTOR));
		ece453_reg_write(GCODE_DIRY_REG, dblYInc>0?1:0);
		ece453_reg_write(GCODE_STEPZ_REG, (int)dblZInc*GCODE_STEP_FACTOR);
		ece453_reg_write(GCODE_DIRZ_REG, dblZInc>0?1:0);
		//ece453_reg_write(GCODE_2724_REG, val);
				
		ece453_reg_write(GCODE_STEPDIV_REG, step_ratio);
		// enable reception of a signal when Gcode finishes interpretation
    ece453_reg_write(IM_REG,  GCODEDONE | KEY0);
		printf("%d is IM_REG", ece453_reg_read(IM_REG));
		//write trmt ready bit
		ece453_reg_write(CONTROL_REG, 0x0001);
		busy = true;
		//ece453_reg_write(CONTROL_REG, 0x0000);
		//wait until interpreter not busy
		while(busy)
		{
			//printf("IRQ_REG:%d\n\r", ece453_reg_read(IRQ_REG));
			//printf("GPIO_OUT_REG:%d\n\r", ece453_reg_read(GPIO_OUT_REG));
      sleep(100);
      printf("woke up\n");
		}
		printf("not busy\n\r");
		
		//lower transmit bit
		ece453_reg_write(CONTROL_REG, 0x0000);
		//temporarily disable interrupts
    ece453_reg_write(IM_REG, 0);
		
	}
	clear_pid();
	//close file
	fclose(fd);
}

