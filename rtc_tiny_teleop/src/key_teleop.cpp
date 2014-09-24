#include "rtc_tiny_teleop/com.h"

int main(void)
{
	system("clear");	
	cout<<"********* Program for DC motor control via AWD10-24 driver *********\n\n";
	RTK_Tiny *robot = new RTK_Tiny();

	cout<<"\t Press Esc for exit.\n\n"
		<<"\t\t\tData transmission\n"
		<<"\t|Output\t\t\t\t|Input\n"
		<<"Value\t|-------------------------------|---------------------------\n"
		<<"\t|Size\t|Data\t\t\t|Size\t|Data\n"
		<<"--------|-------|-----------------------|-------|-------------------\n";

	struct termios old_consol_opt = set_keypress();
	
	bool enter = true;
	int key = 0;
	//robot->echo();
	//robot->echo(1);	
	short value1 = 0;//robot->get_value();	 
	short value2 = 0;//robot->get_value(1);
	while(enter)
	{			
		key = getchar();
		cout<<"\b";
		switch (key)
		{	
		case 87:
		case 119:
			value1 -= STEP;
			value2 += STEP;
			break;
		case 65:
		case 97:
			value1 -= STEP;
			value2 -= STEP;			
			break;
		case 68:
		case 100:
			value1 += STEP;
			value2 += STEP;
			break;
		case 83:
		case 115:
			value1 += STEP;
			value2 -= STEP;
			break;
		case 32:
			value1 = 0x0;
			value2 = 0x0;
			break;
		case 27: enter = false;
		default: break;
		}
		robot->move(value1, value2);
	}
	
	reset_keypress(&old_consol_opt);
	delete robot;
	return 1;
}