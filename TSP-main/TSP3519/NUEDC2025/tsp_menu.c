#include "tsp_menu.h"
#include "tsp_common_headfile.h"


typedef enum {
    keyUP,
    keyDOWN,
    keySTILL
} QESDir;

QESDir Scroll;

const unsigned char MenuCursor16x16[]={
    0x00,0x00,0x00,0x01,0x00,0x03,0x00,0x07,
    0x00,0x0F,0xFE,0x1F,0xFE,0x3F,0xFE,0x7F,
    0xFE,0x3F,0xFE,0x1F,0x00,0x0F,0x00,0x07,
    0x00,0x03,0x00,0x01,0x00,0x00,0x00,0x00,
};


uint8_t menu_item0[8][20]=
{
	"1.OpenLoop",
	"2.CloseLoop",
	"3.ParaSet",
	"4.PID", 
	"5.",
	"6.",
	"7.",
	"8.Battery"	
};

// extern float kp_motor = 1.0f; // 电机控制的比例系数
// extern float ki_motor = 0.0f;
// extern float kd_motor = 0.0f; // 电机控制的微分系数
// extern float kp_servo = 0.0f; // 舵机控制的比例系数
// extern float ki_servo = 0.0f;
// extern float kd_servo = 0.0f; // 舵机控制的微分系数
extern uint8_t speed; // 目标速度
extern uint8_t RES_value;
uint8_t change=0;		// 0: no change; 1: increase; 2: decrease


void tsp_show_arrow(uint8_t x){
	tsp_tft18_show_str(0, 1, "  ");
	tsp_tft18_show_str(0, 2, "  ");
	tsp_tft18_show_str(0, 3, "  ");
	tsp_tft18_show_str(0, 4, "  ");
	tsp_tft18_show_str(0, 5, "  ");
	tsp_tft18_show_str(0, 6, "  ");
	tsp_tft18_show_str(0, 7, "  ");
	tsp_tft18_show_str(0, x, "->");
}
uint8_t tsp_menu_loop(void)
{
	uint8_t ItemNumber=0;
	uint8_t CmdIdx=0, CmdOk=0;
	tsp_tft18_clear(BLACK);
    
	tsp_tft18_show_str(32, 0, menu_item0[0]);
	tsp_tft18_show_str(32, 1, menu_item0[1]);
	tsp_tft18_show_str(32, 2, menu_item0[2]);
	tsp_tft18_show_str(32, 3, menu_item0[3]);
	tsp_tft18_show_str(32, 4, menu_item0[4]);
	tsp_tft18_show_str(32, 5, menu_item0[5]);
	tsp_tft18_show_str(32, 6, menu_item0[6]);
	tsp_tft18_show_str(32, 7, menu_item0[7]);

	show_menu_cursor(ItemNumber, WHITE);
	
	Scroll = keySTILL;


	while (1)
	{
		//TODO:修改按键
		Scroll = keySTILL;
		if (!S1())
		{
			delay_1ms(2);
			if (!S1())
				Scroll = keyUP;
			while(!S1());
		}
		if (!S2())
		{
			delay_1ms(2);
			if (!S2())
				Scroll = keyDOWN;
			while(!S2());
		}
        
		if (keyUP == Scroll)		// cursor move up
		{
			if (ItemNumber>0)
			{
				show_menu_cursor(ItemNumber--, BLACK);
				show_menu_cursor(ItemNumber, WHITE);
			}
		}
		if (keyDOWN == Scroll)	// cursor move down
		{
			if (ItemNumber<7)
			{
				show_menu_cursor(ItemNumber++, BLACK);
				show_menu_cursor(ItemNumber, WHITE);
			}
		}

		if (S0())			// push button pressed        
		{
			delay_1ms(10);	// de-jitter
			if (S0())
			{
				while(S0());
				return ItemNumber;
			}
		}
	}
}

void show_menu_cursor(uint8_t ItemNumber, uint16_t color)
{
	uint8_t i,j;
	uint8_t temp1, temp2;

	for(i=0; i<16; i++)
	{
		tsp_tft18_set_region(12, ItemNumber*16+i, 28, ItemNumber*16+i);
		temp1 = MenuCursor16x16[i*2];
		temp2 = MenuCursor16x16[i*2+1];
		for(j=0; j<8; j++)
		{
			if(temp1&0x01)
				tsp_tft18_write_2byte(color);
			else
				tsp_tft18_write_2byte(BLACK);
			temp1 >>= 1;
		}
		for(j=0; j<8; j++)
		{
			if(temp2&0x01)
				tsp_tft18_write_2byte(color);
			else
				tsp_tft18_write_2byte(BLACK);
			temp2 >>= 1;
		}
	}
}


void para_set()
{
	char value_str[4];
	uint8_t item=0, item_t=0;

	tsp_tft18_clear(BLACK);
	tsp_tft18_set_region(0, 0, TFT_X_MAX-1, TFT_Y_MAX-1);
	tsp_tft18_show_str_color(0, 0, "-Smartcar PID Demo--", BLUE, YELLOW);
	tsp_tft18_show_str(24, 1, "TargetV:");
	tsp_tft18_show_str(32, 2, "Kp M: ");
	tsp_tft18_show_str(32, 3, "Ki M: ");
	tsp_tft18_show_str(32, 4, "Kd M: ");
	tsp_tft18_show_str(32, 5, "Kp S: ");
	tsp_tft18_show_str(32, 6, "Ki S: ");
	tsp_tft18_show_str(32, 7, "Kd S: ");
	
	sprintf(value_str, "%03d", speed);
	tsp_tft18_show_str(88, 1, value_str);
	tsp_tft18_show_str(0, 1, "->");

	sprintf(value_str, "%0.1f", kp_motor);
	tsp_tft18_show_str(88, 2, value_str);

	sprintf(value_str, "%0.1f", ki_motor);
	tsp_tft18_show_str(88, 3, value_str);

	sprintf(value_str, "%2.1f", kd_motor);
	tsp_tft18_show_str(88, 4, value_str);

	sprintf(value_str, "%0.3f", kp_servo);
	tsp_tft18_show_str(88, 5, value_str);

	sprintf(value_str, "%0.3f", ki_servo);
	tsp_tft18_show_str(88, 6, value_str);

	sprintf(value_str, "%2.3f", kd_servo);
	tsp_tft18_show_str(88, 7, value_str);
	item = item_t = 0;

	while(1)
	{
		Scroll = keySTILL;
		if (!S1())
		{
			delay_1ms(2);
			if (!S1())
			{
				Scroll = keyUP;
				if(item>0)
				{
				  item--;
				  item_t = item;
				}
			}
			while(!S1());
		}
		if (!S2())
		{
			delay_1ms(2);
			if (!S2())
			{
				Scroll = keyDOWN;
				if(item<6)
				{
				  item++;
				  item_t = item;
				}
			}
			while(!S2());
		}
		if(Scroll != keySTILL)
		{
			switch(item)
			{
			case 0:
				tsp_show_arrow(1);
				break;
			case 1:
				tsp_show_arrow(2);
				break;
			case 2:
				tsp_show_arrow(3);
				break;
			case 3:
				tsp_show_arrow(4);
				break;
			case 4:
				tsp_show_arrow(5);
			    break;
			case 5:
				tsp_show_arrow(6);
				break;
			case 6:
				tsp_show_arrow(7);
				break;
			default:
				break;
			}
		}

		change = 0;
		delay_1ms(10);
		switch(item)
		{
		case 0:		// Speed
			if(change == 1)
			{
				speed += 1;
            if(speed > 40)
			speed = 40;
			}
			else if(change == 2)
			{
				if(speed > 5 )
				speed -= 1;
			}
			break;
		case 1:		// Kp Motor
			if(change == 1)
			{
				kp_motor += 0.1;
            if(kp_motor > 9.0)
              kp_motor = 9.0;
			}
			else if(change == 2)
			{
				if(kp_motor > 2.1 )
				  kp_motor -= 0.1;
			}
			break;
		case 2:		// Ki Motor
			if(change == 1)
			{
				if(ki_motor < 1)
				  ki_motor += 0.1;
			}
			else if(change == 2)
			{
				ki_motor -= 0.1;
            if(ki_motor < 0.0 )
               ki_motor = 0.0;
			}
			break;
		case 3:		// Kd Motor
			if(change == 1)
			{
				if(kd_motor < 15)
				  kd_motor += 0.5;
			}
			else if(change == 2)
			{
				kd_motor -= 0.5;
            if(kd_motor < 0.0 )
               kd_motor = 0.0;
			}
			break;
		case 4:		// Kp Servo
			if(change == 1)
			{
				kp_servo += 0.01;
            if(kp_servo > 9.0)
              kp_servo = 9.0;
			}
			else if(change == 2)
			{			
				kp_servo -= 0.01;
			if(kp_servo < 0.0 )
			   kp_servo = 0.0;
			}
			break;
		case 5:		// Ki Servo
			if(change == 1)
			{
				if(ki_servo < 1)
				  ki_servo += 0.01;
			}
			else if(change == 2)
			{
				ki_servo -= 0.01;
            if(ki_servo < 0.0 )
               ki_servo = 0.0;
			}
			break;
		case 6:		// Kd Servo
			if(change == 1)
			{
				if(kd_servo < 15)
				  kd_servo += 0.01;
			}
			else if(change == 2)
			{
				kd_servo -= 0.01;
            if(kd_servo < 0.0 )
               kd_servo = 0.0;
			}
			break;
		default:		
			break;
		}
		
		if(change != 0)
		{
			sprintf(value_str, "%03d", speed);
			tsp_tft18_show_str(88, 1, value_str);

			sprintf(value_str, "%0.1f", kp_motor);
			tsp_tft18_show_str(88, 2, value_str);
			
			sprintf(value_str, "%0.1f", ki_motor);
			tsp_tft18_show_str(88, 3, value_str);

			sprintf(value_str, "%0.1f", kd_motor);
			tsp_tft18_show_str(88, 4, value_str);

			sprintf(value_str, "%0.3f", kp_servo);
			tsp_tft18_show_str(88, 5, value_str);

			sprintf(value_str, "%0.3f", ki_servo);
			tsp_tft18_show_str(88, 6, value_str);

			sprintf(value_str, "%0.3f", kd_servo);
			tsp_tft18_show_str(88, 7, value_str);
		}
		// sprintf(value_str, "RES: %3d", RES_value);
		// tsp_tft18_show_str(0, 6, value_str);
		if(S0())
		{
			tsp_tft18_show_str(0, 1, "  ");
			tsp_tft18_show_str(0, 2, "  ");
			tsp_tft18_show_str(0, 3, "  ");
			tsp_tft18_show_str(0, 4, "  ");
			tsp_tft18_show_str(0, 5, "  ");
			tsp_tft18_show_str(0, 6, "  ");
			tsp_tft18_show_str(0, 7, "  ");
			break;
         while(S0()) {}
		}
	}
}
