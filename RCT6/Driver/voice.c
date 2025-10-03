#include "voice.h"

void voice_init(void){
	printf("<V>4"); //设置音量 1-4
	vTaskDelay(500);
	printf("<S>3"); //设置语速 1-3
	vTaskDelay(500);
	//printf("美团外卖，您的外卖将于3分钟后到达!");  
	printf("请装入物料并关闭仓门");  
}
