#include <zephyr/kernel.h>
#include "Init.h"

int main(void)
{
	Init();

	while (1) 
	{
		k_msleep(1000);
	}
}
