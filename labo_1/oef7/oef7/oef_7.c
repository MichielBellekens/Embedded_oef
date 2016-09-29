#include<stdio.h>
#include"list.h"

int main(void)
{
	//create a pointer to the first node --> the address is used
	struct node * ptr = &node0;
	//As long as the current pointer address isn't NULL we iterate
	while (ptr != NULL)
	{
		//print the data of the node the pointer is pointing at
		printf("%s", ptr->data);
		//change the pointer to the next pointer address
		ptr = ptr->next;
	}
	//wait untill keypress to exit the application
	getchar();
	return 0;
}