#include<stdio.h>
#include<string.h>

#define len 140

int main(void)
{
	char str[] = "Pointers zijn zeer krachtig!\n";
	printf("%s", str);
	//not an & symbol since the array already return the address
	char * ptr = str;
	ptr += 14;
	*ptr = 'F';
	ptr += 2;
	*ptr = 'N';
	ptr++;
	*ptr = '\0';
	//the & symbol is necessary since str[15] return the value on the 16 position of the array
	ptr = &str[15];
	*ptr = 'U';
	printf("%s", str);
	getchar();
	return 0;
}