#include<stdio.h>
#include<string.h>
#define len 140

int main(void)
{
	char text[len];
	printf("Please provide a sentence.\n");
	fgets(text, len, stdin);
	printf("the sentence you entered is : %s", text);
	getchar();
	return 0;
}