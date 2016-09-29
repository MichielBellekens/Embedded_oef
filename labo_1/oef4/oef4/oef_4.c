#include<stdio.h>
#include<string.h>

#define len 140

//The functions need to be declared at the top of the page

//The char array is a pointer by itself
void to_u(char* text)
{
	for (int i = 0; i < strlen(text); i++)
	{
		text[i] &= ~(1 << 5);
	}
	return;
}

int main(void)
{
	char text[len];
	printf("Please provide a sentence\n");
	fgets(text, len, stdin);
	to_u(text);
	printf("the sentence is : %s", text);
	getchar();
	return 0;


}

