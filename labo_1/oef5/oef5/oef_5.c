#include<stdio.h>
#include<string.h>

#define len 140

//The functions need to be declared at the top of the page

//The char array is a pointer by itself
void to_I(char* text)
{
	for (int i = 0; i < strlen(text); i++)
	{
		//the or is used to force the 6th bit to 1
		//this adds 32 to the ascii code which is the difference between CAPS and regular
		text[i] |= (1 << 5);
	}
	return;
}

int main(void)
{
	char text[len];
	printf("Please provide a sentence\n");
	fgets(text, len, stdin);
	to_I(text);
	printf("the sentence is : %s", text);
	getchar();
	return 0;


}

