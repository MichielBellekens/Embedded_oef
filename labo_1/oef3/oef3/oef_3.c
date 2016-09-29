#include<stdio.h>
#include<string.h>

#define len 140

int main(void)
{
	char text[len];
	printf("Please provide a sentence\n");
	fgets(text, len, stdin);
	//loop over all the characters
	for (int i = 0; i < strlen(text); i++)
	{
		//and the ascii code with the negation (=inverse) of the all zero except the 6 bit
		text[i] &= ~(1<<5);	// start from a 0 byte and shift a one to the left for 5 positions
		/*
		//alternate code
		text[i] &= 0xDF;
		*/
	}
	printf("the sentence is : %s", text);
	//the output can be explained by the asccii code of the space character which has only 1 1 bit in the code
	//if this bit is changed it results in a nulbyte which represent the end of the string/char array
	getchar();
	return 0;
}