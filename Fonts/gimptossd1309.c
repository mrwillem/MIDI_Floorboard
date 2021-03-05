#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
	FILE * infile;
	FILE * outfile;
	char c;	
	ssize_t linelength;
	char  *linebuffer=NULL;
	size_t len;
	int eof;
	int strc;
	int i,j;
	len=0;
	char **buffer;
	int bufferlines=1;
	int datalength;
	int value;
	if(argc < 2)
	{
		printf("Please provide infile and outfile on the command line\n");
		return 1;
	}
	else
	{
		infile = fopen(argv[1], "r");
		if (infile == NULL)
		{
			printf("Error opening input file %s\n", argv[0]);
 			return 1;
		}
		outfile = fopen(argv[2], "w");
		if ( outfile == NULL )
		{
			printf("Error opening output file %s\n", argv[1]);
			return 1;
		}
		eof=0;
		buffer=NULL;
		/*if(buffer == NULL)
		{
			printf("Error: No Space for buffer\n");
			return 1;
		}*/
		while(eof !=2)
		{
			linelength=getline(&linebuffer, &len, infile);
			if(linelength == -1)
			{
				printf("End of file\n");
				eof=2;
			}
			switch(eof)
			{
				case 0:
					if(strstr(linebuffer, "header_data[") != NULL)
					{

						printf("found a match\n");
					 	eof=1;
					}
					break;
				case 1:
					if(strchr(linebuffer, '}')==NULL)
					{
						buffer=(char **) realloc(buffer, (bufferlines*sizeof(char*)));
						if(buffer == NULL)
						{
							printf("No storage for buffer\n");
							return 1;
						}
						datalength=linelength;
						buffer[bufferlines-1]= (char*) malloc((linelength+1)*sizeof(char));
						if(buffer[bufferlines-1]==NULL)
						{
							printf("No storage for linebuffer\n");
							return 2;
						}
						i=0;
						j=0;
						c=linebuffer[i];
						while((c!='\n') && (c != '\0'))
						{
							c=linebuffer[i];
							if((c == 48) || (c == 49))
							{
								buffer[bufferlines-1][j]=c;
								j++;
							}
							i++;
						}
						buffer[bufferlines-1][j]='\0';
						bufferlines++;
					}
					break;
			}

		}
		datalength=strlen(buffer[0]);
		for(j=0; j<(bufferlines-1); j+=8)
		{
			for(i=0;i<datalength;i++)
			{
				value=((buffer[j][i])-48)+
					(2*(buffer[j+1][i]-48))+
					(4*(buffer[j+2][i]-48))+
					(8*(buffer[j+3][i]-48))+
					(16*(buffer[j+4][i]-48))+
					(32*(buffer[j+5][i]-48))+
					(64*(buffer[j+6][i]-48))+
					(128*(buffer[j+7][i]-48));
				printf("0x%02X,", value);

			}
			printf("\n");
		}	

		for(i=0; i<bufferlines-1; i++)
		{
			printf("Line %d, %s\n", i, buffer[i]);
			free(buffer[i]);
		}
		free(buffer);
	
	}
	free(linebuffer);
}
