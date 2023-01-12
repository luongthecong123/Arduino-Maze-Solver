/*
    The user manually inputs the path the robot took,
    this program will print out the shortest path for the maze.
    example input: L B L L L B S B L L B S L L E   (E is used to exit the input section)
*/
#include<stdbool.h>
#include <stdio.h>
#include <string.h>
int main()
{
   char path[100];
   int pathLength = 0;
   int pathIndex = 0;
   int i;
   int runner;
   printf("Enter path: ");

   while(true)
   {
       scanf("%s",&path[pathIndex]);
       //printf(" %s ",path[pathLength]);
       if (path[pathLength] == 'E') break;  // enter E to exit
       pathLength=pathLength + 1;
       pathIndex=pathIndex + 1;
   }

    printf("Path Length: %d \n",pathLength);
    printf("Final Path: ");
   for (i=0;i<pathLength;++i) //printing
   {
       printf("%c",path[i]);
   }
   printf(" \n");
////////////////////////////////////////////////
void del_replace(char path[],char replacement,int length,int position)
{
    int i=0;                   // replace 3 element in an array with 'replacement'
    int pointer=0;             //based on delete element in an array algorithm used in Data structure
    int a=length;
    pointer=position;
    for(i=pointer-1;i<=a-3;i++)
    {
        path[i+1]=path[i+3];
    }
    path[pointer-1]=replacement;
}
//////////////////////////////////////////////////// LBS -> R
int countB=0;
while(true){
    for (i=0;i<=pathLength-1;i++)
    {
        if (path[i] == 'B') countB=countB+1; //check if B is gone
    }
    printf("number of Bs: %d \n",countB);
    if (countB != 0)
    {


        for (i=0;i<=pathLength-1;i++)  //if current element ='B', check the previous & the following element
        {
            if (path[i] == 'B')
            {
                if (i==0) continue;  //skip this element if it's the first index of the array
                else if (i==pathLength-1) break; //exit loop if it's the last index of the array
                else
                {
                    if(path[i-1]=='L')
                    {
                        if(path[i+1]=='S')
                            {del_replace(path,'R',pathLength,i);countB=0;pathLength=pathLength-2;break;}
                        else if(path[i+1]=='R')
                            {del_replace(path,'B',pathLength,i);countB=0;pathLength=pathLength-2;break;}
                        else if(path[i+1]=='L')
                            {del_replace(path,'S',pathLength,i);countB=0;pathLength=pathLength-2;break;}
                    }
                    if(path[i-1]=='R')
                    {
                        if(path[i+1]=='L')
                           {del_replace(path,'B',pathLength,i);countB=0;pathLength=pathLength-2;break;}
                    }
                    if(path[i-1]=='S')
                    {
                        if(path[i+1]=='L')
                            {del_replace(path,'R',pathLength,i);countB=0;pathLength=pathLength-2;break;}
                        else if(path[i+1]=='S')
                            {del_replace(path,'B',pathLength,i);countB=0;pathLength=pathLength-2;break;}
                    }

                }
            }
        }
    for (i=0;i<=pathLength-1;i++)
    {
        printf("%c",path[i]);
    }   printf("\n");printf("pathLength= %d \n",pathLength);
    }
    else break;
}
printf("--------------------------- \n");
printf("SHORTEST PATH: ");

    for (i=0;i<=pathLength-1;i++)
    {
        printf("%c",path[i]);
    }
    printf(" \n");
    printf("--------------------------- \n");
/* Pseudo code on robot operations on second pass after
    receiving the shortest path array
second_pass_step = pathLength;
i=0;
while(true)
{
    if(sensors == 11111) { initiate_shortest_path(path[i]); i=i+1; if (i == second_pass_step - 1) break; }
}
void initiate_shortest_path(char instruction)
{
    if (instruction == 'L') left90();
    else if (instruction == 'R') right90();
    else if (instruction == 'S') {mini start up};
}
*/
}


