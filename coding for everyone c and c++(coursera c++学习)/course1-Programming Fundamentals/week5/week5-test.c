/*
 read them into an array and
 compute the average weight for the set of the elephant seals.

 Billy Sheng
 06/11/2021

*/

#include <stdio.h>
#include <stdlib.h>

int get_array_size(FILE *mfile)
{
    int size = 0;
    int total;

    if (mfile == NULL)
    {
        printf("Error, the file is empty.");
        exit(0);
    }
    while(fscanf(mfile, "%d", &total) != EOF)
    {
        size++;
    }
    printf("The file contains %d numbers.\n", size);
    rewind(mfile);
    return size;
}

double get_average_weight(int size, int weights_elephant_seals[])
{
    int i;
    double sum = 0.0;
    for (i = 0; i < size; i++)
        sum += weights_elephant_seals[i];
    return (sum / size);
}

int main(void)
{
    FILE *elephant_file;
    elephant_file = fopen("elephant seals.txt", "r");

    int i = 0;
    int array_size = get_array_size(elephant_file);
    int weights_elephant_seals[array_size];

    for (i = 0; i < array_size; i++)
    {
        fscanf(elephant_file, "%d", &weights_elephant_seals[i]);
    }
    double avg_weight = 0.0;
    avg_weight = get_average_weight(array_size, weights_elephant_seals);
    printf("The average weight of an elephant seal is: %lf", avg_weight);
    fclose(elephant_file);

    return 0;
}

