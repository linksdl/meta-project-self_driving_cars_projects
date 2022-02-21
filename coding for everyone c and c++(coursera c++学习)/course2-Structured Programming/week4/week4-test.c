//
// Created by Billy SHENG on 2021/11/8.
//

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

// compute the average value
double get_average_value(int size, int array[])
{
    int i;
    double sum = 0.0;
    for (i = 0; i < size; i++)
        sum += array[i];
    return (sum / size);
}

// compute the max value.
int get_max_value(int size, int array[])
{
    int max_value;
    int i;
    for (i = 0; i < size; i++)
    {
        if (array[i] > max_value)
        {
            max_value = array[i];
        }
    }
    return max_value;

}

int main()
{
    FILE *file;
    file = fopen("integers.txt", "r");
    int i = 0;
    int size = get_array_size(file);
    int array[size];
    for (i = 0; i < size; i++)
    {
        fscanf(file, "%d", &array[i]);
    }
    fclose(file);
    int data[size-1];
    for(i = 1; i< size; i++)
    {
        data[i-1] = array[i];
    }

    double average_value = get_average_value(size-1, data);
    int max_value = get_max_value(size-1, data);

    FILE *fp = NULL;
    fp = fopen("answer-hw3.txt", "w+");
    printf("The average value is: %lf \t the max value is: %d.", average_value,max_value);
    fprintf(fp, "The average value is: %lf \t the max value is: %d.", average_value,max_value);
    fclose(fp);

    return 0;
}