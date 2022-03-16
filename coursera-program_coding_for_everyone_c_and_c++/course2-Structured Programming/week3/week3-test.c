/*
 * C program to implement Bubble Sort on singly linked list
 * Created by Billy SHENG on 2021/11/8.
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* structure for a node */
struct Node
{
    int data;
    struct Node *next;
};

/* Function to insert a node at a linked list */
void insert_to_list(struct Node **start_ref, int data);

/* print nodes in a given linked list */
void print_list(struct Node *head);

/* bubble sort the given linked list */
void bubble_sort(struct Node *head);

/* swap data of two nodes a and b*/
void swap(struct Node *a, struct Node *b);

int main()
{
    int size = 100;
    int random_array[size];
    int i;
    // randomly generated set of 100 integers
    for (i = 0; i < size; i++)
    {
//        srand((unsigned)time(NULL));
        random_array[i] = rand() % 100 + 11;
    }

    // empty linked list
    struct Node *head = NULL;
    // Create linked list from the array random_array[]
    for(i = 0; i< size; i++)
    {
        insert_to_list(&head,random_array[i]);
    }

    // Print out the list before sorting
    printf("\nLinked list before sorting: \n");
    print_list(head);

    // sort the linked list using the bubble sort algorithm
    bubble_sort(head);

    // Print out the list after sorting
    printf("\nLinked list after sorting: \n");
    print_list(head);

    return 0;
}

void insert_to_list(struct Node **head, int data)
{
    struct Node *ptr1 = (struct Node*)malloc(sizeof(struct Node));
    ptr1->data = data;
    ptr1->next = *head;
    *head = ptr1;
}


void print_list(struct Node *head)
{
    struct Node *temp = head;
//    printf(" ");
    int i = 0;
    while (temp != NULL)
    {
        if (i % 5 == 0){
            printf("\n");
        }
        printf("%d ", temp->data);
        temp = temp->next;
        i++;
    }
}


void bubble_sort(struct Node *head)
{
    int swapped, i;
    struct Node *ptr1;
    struct Node *lptr = NULL;

    // Checking for empty list
    if (head == NULL)
        return;
    do
    {
        swapped = 0;
        ptr1 = head;
        while (ptr1->next != lptr)
        {
            if (ptr1->data > ptr1->next->data)
            {
                swap(ptr1, ptr1->next);
                swapped = 1;
            }
            ptr1 = ptr1->next;
        }
        lptr = ptr1;
    }
    while (swapped);
}

// swap data of two nodes a and b
void swap(struct Node *a, struct Node *b)
{
    int temp = a->data;
    a->data = b->data;
    b->data = temp;
}


