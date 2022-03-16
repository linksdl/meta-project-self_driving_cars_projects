/*
 * Using Struct
 Created by Billy SHENG on 2021/11/8.

*/

#include <stdio.h>

struct Book{
    char title[50];
    char author[50];
    char subject[50];
    int book_id;
    double price;

} book = {"C programming language", "Running", "Programming", 123456, 39.99};


int main(void)
{
    printf("BooK Title: %s \nBook Author: %s \nBook Subject: %s \nBook ID: %d \nBook Price: %lf \n", book.title, book.author, book.subject, book.book_id, book.price);

}


