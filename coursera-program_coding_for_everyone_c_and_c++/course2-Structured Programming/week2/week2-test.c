//
// Created by Billy SHENG on 2021/11/8.
//


#include <stdio.h>
#include <stdlib.h>

// business department
typedef enum
{
    HR,
    SALES,
    RESEARCH,
    SOFTWARE,
    EXECUTIVE
} Department;


typedef struct {
    Department  department;
    int annual_salary;
    unsigned int social_security_number;
} Employee;

int get_annual_salary(Department department){
    int salary;
    switch (department) {
        case HR: salary = 30000; break;
        case SALES: salary = 35000; break;
        case RESEARCH: salary = 50000; break;
        case SOFTWARE: salary = 60000; break;
        case EXECUTIVE: salary = 100000; break;
        default: salary = -1;
    }
    return salary;
}


int main(void )
{
    int employee_count = 10;
    Employee employees[employee_count];
    for (int i=0; i < employee_count; i++)
    {
        employees[i].department = (Department) rand() % 5;
        employees[i].annual_salary = get_annual_salary(employees[i].department);
        employees[i].social_security_number = (unsigned int) &employees[i];

        printf("Employee # %d: \t Department: %d \t Annual Salary: %d \t SSN: %lu \n", i, employees[i].department, employees[i].annual_salary, employees[i].social_security_number);
    }

    return 0;
}