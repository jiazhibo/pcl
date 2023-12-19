#include <stdio.h>
#include "get.h"

static int x=0;
int get ()
{
        printf ("get x=%d\n", x);
        return x;
}

int set (int a)
{
        printf ("set a=%d\n", a);
        x = a;
        return x;
}
