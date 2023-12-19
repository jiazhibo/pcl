#include <stdio.h>
#include "get.h"
int main (int argc, char** argv)
{
        int a = 100;
        int b = get ();
        int c = set (a);
        int d = get ();

        printf ("a=%d,b=%d,c=%d,d=%d\n",a,b,c,d);

        return 0;
}
