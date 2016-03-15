// Copyright 2016 Lucas Walter
#include <xdo.h>

int main(int argc, char** argv)
{
  xdo_t* xdo = xdo_new(NULL);
  xdo_move_mouse(xdo, 50, 50, 0);
}
