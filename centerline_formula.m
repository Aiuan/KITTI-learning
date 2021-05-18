clear all;clc;
syms u v x y z;
syms x11 x12 x13 x14 x21 x22 x23 x24 x31 x32 x33 x34;
syms theta;

A = [u*x31-x11 u*x32-x12;v*x31-x21 v*x32-x22];
B = [x14-u*x34+x13*z-u*x33*z; x24-v*x34+x23*z-v*x33*z];

temp = A \ B;
x = temp(1);
y = temp(2);

theta = atan(x/z);