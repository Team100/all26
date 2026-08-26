package org.team100.lib.kinematics.five_bar;

/**
 * Jacobian for five-bar planar linkage.
 * 
 * Adapted from http://charm.stanford.edu/ME327/JaredAndSam
 * 
 * Which is itself adapted from "The Pantograph Mk-II: A Haptic Instrument"
 * Hayward, 2005 https://cim.mcgill.ca/~haptic/pub/GC-QW-VH-IROS-05.pdf
 */
public class FiveBarJacobian {
    
    /*
    matlab code:

    d = norm(P2-P4);
b = norm(P2-Ph);
h = norm(P3-Ph);

del1_x2 = -a1*sin(t1);  %NOTE: THE AUTHOR FORGOT NEGATIVE SIGN IN THE PAPER
del1_y2 = a1*cos(t1);
del5_x4 = -a4*sin(t5);  %NOTE: THE AUTHOR FORGOT NEGATIVE SIGN IN THE PAPER
del5_y4 = a4*cos(t5);

del1_y4 = 0;
del1_x4 = 0;
del5_y2 = 0;
del5_x2 = 0;

% joint 1
del1_d = ( (x4-x2)*(del1_x4-del1_x2) + (y4-y2)*(del1_y4-del1_y2) ) / d;
del1_b = del1_d - (del1_d*(a2^2-a3^2+d^2))/(2*d^2);
del1_h = -b*del1_b / h;

del1_yh = del1_y2 + (del1_b*d-del1_d*b)/d^2 * (y4-y2) + b/d * (del1_y4 - del1_y2);
del1_xh = del1_x2 + (del1_b*d-del1_d*b)/d^2 * (x4-x2) + b/d * (del1_x4 - del1_x2);

del1_y3 = del1_yh - h/d * (del1_x4-del1_x2) - (del1_h*d - del1_d*h)/d^2 *(x4 - x2);
del1_x3 = del1_xh + h/d * (del1_y4-del1_y2) + (del1_h*d - del1_d*h)/d^2 *(y4 - y2);

% joint 2
del5_d = ( (x4-x2)*(del5_x4-del5_x2)+(y4-y2)*(del5_y4-del5_y2) ) / d;
del5_b = del5_d - (del5_d*(a2^2-a3^2+d^2))/(2*d^2);
del5_h = -b*del5_b / h;

del5_yh = del5_y2 + (del5_b*d-del5_d*b)/d^2 * (y4-y2) + b/d * (del5_y4 - del5_y2);
del5_xh = del5_x2 + (del5_b*d-del5_d*b)/d^2 * (x4-x2) + b/d * (del5_x4 - del5_x2);

del5_y3 = del5_yh - h/d * (del5_x4-del5_x2) - (del5_h*d - del5_d*h)/d^2 *(x4 - x2);
del5_x3 = del5_xh + h/d * (del5_y4-del5_y2) + (del5_h*d - del5_d*h)/d^2 *(y4 - y2);

J = [del1_x3 del5_x3; 
     del1_y3 del5_y3];


     */
}
