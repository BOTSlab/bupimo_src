#include "MFDID.h"

MFDID::MFDID( int width, int height ) 
    : T_11(width, height),
      T_12(width, height),
      T_21(width, height),
      T_22(width, height),
      gradX(width, height),
      gradY(width, height),
      diff(width, height),
      temp1(width, height),
      temp2(width, height),
      temp3(width, height),
      Vx(width, height),
      Vy(width, height)
{
    // Precompute the inverse matrix for all angles
    float delta = 2*(float)M_PI / width;
    float gamma0 = delta * (height-1)/2.0f;
    for ( int i=0; i<width; i++ )
        for ( int j=0; j<height; j++ ) {
            float beta = 2*(float)M_PI - delta*i;
            float gamma = gamma0 - delta*j;
            if ( fabs(cos(gamma)) > 0.01 ) {
                T_11.set(i, j, (float) (sin(beta)/cos(gamma)));
                T_12.set(i, j, (float) (cos(beta)*sin(gamma)));
                T_21.set(i, j, (float) (-cos(beta)/cos(gamma)));
                T_22.set(i, j, (float) (sin(beta)*sin(gamma)));
            }
        }
}

Vec2 MFDID::currentView( Img* CV ) {
    // Compute image gradient (stored as two images)
    ImgOps::diffHorz(CV, &gradX);
    ImgOps::diffVert(CV, &gradY);

    // Compute difference image
    ImgOps::sub(SS, CV, &diff);

    // Compute the x and y components of the home vector from equation (12).
    ImgOps::mult(&T_11, &gradX, &temp1); 
    ImgOps::mult(&T_12, &gradY, &temp2);
    ImgOps::add(&temp1, &temp2, &temp3);
    ImgOps::mult(&temp3, &diff, &Vx);

    ImgOps::mult(&T_21, &gradX, &temp1); 
    ImgOps::mult(&T_22, &gradY, &temp2);
    ImgOps::add(&temp1, &temp2, &temp3);
    ImgOps::mult(&temp3, &diff, &Vy);

    float vx = Vx.getSum();
    float vy = Vy.getSum();

    double mag = sqrt(vx*vx + vy*vy);
cout << "mag: " << mag << endl;
    return Vec2(1, (float) atan2(vy, vx));
}
