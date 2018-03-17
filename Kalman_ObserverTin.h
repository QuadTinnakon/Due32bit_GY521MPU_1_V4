/*
project_Quad 32 bit Arduino Due
//GPS
1. stabilized quadrotor 
by: tinnakon kheowree 
0860540582
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com/
https://www.facebook.com/tinnakonza

//A=[1 0.01;0 1]; //B=[0;0.01];  //C = [1 0];
//Plant = ss(A,[B B],C,0,-1,'inputname',{'u' 'w'},'outputname','y');
//Q = 0.1; % A number greater than zero
//R = 0.135; % A number greater than zero
//[kalmf,L,P,M,Z] = kalman(Plant,Q,R); //%kalmf = kalmf(1,:); //M,   % innovation gain
*/
void Observer_kalman_filter()
{
///Complimentary Filter, Observer velocity vx vy vz kalman//accelerometer and GPS/////////////////////////////////////////////////////
     //velocity_northf = _velocity_north + (gyroY_Earth*arm_GPS);//+ (gyroY_Earth*arm_GPS) velocity GPS Rotated Frame of arm gps
     //velocity_eastf = _velocity_east - (gyroX_Earth*arm_GPS);//- (gyroX_Earth*arm_GPS)
     //Predicted (a priori) state estimate ,,mx_dotdot + cx_dot = m*g*sin(pitch)
      x2_vt = vx_hat2 + ((accrX_Earth*100.1 - (vx_hat2*c_quad/m_quad))*G_Dt);//cm/s
      y2_vt = vy_hat2 + ((accrY_Earth*100.1 - (vy_hat2*c_quad/m_quad))*G_Dt);//cm/s
      x2_vt = constrain(x2_vt, -400, 400);//+-4 cm/s
      y2_vt = constrain(y2_vt, -400, 400);//+-4 cm/s
      //x1_xt += x2_vt*G_Dt;
      //Updated (a posteriori) state estimate //Update estimate with measurement zk
      //float temp_vx = accrX_Earth*100.1 + (velocity_northf - vx_hat)*9.2;//2.15 cm/s 1.5 6.5      vx_hat = vx_hat + temp_vx*G_Dt;
      vx_hat2 = x2_vt + (_velocity_north - x2_vt)*0.012821;//kalman gain ,0.009821, 0.01321 ,0.0221,
      vy_hat2 = y2_vt + (_velocity_east - y2_vt)*0.012821;//0.008821
      vx_hat = vx_hat2;
      vy_hat = vy_hat2;
      applyDeadband(vx_hat, 4.5);
      applyDeadband(vy_hat, 4.5);//4.5 10.5
//Predicted (a priori) state estimate  Altitude
u_z = ((motor_FrontLf + motor_FrontRf + motor_BackLf + motor_BackRf) - 4000.0)*0.00630057 - 8.81;//1557us = 1.16kg  uz - g ,,unit N *0.001691) - 9.81
u_zdot = (u_z - u_zold)/G_Dt;
u_zold = u_z;
//u_z = accrZ_Earth;//accrZ_Earth applyDeadband = 0.1 m/s^2  = ,,(u - c*z2_hat)/m = Accz
z3_acc = z3_hat + (u_zdot - c_quad*z3_hat)*G_Dt;//1.52, 1.23 ,z3_hat = Acceleration,,m/s^2
z2_hat2 = z2_hat + z3_acc*G_Dt;//z2_hat = velocity ,, m/s
z1_hat2 = z1_hat + z2_hat2*G_Dt;//z1_hat = Altitude ,, m
//z1_hat = constrain(z1_hat, 0, 100);//0 - 100 m
///////////////////////////////////////////////////////////////////
//Updated (a posteriori) state estimate //Update estimate with measurement zk
float ee1 = (Altitude_Baro_ult - z1_hat2);
float ee2 = (Vz_Baro_ult - z2_hat2);
float ee3 = (accrZ_Earth - z3_acc);
//ui_z1_hat = ui_z1_hat + ee3*G_Dt;//I,0.00086056 0.001056 ,0.004056 ,0.008056 ,0.02156 ,1.0156 ,0.156
z3_hat = z3_acc + 0.088824*ee3;//0.018824 ,0.18824 m/s^2 ,,0.12824 0.21845
//K3 = 0.00124 0.12502 ,, k3 = 0.01641023 ,, k-3 = 0.12367 0.72367
z2_hat = z2_hat2 + 0.00151202*ee2;//m/s 0.00151202 0.00751202 0.086056
//K2 = 0.0091202 0.012502 0.0002141023, 0.0001641, 0.00145, 0.00045,,,k2 = 0.0001641023 0.097502,  0.065502, 0.009
z1_hat = z1_hat2 + 0.0081102*ee1;
//K1 = 0.0081102 0.011102 0.015102 0.035102, 0.015102 0.065 0.2887 ,, k1 = 0.0001741023 0.09187 0.01187 0.0140
}
