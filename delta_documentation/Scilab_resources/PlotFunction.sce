//This script plots the 3D Delta Robot

//Ull, primer cal executar el inverseDelta.sce

//Calcula la base
B1x = [-sB/2,sB/2] 
B1y = [-tan(%pi/6)*sB/2,-tan(%pi/6)*sB/2] 
B1z = [0,0]

BaseX = [-sB/2,sB/2,0,-sB/2]
BaseY = [-wB,-wB,uB,-wB]
BaseZ = [0,0,0,0]

//Calcula la cinemàtica inversa
x = 0;
y = 0.1;
z = -1.0;
[theta1,theta2,theta3] = invDelta(x,y,z);
[A1,A2,A3] = jointsPosition(theta1,theta2,theta3);

//Calcula l'end-effector
EFX = [0+x,sP/2+x,-sP/2+x,0+x];
EFY = [-uP+y,wP+y,wP+y,-uP+y];
EFZ = [z,z,z,z]

//Calcula el braç 1
b1x = [0,A1(1),x]
b1y = [-wB,A1(2),-uP+y]
b1z = [0,A1(3),z] 
//Calcula el braç 2
b2x = [sqrt(3)*wB/2,A2(1),sP/2+x]
b2y = [0.5*wB,A2(2),wP+y]
b2z = [0,A2(3),z] 
//Calcula el braç 3
b3x = [-sqrt(3)/2*wB,A3(1),-sP/2]
b3y = [0.5*wB,A3(2),wP+y]
b3z = [0,A3(3),z] 

close
plot3d(BaseX',BaseY',BaseZ', flag=[0,8,4]) 
plot3d(EFX',EFY',EFZ', flag=[2,8,4]) 
param3d(b1x',b1y',b1z') 
param3d(b2x',b2y',b2z') 
param3d(b3x',b3y',b3z') 


