//IKP equations
//Defines
// En aquestes línies es descriu el cas puntual de robot que es vol testejar. 
//Aquestes variables són globals, alguns programes simplement les planxen per 
//provar altres valors
sB = 0.2//0.18;
sP = 0.08//0.08;
L = 0.095//0.09
l = 0.315//0.32;

// Les següents densitats són necessàries per calcular els parells en funció de 
//les longituds provades.
densityL = 0.2;// Kg/m
densityl = %pi*0.002^2*8000//0.15*2;//Kg/m
weightEff = 0.1;//Kg

//Calculations
// Càlculs previs en quant a distàncies, pesos i altres variables que es poden 
//trobar a l'article
// The Delta Robot: Kinematics Solutions de Robert L. Williams
g = 9.81;
wB = sB*sqrt(3)/6;
uB = sB*sqrt(3)/3;
wP = sP*sqrt(3)/6;
uP = sP*sqrt(3)/3;

a = wB-uP;
b = (sP/2)-(wB*sqrt(3)/2);
c = wP-(0.5*wB);

pesL = L*densityL*g;
pesl = l*densityl*g*2;
pesEff = weightEff*g;

//La següent funció resol el problema cinemàtic invers: Donades unes coordenades,
//retorna els àngles. En cas que no es pugui trobar una solució, fa saltar 
//un error abans de retornar números imaginaris o més enllà de -pi..pi. És per
//aquesta raó que es recomana cridar aquesta funció amb un "try - catch"
function[theta1,theta2,theta3] = invDelta(x,y,z)
    E1 = 2*L*(y+a);
    F1 = 2*z*L;
    G1 = x^2+y^2+z^2+a^2+L^2+2*y*a-l^2;
    
    E2 = -L*(sqrt(3)*(x+b)+y+c);
    F2 = 2*z*L;
    G2 = x^2+y^2+z^2+b^2+c^2+L^2+2*(x*b+y*c)-l^2;
    
    E3 = L*(sqrt(3)*(x-b)-y-c);
    F3 = 2*z*L
    G3 = x^2+y^2+z^2+b^2+c^2+L^2+2*(-x*b+y*c)-l^2;
    
    sol1plus = 2*atan((-F1+sqrt(E1^2+F1^2-G1^2))/(G1-E1));   
    sol1min = 2*atan((-F1-sqrt(E1^2+F1^2-G1^2))/(G1-E1));
    sol2plus = 2*atan((-F2+sqrt(E2^2+F2^2-G2^2))/(G2-E2));   
    sol2min = 2*atan((-F2-sqrt(E2^2+F2^2-G2^2))/(G2-E2)); 
    sol3plus = 2*atan((-F3+sqrt(E3^2+F3^2-G3^2))/(G3-E3));   
    sol3min = 2*atan((-F3-sqrt(E3^2+F3^2-G3^2))/(G3-E3));
    
    if isreal(sol1min) && sol1min<%pi && sol1min>-%pi then
        theta1 = sol1min;    
    elseif isreal(sol1plus) && sol1plus<%pi && sol1plus>-%pi then
        theta1 = sol1plus;
    else
        error("theta1 has not practical solution");
    end
    if isreal(sol2min) && sol2min<%pi && sol2min>-%pi then
        theta2 = sol2min;
    elseif isreal(sol2plus) && sol2plus<%pi && sol2plus>-%pi then
        theta2 = sol2plus;
    else
        error("theta2 has not practical solution");
    end
    if isreal(sol3min) && sol3min<%pi && sol3min>-%pi then
        theta3 = sol3min;
    elseif isreal(sol3plus) && sol3plus<%pi && sol3plus>-%pi then
        theta3 = sol3plus;
    else
        error("theta3 has not practical solution");
    end
    
//En graus:
/*    theta1 = theta1*180/%pi;
    theta2 = theta2*180/%pi;
    theta3 = theta3*180/%pi;
    disp('recorda treure angles')
*/            
    endfunction

// Aquesta funció retorna la posició de les joints A_i. Es farà servir més
//endavant
function[A1,A2,A3] = jointsPosition(theta1,theta2,theta3)
    
    A1 = [0,-wB-L*cos(theta1),-L*sin(theta1)];
    A2 = [sqrt(3)*(wB+L*cos(theta2))/2,(wB+L*cos(theta2))/2,-L*sin(theta2)];
    A3 = [-sqrt(3)*(wB+L*cos(theta3))/2,(wB+L*cos(theta3))/2,-L*sin(theta3)];
    endfunction

// Aquesta funció dibuixa el robot Delta en una configuració donada. Fa servir
// les funcions invDelta i jointsPosition. Sobre el seu ús: cal notar que no 
// obre una finestra nova, de manera que cal fer-ho abans. Es fa així per poder
// pintar diferents configuracions sense esborrar les prèvies.
function plotDelta(x,y,z)
    //Calcula la base: posició dels punts B
    B1x = [-sB/2,sB/2] 
    B1y = [-tan(%pi/6)*sB/2,-tan(%pi/6)*sB/2] 
    B1z = [0,0]
    
    //Calcula la base: és el triangle vermell (es dóna en 4 punts)
    BaseX = [-sB/2,sB/2,0,-sB/2]
    BaseY = [-wB,-wB,uB,-wB]
    BaseZ = [0,0,0,0]
    
    //Calcula la cinemàtica inversa
    [theta1,theta2,theta3] = invDelta(x,y,z);
    [A1,A2,A3] = jointsPosition(theta1,theta2,theta3);
    
    //Calcula l'end-effector, és el triangle blau (es dóna en 4 punts)
    EFX = [0+x,sP/2+x,-sP/2+x,0+x];
    EFY = [-uP+y,wP+y,wP+y,-uP+y];
    EFZ = [z,z,z,z]
    
    //Braç L
    //Calcula el braç 1
    b1xL = [0,A1(1)]
    b1yL = [-wB,A1(2)]
    b1zL = [0,A1(3)] 
    //Calcula el braç 2
    b2xL = [sqrt(3)*wB/2,A2(1)]
    b2yL = [0.5*wB,A2(2)]
    b2zL = [0,A2(3)] 
    //Calcula el braç 3
    b3xL = [-sqrt(3)/2*wB,A3(1)]
    b3yL = [0.5*wB,A3(2)]
    b3zL = [0,A3(3)] 
    
    //Braç l
    //Calcula el braç 1
    b1xl = [A1(1),x]
    b1yl = [A1(2),-uP+y]
    b1zl = [A1(3),z] 
    //Calcula el braç 2
    b2xl = [A2(1),sP/2+x]
    b2yl = [A2(2),wP+y]
    b2zl = [A2(3),z] 
    //Calcula el braç 3
    b3xl = [A3(1),-sP/2+x]
    b3yl = [A3(2),wP+y]
    b3zl = [A3(3),z] 
    
    //A partir d'aquí fa el dibuix, amb diferents línies i colors
    plot3d(BaseX',BaseY',BaseZ', flag=[5,8,4]) 
    plot3d(EFX',EFY',EFZ', flag=[2,8,4]) 
    param3d(b1xL',b1yL',b1zL')
    p=get("hdl"); //get handle on current entity (here the polyline entity)
    p.foreground=3;
    p.thickness=3;
    //p.mark_style=9; 
    param3d(b2xL',b2yL',b2zL') 
    p=get("hdl"); //get handle on current entity (here the polyline entity)
    p.foreground=3;
    p.thickness=3;
    //p.mark_style=9; 
    param3d(b3xL',b3yL',b3zL')
    p=get("hdl"); //get handle on current entity (here the polyline entity)
    p.foreground=3;
    p.thickness=3;
    //p.mark_style=9;  
    
    param3d(b1xl',b1yl',b1zl') 
    param3d(b2xl',b2yl',b2zl') 
    param3d(b3xl',b3yl',b3zl') 

    endfunction   

//Funció copiada d'internet, fa el producte vectorial
function [p] = CrossProd(u,v)
    //Calculates the cross-product of two vectors u and v.
    //Vectors u and v can be columns or row vectors, but
    //they must have only three elements each.
    [nu,mu] = size(u);
    [nv,mv] = size(v);
    if nu*mu <> 3 | nv*mv <> 3 then
    error('Vectors must be three-dimensional only')
    abort;
    end
    A1 = [ u(2), u(3); v(2), v(3)];
    A2 = [ u(3), u(1); v(3), v(1)];
    A3 = [ u(1), u(2); v(1), v(2)];
    px = det(A1); py = det(A2); pz = det(A3);
    p = [px, py, pz]
    //end function
    endfunction

// La següent funció calcula el parell que han de fer els motors del Robot
//en una configuració donada. Aquest parell és estàtic, no es tenen en compte
//inèrcies... Aquesta funció fa servir invDelta, jointsPosition i CrossProd.
//A la mateixa funció s'explica dón surten les equacions.
function[P1,P2,P3] = torqueDelta(x,y,z)
    //Calcula la cinemàtica inversa
    [theta1,theta2,theta3] = invDelta(x,y,z);
    [A1,A2,A3] = jointsPosition(theta1,theta2,theta3);
        //Suposicions fetes, descartant el moment en Z i tenint en compte les articulacions. El problema es planteja de la següent forma:
        // - Es considera només una branca.
        // - La base és estàtica i horitzontal. Això implica que la suma de forces verticals i horitzontals ha de ser zero
        // - Com la base està muntada sobre ròtules, el càlcul dels moments respecte qualsevol punt també ha de ser zero (excepte en z)
        // - En cas d'un pes mal distribuït al centroide del triangle, aquesta funció no seria vàlida, perquè el pes de l'end-effector
        // ja no estaria repartit exactament per 3 a cada branca.
    
    //Recàlcul dels punts P de la base mòvil
    P1B = [x,-uP+y,z];
    P2B = [sP/2+x,wP+y,z];
    P3B = [-sP/2+x,wP+y,z];
    //Recàlcul dels punts B de la base fixa
    B1 = [0,-wB,0];
    B2 = [sqrt(3)*wB/2, 0.5*wB,0];
    B3 = [-sqrt(3)*wB/2, 0.5*wB,0];
    //Vector "l" que va del punt A al punt P
    lv1 = P1B - A1;
    lv2 = P2B - A2;
    lv3 = P3B - A3;
    // Es calcula el mateix vector en unitari   
    lv1_u = lv1/norm(lv1);
    lv2_u = lv2/norm(lv2);
    lv3_u = lv3/norm(lv3);
    
    //Aquí es resol el sistema d'equacions que afecta a l'end-efector. S'imposa
    //el següent:
    //[lx ly lz] és un vector unitari de l
    //[lx ly lz]*k = F1
    //Això vol dir que la força que es farà en un extrem de la base de l'end
    //effector tindrà la direcció de l. D'acord amb això i posant el cas de
    //les tres barres, el que s'ha de complir és que la suma de forces sigui = 0
    // Per tant 
    //lx1*k+lx2*h+lx3*n = 0; Suma de forces en X
    //ly1*k+ly2*h+ly3*n = 0; Suma de forces en Y
    //lz1*k+lz2*h+lz3*n + P = 0 Suma de forces en Z
    
    //res = [k,h,n]
    //Multiplicant els vectors unitaris per la seva corresponent solució, 
    //donarà la força que aplica la barra l
    A = [lv1_u' lv2_u' lv3_u'];
    b = [0;0;-pesEff];
    res = linsolve(A,b);
    //disp(lv1_u*res(1))
    //disp(lv2_u*res(2))
    //disp(lv3_u*res(3))    
    //disp(res)
    
    //El pes de l'end effector no és la única força que actua sobre la barra,
    //també està el seu propi pes.Fer saber com es trasmet el pes a través de
    //la barra, es procedeix de manera similar a abans. El pes actua en l'eix
    //Z sempre. Si l és un vector unitari, la seva component en Z multiplicada
    //per alguna cosa ha de donar el Pes de la barra. Per tant:
    //-pesl/lv1_u(3) donarà aquesta incògnita. En multiplicar pel vector unitari
    //es sabrà com es trasmet la força a la barra. Això mateix sumant-li les
    //forces calculades abans, donarà l'estat de tensió de la barra. Cal notar el
    //signe negatiu dels valors calculats abans. Això es fa perque es calcula la
    //força que subjecta la base (la tensió de la barra) 
    tensiol1=(-pesl/lv1_u(3))*lv1_u - lv1_u*res(1)
    tensiol2=(-pesl/lv2_u(3))*lv2_u - lv2_u*res(2)
    tensiol3=(-pesl/lv3_u(3))*lv3_u - lv3_u*res(3)
    //disp(tensiol1)   
    
   
    //Finalment s'aplica la definició de parell respecte a un punt
    P1 = CrossProd((A1-B1), tensiol1) + CrossProd(0.5*(A1-B1), [0,0,-pesL])
    P2 = CrossProd((A2-B2), tensiol2) + CrossProd(0.5*(A2-B2), [0,0,-pesL])    
    P3 = CrossProd((A3-B3), tensiol3) + CrossProd(0.5*(A3-B3), [0,0,-pesL])
    
    //Si només es vol saber el parell dels motors. Es calcula el vector unitari de l'eix i es projecta. 
    P1 = [1,0,0]*P1'
    P2 = [cos(2*%pi/3),sin(2*%pi/3),0]*P2'
    P3 = [cos(4*%pi/3),sin(4*%pi/3),0]*P3'

    endfunction

function[Jx,Jq] = Jacobians(x,y,z)
    // Aquesta funció calcula els jacobians del robot Delta. Per veure la raó
    // per la qual apareixen 2, cal revisar l'article al que fa referència
    try
        [theta1,theta2,theta3] = invDelta(x,y,z)
        Jx=[x, y+a+L*cos(theta1), z+L*sin(theta1);...
        2*(x+b)-sqrt(3)*L*cos(theta2),2*(y+c)-L*cos(theta2),2*(z+L*sin(theta2));...
        2*(x-b)+sqrt(3)*L*cos(theta3),2*(y+c)-L*cos(theta3),2*(z+L*sin(theta3))]
        
        Jq=[L*((y+a)*sin(theta1)-z*cos(theta1)),0,0;...
        0,-L*((sqrt(3)*(x+b)+y+c)*sin(theta2)+2*z*cos(theta2)),0;...
        0,0,L*((sqrt(3)*(x-b)-y-c)*sin(theta3)-2*z*cos(theta3))]    
    catch
        Jx=ones(3,3)*1000
        Jq=eye(3,3)*0.001
    end
        
    endfunction

function[P1,P2,P3,Jx,Jq] = allTogether(x,y,z)
    // Aquesta funció és un resum de totes les anteriors, excepte les de fer plots.
    // Es disposa d'aquesta forma perquè a l'hora de fer iteracions, cal evitar
    // cridar funcions de forma repetitiva. El que s'intenta és sintetitzar al màxim
    // les funcions per millorar el temps de càlcul.
    E1 = 2*L*(y+a);
    F1 = 2*z*L;
    G1 = x^2+y^2+z^2+a^2+L^2+2*y*a-l^2;
    
    E2 = -L*(sqrt(3)*(x+b)+y+c);
    F2 = 2*z*L;
    G2 = x^2+y^2+z^2+b^2+c^2+L^2+2*(x*b+y*c)-l^2;
    
    E3 = L*(sqrt(3)*(x-b)-y-c);
    F3 = 2*z*L
    G3 = x^2+y^2+z^2+b^2+c^2+L^2+2*(-x*b+y*c)-l^2;
    
    sol1plus = 2*atan((-F1+sqrt(E1^2+F1^2-G1^2))/(G1-E1));   
    sol1min = 2*atan((-F1-sqrt(E1^2+F1^2-G1^2))/(G1-E1));
    sol2plus = 2*atan((-F2+sqrt(E2^2+F2^2-G2^2))/(G2-E2));   
    sol2min = 2*atan((-F2-sqrt(E2^2+F2^2-G2^2))/(G2-E2)); 
    sol3plus = 2*atan((-F3+sqrt(E3^2+F3^2-G3^2))/(G3-E3));   
    sol3min = 2*atan((-F3-sqrt(E3^2+F3^2-G3^2))/(G3-E3));
    
    if isreal(sol1min) && sol1min<%pi && sol1min>-%pi then
        theta1 = sol1min;    
    elseif isreal(sol1plus) && sol1plus<%pi && sol1plus>-%pi then
        theta1 = sol1plus;
    else
        error("theta1 has not practical solution");
    end
    if isreal(sol2min) && sol2min<%pi && sol2min>-%pi then
        theta2 = sol2min;
    elseif isreal(sol2plus) && sol2plus<%pi && sol2plus>-%pi then
        theta2 = sol2plus;
    else
        error("theta2 has not practical solution");
    end
    if isreal(sol3min) && sol3min<%pi && sol3min>-%pi then
        theta3 = sol3min;
    elseif isreal(sol3plus) && sol3plus<%pi && sol3plus>-%pi then
        theta3 = sol3plus;
    else
        error("theta3 has not practical solution");
    end
    A1 = [0,-wB-L*cos(theta1),-L*sin(theta1)];
    A2 = [sqrt(3)*(wB+L*cos(theta2))/2,(wB+L*cos(theta2))/2,-L*sin(theta2)];
    A3 = [-sqrt(3)*(wB+L*cos(theta3))/2,(wB+L*cos(theta3))/2,-L*sin(theta3)];
    
    P1B = [x,-uP+y,z];
    P2B = [sP/2+x,wP+y,z];
    P3B = [-sP/2+x,wP+y,z];
    B1 = [0,-wB,0];
    B2 = [sqrt(3)*wB/2, 0.5*wB,0];
    B3 = [-sqrt(3)*wB/2, 0.5*wB,0];
    
    lv1 = P1B - A1;
    lv2 = P2B - A2;
    lv3 = P3B - A3;
     // Es calcula el mateix vector en unitari   
    lv1_u = lv1/norm(lv1);
    lv2_u = lv2/norm(lv2);
    lv3_u = lv3/norm(lv3);
   
    A = [lv1_u' lv2_u' lv3_u'];
    bb = [0;0;-pesEff];
    res = linsolve(A,bb);
    
    tensiol1=(-pesl/lv1_u(3))*lv1_u - lv1_u*res(1)
    tensiol2=(-pesl/lv2_u(3))*lv2_u - lv2_u*res(2)
    tensiol3=(-pesl/lv3_u(3))*lv3_u - lv3_u*res(3)
   
    P1 = CrossProd((A1-B1), tensiol1) + CrossProd(0.5*(A1-B1), [0,0,-pesL])
    P2 = CrossProd((A2-B2), tensiol2) + CrossProd(0.5*(A2-B2), [0,0,-pesL])    
    P3 = CrossProd((A3-B3), tensiol3) + CrossProd(0.5*(A3-B3), [0,0,-pesL])
    
    //Si només es vol saber el parell dels motors. Es calcula el vector unitari de l'eix i es projecta. 
    P1 = [1,0,0]*P1'
    P2 = [cos(2*%pi/3),sin(2*%pi/3),0]*P2'
    P3 = [cos(4*%pi/3),sin(4*%pi/3),0]*P3'

    Jx=[x, y+a+L*cos(theta1), z+L*sin(theta1);...
    2*(x+b)-sqrt(3)*L*cos(theta2),2*(y+c)-L*cos(theta2),2*(z+L*sin(theta2));...
    2*(x-b)+sqrt(3)*L*cos(theta3),2*(y+c)-L*cos(theta3),2*(z+L*sin(theta3))]
    
    Jq=[L*((y+a)*sin(theta1)-z*cos(theta1)),0,0;...
    0,-L*((sqrt(3)*(x+b)+y+c)*sin(theta2)+2*z*cos(theta2)),0;...
    0,0,L*((sqrt(3)*(x-b)-y-c)*sin(theta3)-2*z*cos(theta3))]
    
    if abs(det(Jx))<0.000001 || abs(det(Jq))<0.000001then
        error("J no invertible");
    end
endfunction


