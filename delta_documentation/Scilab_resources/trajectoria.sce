//Draw trajectory
close();

// Aquesta funció requereix l'execució de l'script inverseDelta.sce. Totes les
// subrutines estan en aquest arxiu. Abans, però, cal definir els paràmetres de
// la màquina al mateix inverseDelta.sce (els trobareu a la capçalera). L'usuari
// ha de modificar el següent:
startPoint = [-0.15, -0.15, -0.25]
goalPoint = [0.15,0.15,-0.25]
stepNumber = 100
//

// Inicialitzacions, primer es calcula un vector trossejat en tantes parts com
// passos definits.
stepVect = (goalPoint - startPoint)*1/stepNumber;
// Altres inicialitzacions
keepTorques = zeros(stepNumber,3);
keepThetas = zeros(stepNumber,3);
keepSample = zeros(stepNumber,1);
figure(1);

for i=1:stepNumber
    // Calcula el següent punt
    currentPoint = startPoint + stepVect*i
    // Grafica'l
    plotDelta(currentPoint(1),currentPoint(2),currentPoint(3))
    //Calcula els angles
    [T1,T2,T3]=invDelta(currentPoint(1),currentPoint(2),currentPoint(3))
    keepThetas(i,:)=[T1,T2,T3]
    //Calcula els parells
    [T1,T2,T3]=torqueDelta(currentPoint(1),currentPoint(2),currentPoint(3))    
    keepTorques(i,:)=[T1,T2,T3]
    //Guarda la mostra
    keepSample(i,:) = i;
    
    clear T1
    clear T2
    clear T3
    //Si es vol fer una animació, descomentant la linia següent va dibuixant el
    //robot pas a pas
    //sleep(200)
end

// Dibuixa els altres gràfics amb les dades obtingudes
figure(2);
plot2d(keepSample(:,1),[keepThetas(:,1),keepThetas(:,2),keepThetas(:,3)],leg="theta1@theta2@theta3")
title("Diagrama dels angles dels motors")
xlabel("Sample")
ylabel("Angle [rad]")

figure(3);
plot2d(keepSample(:,1),[keepTorques(:,1),keepTorques(:,2),keepTorques(:,3)],leg="theta1@theta2@theta3")
title("Parells motors")
xlabel("Sample")
ylabel("Parell [Nm]")

// PS, és una forma bastant ineficient de fer els càlculs. S'ha millorat l'algorisme
// amb la funció allTogether, però es deixa per altres scripts més intensius en
// càlcul

