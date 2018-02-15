//Draw trajectory
// Aquest script dibuixarà una trajectòria del delta esquivant punts singulars.
// Pot ser necessari ajustar els paràmetres de la màquina al fitxer inverseDelta.sce
// Aquest script ÉS TOTALMENT EXPERIMENTAL, no hiha garanties de que funcioni
// Neteja la memòria, figures i executa l'script inverseDelta.sce
close();
clear
exec('/home/txema/Documents/IntegrationProject/Personal/inverseDelta.sce', -1)

// Definicions de l'usuari: Pas mínim i zona de treball vàlida
minStepSize = 0.005;
xRange=[-0.3,0.3];
yRange=[-0.3,0.3];
zRange=[-0.5,-0.15];

// Número de Frobenius, per més info: "Análisis del desempeño cinetostático de un robot
// paralelo tipo Delta reconfigurable". Com més alt, més proper a la inestabilitat
nFrobeniusMax = 50;

// Coordenades origen i objectiu respectivament:
testPoint = [-0.15,-0.15,-0.2];
objective=[0.15,0.15,-0.2];

// Inicialitzacions
exploredValues = ones(1000,3);
indexExplored = 1;
figure(1);

// Current point passa a ser l'origen
currentPoint = testPoint;

// Bucle limitat a 1000 iteracions per assegurar l'estabilitat del programa.
tic();
for bucles = 1:1000
    //Calcula tots els punts al voltant del punt actual on pot anar el End-effector
    nextPoints = zeros(27,3);
    nextPoints(1,:) = currentPoint + [-1,-1,-1]*minStepSize;
    nextPoints(2,:) = currentPoint + [ 0,-1,-1]*minStepSize;
    nextPoints(3,:) = currentPoint + [+1,-1,-1]*minStepSize;
    nextPoints(4,:) = currentPoint + [-1, 0,-1]*minStepSize;
    nextPoints(5,:) = currentPoint + [ 0, 0,-1]*minStepSize;
    nextPoints(6,:) = currentPoint + [+1, 0,-1]*minStepSize;
    nextPoints(7,:) = currentPoint + [-1,+1,-1]*minStepSize;
    nextPoints(8,:) = currentPoint + [ 0,+1,-1]*minStepSize;
    nextPoints(9,:) = currentPoint + [+1,+1,-1]*minStepSize;
    nextPoints(10,:) = currentPoint + [-1,-1, 0]*minStepSize;
    nextPoints(11,:) = currentPoint + [ 0,-1, 0]*minStepSize;
    nextPoints(12,:) = currentPoint + [+1,-1, 0]*minStepSize;
    nextPoints(13,:) = currentPoint + [-1, 0, 0]*minStepSize;
    // Aquest es correspondria al punt actual, en canvi es tria un punt que minimitzi la distància a l'objectiu
    nextPoints(14,:) = currentPoint + sqrt(3)*minStepSize*(objective - currentPoint)/norm(objective - currentPoint);
    nextPoints(15,:) = currentPoint + [+1, 0, 0]*minStepSize;
    nextPoints(16,:) = currentPoint + [-1,+1, 0]*minStepSize;
    nextPoints(17,:) = currentPoint + [ 0,+1, 0]*minStepSize;
    nextPoints(18,:) = currentPoint + [+1,+1, 0]*minStepSize;
    nextPoints(19,:) = currentPoint + [-1,-1,+1]*minStepSize;
    nextPoints(20,:) = currentPoint + [ 0,-1,+1]*minStepSize;
    nextPoints(21,:) = currentPoint + [+1,-1,+1]*minStepSize;
    nextPoints(22,:) = currentPoint + [-1, 0,+1]*minStepSize;
    nextPoints(23,:) = currentPoint + [ 0, 0,+1]*minStepSize;
    nextPoints(24,:) = currentPoint + [+1, 0,+1]*minStepSize;
    nextPoints(25,:) = currentPoint + [-1,+1,+1]*minStepSize;
    nextPoints(26,:) = currentPoint + [ 0,+1,+1]*minStepSize;
    nextPoints(27,:) = currentPoint + [+1,+1,+1]*minStepSize;
    minValue = 50000000;
    optimVect = zeros(1,3);

    //Al següent tros de codi es mira per cada punt quin és el més proper i a més
    //és una solució vàlida
    for k=1:27
        if minValue > norm(objective-nextPoints(k,:))
            if nextPoints(k,1) > xRange(1) && nextPoints(k,1) < xRange(2)
                if nextPoints(k,2) > yRange(1) && nextPoints(k,2) < yRange(2)
                    if nextPoints(k,3) > zRange(1) && nextPoints(k,3) < zRange(2)
                        try
                            [Jx,Jq] = Jacobians(nextPoints(k,1),nextPoints(k,2),nextPoints(k,3))
                            nFrobenius = norm(inv(Jq)*Jx)
                            if (find(members(exploredValues,nextPoints(k,:),'rows'),1)==[] && nFrobenius < nFrobeniusMax)                            
                                minValue = norm(objective-nextPoints(k,:));
                                optimVect = nextPoints(k,:);
                            else
                            end
                        catch
                        end                        
                    end
                end
            end
        end 
    end
    //El punt trobat el guarda com a explorat per no repetir-lo    
    exploredValues(indexExplored,:) = optimVect;
    //Dibuixa la màquina
    plotDelta(optimVect(1),optimVect(2),optimVect(3))
    //Conserva els angles
    [T1,T2,T3]=invDelta(optimVect(1),optimVect(2),optimVect(3))
    keepThetas(indexExplored,:)=[T1,T2,T3]
    //Conserva els parells
    [T1,T2,T3]=torqueDelta(optimVect(1),optimVect(2),optimVect(3))    
    keepTorques(indexExplored,:)=[T1,T2,T3]
    //Conserva la mostra
    keepSample(indexExplored,:) = indexExplored;
    indexExplored = indexExplored + 1;
    //Avança, la posició actual és l'òptima trobada
    currentPoint = optimVect;
    

    if indexExplored > 1000
        indexExplored = 1;
    end
    // Si el punt final és molt proper al punt actual, dóna per acabat el programa
    if norm(objective-currentPoint) < minStepSize
        break
    end
   
end
toc()

//Acaba de graficar i presentar els resultats dels punts guardats a l'apartat anterior
indexExplored = indexExplored -1
figure(2);
plot2d(keepSample(1:indexExplored,1),[keepThetas(1:indexExplored,1),keepThetas(1:indexExplored,2),keepThetas(1:indexExplored,3)],leg="theta1@theta2@theta3")
title("Thetas")

figure(3);
plot2d(keepSample(1:indexExplored,1),[keepTorques(1:indexExplored,1),keepTorques(1:indexExplored,2),keepTorques(1:indexExplored,3)],leg="theta1@theta2@theta3")
title("Torques")

figure(4);
scatter3(exploredValues(1:indexExplored,1),exploredValues(1:indexExplored,2),exploredValues(1:indexExplored,3))

