// Aquesta funció optimitza el robot delta en funció de l'espai de treball
//Pasos previs, netejar la memòria
clc
clear
close
g = 9.81;
exec('/home/txema/Documents/IntegrationProject/Personal/inverseDelta.sce', -1)
warning('off')
tic();
// 1er pas:
// Espai de treball:
xWS = [-0.175,0.175]//m
yWS = [-0.175,0.175]//m
zWS = [-0.28,-0.23]//m

sBRange = [0.1,0.2];
sPRange = [0.08,0.15];
LRange = [0.06,0.1];
lRange = [0.25,0.35];

// 2n pas:
// Alguns límits
maxMotorTorque = 0.25 //Nm

// 3er pas:
// Densitats lineals dels materials i pesos (funció de cost):
densityL = 0.2;// Kg/m
densityl = 0.15*2;//Kg/m
weightEff = 0.1;//Kg

// 4rt pas
// Resolució dels càlculs en l'espai de treball. Com més petit, més duraran els càlculs, però més precissió. Es recomana al voltant de 0.025
resEspaial = 0.025;//m

// 5è pas
// Pas triat en les mides, per exemple, per fer càlculs cada 5mm = 0.005m 
resConstruccio = 0.005;//m

// 6è pas
// Calcular (guardeu i executeu)

mprintf("Calculant . . .\n")
//Redefinicions
x_space= [xWS(1):resEspaial:xWS(2)]
y_space= [yWS(1):resEspaial:yWS(2)]
z_space= [zWS(1):resEspaial:zWS(2)]

sB_vector = [sBRange(1):resConstruccio:sBRange(2)]
sP_vector = [sPRange(1):resConstruccio:sPRange(2)]
L_vector = [LRange(1):resConstruccio:LRange(2)]
l_vector = [lRange(1):resConstruccio:lRange(2)]

totalIteracions = length(sB_vector)*length(sP_vector)*length(L_vector)*length(l_vector)
mprintf("Es faran %i iteracions. Per parar ctrl+c.\n",totalIteracions)
iteracioNum = 0
candidats = 0
results = []
//Iterar, iterar i iterar per cara mida possible en tot l'espai de treball
for i=1:length(sB_vector)
    for j=1:length(sP_vector)
        for k=1:length(L_vector)
            for m=1:length(l_vector)
                iteracioNum = iteracioNum + 1
                // Mostra alguna cosa per pantalla per mostrar a l'usuari que el programa segueix viu
                if modulo(iteracioNum,500)==0 then
                    clc;
                    mprintf("Treballant. . . \n\n")
                    mprintf("Iteracio %i de %i. Trobats %i. Queden %.1fs (%.1fh).\n\n",iteracioNum,totalIteracions,candidats,...
                        toc()*(totalIteracions-iteracioNum)/500,toc()*(totalIteracions-iteracioNum)/(500*3600)  )
                    if candidats<10 && candidats >= 1 then
                        mprintf("Darrer valor trobat:\n")                        
                        mprintf("sB = %.3fm sP = %.3fm L = %.3fm l = %.3fm max torque = %.3fNm ratio %.2f%%\n",...
                                    results(candidats,1),results(candidats,2),results(candidats,3),results(candidats,4),results(candidats,5),results(candidats,8) )
                    end
                    if candidats>10 then
                        mprintf("Darrers valors trobats:\n")
                        for ii=(candidats-9):candidats
                             mprintf("sB = %.3fm sP = %.3fm L = %.3fm l = %.3fm max torque = %.3fNm ratio %.2f%%\n",...
                                    results(ii,1),results(ii,2),results(ii,3),results(ii,4),results(ii,5),results(ii,8) )
                        end
                    end            
                    tic();                    
                end
                sB = sB_vector(i);
                sP = sP_vector(j);
                L = L_vector(k);
                l = l_vector(m);
                                
                wB = sB*sqrt(3)/6;
                uB = sB*sqrt(3)/3;
                wP = sP*sqrt(3)/6;
                uP = sP*sqrt(3)/3;
                
                a = wB-uP;
                b = (sP/2)-(wB*sqrt(3)/2);
                c = wP-(0.5*wB);
                
                pesL = L*densityL*g;
                pesl = l*densityl*g;
                pesEff = weightEff*g;
                
                torqueValidated = []
                numFrobenius = []
                
                members_found = 0;
                members_lost = 0;

                for x=1:length(x_space)
                    for y=1:length(y_space)
                        for z=1:length(z_space)
                            try
                                [P1,P2,P3,Jx,Jq] = allTogether(x_space(x),y_space(y),z_space(z))                                                            
                                nFrobenius = norm(inv(Jq)*Jx)
                                if nFrobenius < 50.0   then                    
                                    numFrobenius = [numFrobenius nFrobenius]//[colorMarkers nDet]                
                                    torqueValidated = [torqueValidated max([P1,P2,P3])]
                                    members_found = members_found + 1;
                                else
                                    members_lost = members_lost + 1;
                                end
                            catch
                                members_lost = members_lost + 1;
                            end
                        end        
                    end    
                end
                membersRatio=members_found/(members_found+members_lost)
                //Avalua si cal guardar-lo
                if max(torqueValidated)<maxMotorTorque && membersRatio>0.95 then
                    candidats = candidats + 1
                    results(candidats,:)=[sB sP L l max(torqueValidated) mean(torqueValidated) mean(numFrobenius) membersRatio*100]
                    
                end               
            end
        end
    end
end
toc();
//Guarda un csv per si de cas...
csvWrite(results, 'Resultats.csv')

//Calculations

