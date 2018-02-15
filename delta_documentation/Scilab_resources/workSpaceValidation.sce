//Work space validation
close

// Aquest script validarà i dibuixarà l'espai de treball del Delta, seccionat en
// diferents alçades. 
// Per tant, es requereix l'execució de l'script inverseDelta.sce. Totes les
// subrutines estan en aquest arxiu. Abans, però, cal definir els paràmetres de
// la màquina al mateix inverseDelta.sce (els trobareu a la capçalera). L'usuari
// ha de modificar el següent:
x_space = [-0.2:0.01:0.2] //El primer número és l'origen, després l'interval de càlcul i el punt final
y_space = [-0.2:0.01:0.2]
// La coordenada z va diferent, el primer número és l'origen, el segon el punt final i el darrer
// és el número de càlculs que es vol fer. ALERTA, més endavant es fa servir la funció subplot
// que mostra els resultats obtinguts. Aquesta funció farà l'arrel quadrada d'aquest darrer
// número i ha de donar enter. Per tant pot ser 4, 9 o 16. TODO: cal arreglar això. 
z_space = linspace(-0.30,-0.20,9)

// Inicialitzacions arbitràries
xValidated = [ ]
yValidated = [ ]
zValidated = [ ]
torqueValidated = []
numFrobenius = []

members_found = 0;
members_lost = 0;

// Repasa un per un, tots els punts que es volen calcular
for x=1:length(x_space)
    for y=1:length(y_space)
        for z=1:length(z_space)
            // El procediment try...catch intentarà executar el codi següent. Pot passar
            // que no hi hagi una solució vàlida i les funcions que depenen de les coordenades
            // passades donin error. Si això passa, s'executa el catch i el programa no
            // acaba.
            try
                [theta1,theta2,theta3] = invDelta(x_space(x),y_space(y),z_space(z))
                [P1,P2,P3] = torqueDelta(x_space(x),y_space(y),z_space(z))
                [Jx,Jq] = Jacobians(x_space(x),y_space(y),z_space(z))
                // El número de Frobenius retorna "com d'aprop" estan les sigularitats.
                // La seva funció és similar a la del determinant de la jacobiana.
                nFrobenius = norm(inv(Jq)*Jx)
                nDet = abs(det(inv(Jq)*Jx))
                // En cas de que el punt sigui vàlid, però es trobi proper a una zona
                // singular, queda descartat
                if nFrobenius < 50.0 && nDet > 0.00000001  then                    
                    xValidated = [xValidated x_space(x)]
                    yValidated = [yValidated y_space(y)]
                    zValidated = [zValidated z_space(z)]
                    numFrobenius = [numFrobenius nFrobenius]
                    // Es mira també que no es superi el parell màxim dels servos                
                    torqueValidated = [torqueValidated max([P1,P2,P3])]
                    // Si tot OK, considera el punt com vàlid i se'l guarda per fer
                    // estadística.
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

f=gcf(); // Create a figure
f.figure_name= "Diagrama de la zona de treball del Delta";

// Aquest loop fa els gràfics dels punts trobats
for z=1:length(z_space)
    subplot(sqrt(length(z_space)),sqrt(length(z_space)),z)
    plotThisX = [];
    plotThisY = [];
    thisColor = [];
    for n=1:members_found
        if zValidated(n)==z_space(z) then
            plotThisX = [plotThisX xValidated(n)]
            plotThisY = [plotThisY yValidated(n)]
            thisColor = [thisColor numFrobenius(n)]
        end
    end
    typeMarkers = 1*ones(1,length(plotThisX))
    // set color map
    f.color_map = jetcolormap(64);
    scatter( plotThisX, plotThisY, typeMarkers,thisColor,"fill")
    title(['z = ' string(z_space(z)*1000) ' mm'])

end

// Es neteja la pantalla i es fa un petit resum de tota l'estadística adquirida.
clc
mprintf('Statistics:\n')
mprintf("Percentatge de l`espai cobert = %f %%\n",members_found/(members_found+members_lost))
mprintf("Parell maxim calculat = %f Nm\n",max(torqueValidated))
mprintf("Parell mig calculat = %f Nm\n",mean(torqueValidated))
mprintf("Mitjana de les normes de Frobenius = %f \n",mean(numFrobenius))

// Nota al peu; si els intervals de càlcul són molt petits, pot ser que costi molt
// fer els darrers diagrames. Per alguna raó són molt exigents en memòria. 





