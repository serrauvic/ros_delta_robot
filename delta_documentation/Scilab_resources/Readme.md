Com heu de fer servir els arxius:

Bàsicament hi ha dos grups, els que van sols (deltaOptimization) i els que fan servir el fitxer inverseDelta.sce estretament.

En el deltaOptimization, el primer que heu de fer és canviar el "exec('/home/txema/Documents/IntegrationProject/Personal/inverseDelta.sce', -1)" per la direcció corresponent al lloc on heu deixat el vostre inverseDelta.sce. La resta és anar seguint els passos indicats als comentaris.

Els altres arxius estan basats en l'inverseDelta.sce i només provaràn la configuració carregada per defecte en aquest arxiu. La configuració carregada per defecte es pot trobar en el propi fitxer:

sB = 0.18;
sP = 0.08;
L = 0.09
l = 0.32;

Per executar qualsevol dels altres fitxers, primer heu de guardar i executar aquest.

workSpaceValidation.sce: Entreu l'espai que voleu provar:
x_space = [-0.2:0.01:0.2]
y_space = [-0.2:0.01:0.2]
z_space = linspace(-0.30,-0.2,9)

El primer número és la coordenada incial, el segon l'increment i el darrer la coordenada final. El z-space va diferent. Es posa la coordenada inicial, la coordenada final i el número de gràfics que voleu que us faci. Aquest número pot ser 1, 4, 9 o 16. Aquest fitxer pintarà les configuracions possibles i com de factibles són amb un color que indica el grau de proximitat a un punt singular.

trajectoria.sce: 
Poca cosa a dir, és bastant explicit. Si no dóna resultats és que els punts no es poden assolir. L'error típic és posar-li una coordenada z positiva. Al mig del codi hi ha un sleep(200). Si descomenteu la linia, entre gràfic i gràfic esperarà 200ms. Està bé perquè el resultat queda com una mena d'animació a càmera lenta.


trajectoriaV2.sce:
Aquest és bastant experimental, són proves per evitar punts crítics dividint la trajectoria en passos. Si es troba que no pot passar per un lloc, busca trajectories alternatives, però no està provat intensivament i potser peta.

Per últim, si l'scilab fa coses races, aneu a la pantalla principal i feu ctrl+c. Això ho para tot. 
 
