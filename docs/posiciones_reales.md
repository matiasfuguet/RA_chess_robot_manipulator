home: 79.77, -80.75, 55.31, -68.87, -87.87, 348.59

d5_hover: 59.03, -91.15, 99.47, -100.65, -88.17, 327.52
d5: 59.03, -87.19, 111.60, -116.73, -88.17, 327.52
e4_hover: 71.64, -79.26, 87.24, -101.75, -86.09, 340.34
e4: 71.64, -75.35, 99.23, -117.66, -86.09, 340.34
graveyard_hover: 30.45, -61.17, 60.89, -90.89, -87.28, 298.92
graveyard: 30.45, -59.38, 73.70, -105.48, -87.28, 292.92

posicion peon negro en d5: X:-51.85mm Y:-321.45mm Z:-357.52mm RX:0.042RAD RY:-3.146RAD RZ:0.081RAD
posicion peon blanco en e4: X:10.77mm Y:-382.27mm Z:-360.40mm RX:0.040RAD RY:-3.182RAD RZ:0.134RAD
posicion graveyard (una casilla a la izquierda de a5): X:-299.99mm Y:-321.43mm Z:-357.58mm RX:0.043RAD RY:-3.146RAD RZ:0.081RAD


Esto nos puso el profe en otro proyecto con este robot: 

Hola a todos,

Después de que algunos de vosotros me comentarais que os aparecían valores extraños al hacer la normalización que os escribí en la pizarra, me quedé haciendo un par de pruebas con el robot real y me he dado cuenta de que la fórmula que os puse estaba mal. ¡Perdonad!

Para las articulaciones 1, 2, 4, 5 y 6 (todas menos el codo):

    q_i en radianes está en [-2pi,2pi]

    q_i_normalizado=(q_i+2pi)/4pi en [0,1]

Para la articulación 3 (el codo):

    q_i en radianes está en [-pi,pi] (en realidad no exactamente, pero podéis expresarlo en este intervalo usando trigonometría básica)

    q_i_normalizado=(q_i+pi)/2pi en [0,1]

Con esto ya debería funcionar correctamente a partir de los valores que obtengáis del robot real.

Saludos y buen finde.

Isiah   
