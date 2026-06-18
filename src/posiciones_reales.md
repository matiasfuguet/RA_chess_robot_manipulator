no tengo ni el home ni el graveyard, toca inventarlos donde parezca razonable, luego ya lo modificare con valores reales pero de momento que funcione.

d5: 59.03, -87.19, 111.60, -116.73, -88.17, 327.52
e4: 71.64, -75.35, 99.23, -117.66, -86.09, 340.34


posicion peon negro en d5: X:-51.85mm Y:-321.45mm Z:-357.52mm RX:0.042RAD RY:-3.146RAD RZ:0.081RAD
posicion peon blanco en e4: X:10.77mm Y:-382.27mm Z:-360.40mm RX:0.040RAD RY:-3.182RAD RZ:0.134RAD


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