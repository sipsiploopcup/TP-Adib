# Rapport – TP Lidar LD06, Jour 1

## 1. Extraction des données lidar

Le LD06 est relié à un ESP32, qui décode les trames binaires reçues sur
son UART à 230400 bauds. La bibliothèque LD06 embarquée contrôle le CRC
(option activée par défaut) et reconstruit les scans. Le programme de test
transmet ensuite les points par USB à 115200 bauds, avec `printScanCSV`.
L'envoi du programme sur la carte a réussi après réduction de la vitesse
de téléversement de 921600 à 115200 bauds.

Nous conservons les six colonnes : numéro du point, angle en degrés,
distance en millimètres, intensité, x et y calculés par la bibliothèque
en millimètres. Les coordonnées polaires demandées restent ainsi disponibles.
Les x/y sont conservés tels quels ; leur convention d'orientation devra être
vérifiée avec un obstacle de position connue avant la cartographie.

Le lecteur Python attend un index zéro, vérifie la continuité des indices
et les valeurs, puis attend la ligne vide séparant les scans. Il contrôle
également que les angles décroissent d'une valeur proche de 360° à une
valeur proche de 0° (tolérance de 5° aux extrémités). Les fichiers portent
un nom horodaté et ne remplacent pas les acquisitions précédentes.
Le CRC n'est pas recalculé en Python, car les octets des trames brutes
ne sont pas transmis dans le CSV. Les contrôles du lecteur ne détectent
pas toutes les pertes possibles de trames en amont dans l'ESP32.

Validation logicielle : trois tests sur des données synthétiques couvrent
l'enregistrement, le rejet de blocs invalides et la reprise à un nouvel
index zéro. Une relecture de la capture série réelle fournie a reconnu
82 blocs de scans, de 342 à 456 points. Cette relecture n'a pas créé de
fichiers d'acquisition. Un premier enregistrement direct par Python a ensuite
réussi sur le port `/dev/cu.usbserial-1420` : le fichier
`scans/scan_20261102_124403_644384.csv` contient 454 points réels et les six
colonnes prévues. Le relevé de la pose et la validation de l'orientation
avec un obstacle connu restent à réaliser.

## 2. Visualisation des nuages de points

Nous réutilisons les coordonnées cartésiennes calculées par la bibliothèque,
sans refaire la conversion trigonométrique des mesures polaires. Le code
embarqué fourni calcule `y_lib = -r*sin(angle)` lorsque les poses embarquées
sont nulles. Pour suivre le repère mathématique du TP, le lecteur convertit
`x = x_lib_mm / 1000` et `y = -y_lib_mm / 1000`. Les coordonnées sont donc
exprimées en mètres, avec des axes de même échelle. Les distances nulles
sont exclues du nuage car elles ne correspondent pas à un impact valide.

Chaque scan reçoit une couleur distincte. Le visualiseur permet de sélectionner
un scan et d'ajuster indépendamment sa pose avec trois curseurs : x, y, θ.
La transformation appliquée est une rotation antihoraire suivie d'une
translation : `p_monde = R(θ) p_local + t`. Les poses peuvent être sauvegardées
dans `poses.json` sans modifier les mesures d'origine. Une croix indique la
position du capteur et une flèche sa direction +X.

Deux tests synthétiques vérifient le changement d'unités, le signe de Y,
l'exclusion des distances nulles et l'ordre rotation puis translation.
La correspondance visuelle avec la salle et l'orientation réelle reste à
confirmer par l'étudiant ; aucun alignement automatique n'est réalisé.

À la demande du professeur, la visualisation en direct utilise désormais
Teleplot : le programme ESP32 transmet `printScanTeleplot(Serial)` sur USB
à 115200 bauds. L'extension Teleplot 1.1.4 de VS Code a été installée et
l'étudiant confirme que l'affichage fonctionne. La vérification métrique
avec un obstacle connu reste à documenter. Les scans CSV précédemment
enregistrés sont conservés ; leur acquisition nécessite de sélectionner
à nouveau la sortie CSV dans le programme embarqué.

La sortie CSV a ensuite été remise sur l'ESP32 et neuf acquisitions
supplémentaires ont été enregistrées. L'étudiant confirme des rotations
vers la gauche ainsi que des déplacements, sans poses relatives mesurées.
Pour fusionner les 12 fichiers disponibles, nous estimons donc leurs poses
par recalage ICP avec plusieurs initialisations en translation et rotation.
Le scan du 2 novembre à 17:04:38 sert de référence conventionnelle.
Les scans peu compatibles avec cette référence sont réévalués sur l'union
des scans initialement mieux alignés. La pièce rectangulaire peut créer
des poses alternatives ; les diagnostics d'ambiguïté sont exportés.

La fusion contient 5 444 points de distance non nulle, avec provenance et
intensité conservées. Les coordonnées finales sont en mètres, dans un même
repère. Les fichiers d'origine restent inchangés. Le voisinage à moins de
60 cm du capteur est exclu du recalage, mais conservé dans l'export.
Le recouvrement à 20 cm est compris entre 58 % et 100 % ; les résultats
constituent des estimations à vérifier visuellement, pas des poses mesurées.
Les zones masquées ne sont pas complétées artificiellement.

Les résultats sont dans [resultats/](./resultats/README.md). Huit tests passent,
dont trois sur des données synthétiques de recalage. Une vérification des
exports confirme le nombre de points, leur provenance et l'application des
transformations enregistrées. Une validation physique reste nécessaire.

## 3. Construction d'une grid map 2D

## 4. Extraction et manipulation de segments

## 5. Construction d'une carte vectorielle
