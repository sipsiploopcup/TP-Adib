# Comparaison des méthodes de localisation

Seuils : position ≤ 0,10 m et angle ≤ 1°. Trois poses × trois initialisations par jeu.

| Données | Méthode | Erreur position (m) | Erreur angle (°) | Temps (s) | Réussites |
| --- | --- | ---: | ---: | ---: | ---: |
| simulation | grille | 0.0245 | 0.533 | 0.211 | 9/9 |
| simulation | vectorielle | 0.0133 | 0.267 | 1.900 | 9/9 |
| simulation | scan_recherche | 0.0108 | 0.333 | 2.126 | 9/9 |
| simulation | scan_icp | 0.0085 | 0.277 | 0.021 | 9/9 |
| scan_1 | grille | 0.0251 | 0.800 | 0.222 | 9/9 |
| scan_1 | vectorielle | 0.0407 | 0.400 | 2.417 | 9/9 |
| scan_1 | scan_recherche | 0.0200 | 0.800 | 2.065 | 9/9 |
| scan_1 | scan_icp | 0.0158 | 0.775 | 0.014 | 9/9 |
| scan_2 | grille | 0.0559 | 1.000 | 0.282 | 9/9 |
| scan_2 | vectorielle | 0.0224 | 0.000 | 1.142 | 9/9 |
| scan_2 | scan_recherche | 0.0518 | 1.000 | 2.513 | 9/9 |
| scan_2 | scan_icp | 0.0294 | 0.808 | 0.049 | 9/9 |
| scan_3 | grille | 0.0290 | 0.800 | 0.233 | 9/9 |
| scan_3 | vectorielle | 0.0114 | 0.400 | 1.140 | 9/9 |
| scan_3 | scan_recherche | 0.0244 | 0.800 | 2.123 | 9/9 |
| scan_3 | scan_icp | 0.0270 | 0.807 | 0.020 | 9/9 |

La simulation utilise des poses exactes. Les données réelles utilisent des transformations virtuelles sur points réservés ; les poses physiques et leurs incertitudes ne sont pas mesurées.
