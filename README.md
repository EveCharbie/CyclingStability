# CyclingStability


## Coding steps
1. Creation of the bicycle model using SimBRiM + translate it into CasADi/biorbd [bicycle_rider_model.py](bicycle_rider_model.py)
2. Stochastic optimal control problem definition and solution [SOCP_bike_trunk_torque_driven.py](SOCP_bike_trunk_torque_driven.py)


## Requirements

Je ne sais pas il y a de réelles limitations mais pour l'instant ça tourne sous python 3.13.5 de mon coté.

Quelques installations pour faire tourner le code qui produit les équations du mouvements, simule et crée l'animation.
```
pip install symbrim
pip install bicycleparameters
pip install symmeplot
conda install -c conda-forge casadi
conda install -c conda-forge black ipython
```

## Overview du code

_bicycle_rider_mode_ permet de produire les EOMs du vélo basique, montre aussi comment le simualer et l'animer.

_simulator_ est appelé par le _bicycle_rider_mode_ pour produire la simulation. 


## Jules' todo list
- [] Equation du mouvement du modèle le plus simple de SimBRiM
- [] Paramètres d'un vélo basique
- [] Proposer des conditions de validation du modèle

---

## Validations **possibles** du modèle

| Nom                  | Niveau    | Description                                                                                                    | Métriques/Analyses              | Ce que ça demande de faire                                      | Intérêt                                                   | Ref                     |
|----------------------|-----------|----------------------------------------------------------------------------------------------------------------|----------------------------------|---------------------------------------------------------------|-----------------------------------------------------------|-------------------------|
| Stratégies qualitatives | FAIBLE    | Stratégie de maintien de l'équilibre temporelle similaire (braquage, contre-braquage, lean, counter lean)       | Séquences d'actions             | Codifier les mouvements en séquences (jamais fait) + décrire des séquences expé (notre ref) | On manque de construit sur cet aspect qualitatif, ça ira pas loin |                         |
| Cinématique   | MOYENNE   | Produit des quantités angulaires (ex: amplitudes des vitesses angulaires, fréquences) conformes aux mesures expérimentales dans des conditions d'observations similaires (perturbations, suivi de ligne, maintien de l'équilibre) | Petit traitement du signal de données open source              | Extraire les données de ref chez Moore et al., Dialynas et al., Ronné et al. |      | Moore et al., Dialynas et al., Ronné et al. |
|Dynamique / Stabilité  | MOYENNE+  | Produit un mouvement d'une stabilité proche de ceux observés expérimentalement (perturbations, suivi de ligne, maintien de l'équilibre), l'effet de la vitesse est probablement le meilleur point de départ | moment cinétique, temps retour à l'équilibre, cycle limite    | Extraire les données de ref chez Moore et al. (voire perturbations + chute) |                   | Moore et al. + autre         |
| Reproduction des perfs     | FORTE     | Produit une performance de tâche reproduisant les effets : du design du vélo, de la vitesse                     | distance à la ligne                | Un scénario de suivi de ligne                                      |  | Ronné et al.            |
| Stratégie visuelle   | FORTE    | Produit une stratégie visuelle compatible/similaire avec les dernières observations                            | Métriques a définir            | Inclure le mouvement des yeux dans le modèle                           |        |                         |
| Domaine fréquentiel | FAIBLE (en tant que validation, mais fort scientifiquement)  | Produit un comportement fréquentiel vérifiant la théorie de McRuer et Jex                                      | Diagramme de Bode           | Tester le modèle a differentes fréquences de commande    |  | McRuer et Jex           |

Biensûr le mieux c'est de faire des combo de validation, plus on coche de case plus on aura confiance dans le modèle!

## En cas de validation 

Si le modèle reproduit bien la stabilité voire la performance, ça permet ensuite de spéculer sur le lien entre control et charge cognitive, ouvrant la porte à plein de choses :)


## Ressources numériques

- https://moorepants.github.io/learn-multibody-dynamics/nonholonomic-eom.html
- https://mechmotum.github.io/symbrim/tutorials/my_first_bicycle.html
